'''
Async terrain manager for the 3D map.

Fetches ArduPilot quantized-mesh terrain tiles, decodes them, and drapes
satellite imagery (via mp_tile) over them with view-dependent texture LOD.
Network + decode + imagery happen on a background worker thread; VTK object
creation happens on the main GUI thread (drained via process()).
'''

import math
import os
import queue
import threading
import urllib.request

import numpy as np
import vtk
from vtk.util import numpy_support

from quantized_mesh_tile import decode as qmt_decode
from quantized_mesh_tile.global_geodetic import GlobalGeodetic

from MAVProxy.modules.mavproxy_map import mp_tile

R = 6378137.0
QUANTIZED_BASE = "https://plot.ardupilot.org/quantized"
CACHE_DIR = os.path.join(os.environ.get("HOME", "."), ".tilecache", "quantized")


def enu(lat, lon, h, lat0, lon0):
    e = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * R
    return e, n, h


def np_rgb_to_texture(img):
    h, w = img.shape[:2]
    flat = np.ascontiguousarray(img[::-1])    # vtkImageData origin is bottom-left
    vimg = vtk.vtkImageData()
    vimg.SetDimensions(w, h, 1)
    arr = numpy_support.numpy_to_vtk(flat.reshape(-1, 3), deep=1,
                                     array_type=vtk.VTK_UNSIGNED_CHAR)
    arr.SetNumberOfComponents(3)
    vimg.GetPointData().SetScalars(arr)
    tex = vtk.vtkTexture()
    tex.SetInputData(vimg)
    tex.InterpolateOn()
    tex.EdgeClampOn()
    return tex


def fetch_terrain_tile(z, x, y):
    path = os.path.join(CACHE_DIR, str(z), str(x), "%d.terrain" % y)
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        url = "%s/%d/%d/%d.terrain" % (QUANTIZED_BASE, z, x, y)
        req = urllib.request.Request(url, headers={"Accept": "application/octet-stream"})
        data = urllib.request.urlopen(req, timeout=30).read()
        with open(path, "wb") as f:
            f.write(data)
    with open(path, "rb") as f:
        gz = f.read(2) == b"\x1f\x8b"
    return path, gz


def decode_terrain(z, x, y):
    '''worker-safe: return dict(bbox, verts, idx) using numpy only'''
    g = GlobalGeodetic(True)
    bbox = g.TileBounds(x, y, z)
    path, gz = fetch_terrain_tile(z, x, y)
    tile = qmt_decode(path, bbox, gzipped=gz)
    verts = np.array(tile.getVerticesCoordinates())
    nv = len(verts)
    mask = 0xFFFF if nv <= 0x10000 else 0xFFFFFFFF
    idx = (np.array(tile.indices) & mask).astype(np.int64).reshape(-1, 3)
    return {"bbox": bbox, "verts": verts, "idx": idx}


_sample_cache = {}


def sample_terrain(lat, lon, zoom=12):
    '''return terrain elevation (m AMSL) at lat/lon from the quantized mesh
    (same source we render), or None. Decoded tiles are cached. Assumes the
    regular grid mesh ArduPilot publishes; falls back to the tile mean for a
    non-grid tile.'''
    g = GlobalGeodetic(True)
    x, y = g.LonLatToTile(lon, lat, zoom)
    key = (zoom, x, y)
    dec = _sample_cache.get(key)
    if dec is None:
        try:
            dec = decode_terrain(zoom, x, y)
        except Exception:
            return None
        _sample_cache[key] = dec
    (W, S, E, N) = dec["bbox"]
    verts = dec["verts"]
    n = int(round(len(verts) ** 0.5))
    if n * n != len(verts) or E == W or N == S:
        return float(np.mean(verts[:, 2]))
    # vertex grid is row-major, row 0 = north edge, col 0 = west edge
    fx = min(n - 1, max(0.0, (lon - W) / (E - W) * (n - 1)))
    fy = min(n - 1, max(0.0, (N - lat) / (N - S) * (n - 1)))
    x0, y0 = int(fx), int(fy)
    x1, y1 = min(n - 1, x0 + 1), min(n - 1, y0 + 1)
    tx, ty = fx - x0, fy - y0
    h = verts[:, 2]
    top = h[y0 * n + x0] * (1 - tx) + h[y0 * n + x1] * tx
    bot = h[y1 * n + x0] * (1 - tx) + h[y1 * n + x1] * tx
    return float(top * (1 - ty) + bot * ty)


def lod_tile_set(g, z_fine, fx0, fx1, fy0, fy1, z_min, ring):
    '''nested quadtree LOD: fine tiles, then coarser rings. Returns set of (z,x,y)'''
    out = set((z_fine, x, y)
              for x in range(fx0, fx1 + 1) for y in range(fy0, fy1 + 1))
    hx0, hx1, hy0, hy1 = fx0, fx1, fy0, fy1
    z = z_fine
    while z - 1 >= z_min:
        z -= 1
        skx0, skx1 = (hx0 + 1) // 2, (hx1 - 1) // 2
        sky0, sky1 = (hy0 + 1) // 2, (hy1 - 1) // 2
        ox0, ox1 = hx0 // 2 - ring, hx1 // 2 + ring
        oy0, oy1 = hy0 // 2 - ring, hy1 // 2 + ring
        for x in range(ox0, ox1 + 1):
            for y in range(oy0, oy1 + 1):
                if skx0 <= x <= skx1 and sky0 <= y <= sky1:
                    continue
                out.add((z, x, y))
        hx0, hx1, hy0, hy1 = ox0, ox1, oy0, oy1
    return out


class Tile:
    '''one terrain tile (main-thread VTK objects + view-dependent drape)'''
    def __init__(self, decoded, lat0, lon0, zexag, z_offset):
        self.bbox = decoded["bbox"]                 # (W,S,E,N)
        self.verts = decoded["verts"]
        idx = decoded["idx"]
        self.zoff = z_offset
        nv = len(self.verts)

        pts = vtk.vtkPoints()
        pts.SetNumberOfPoints(nv)
        for i in range(nv):
            lon, lat, h = self.verts[i]
            e, n, u = enu(lat, lon, h, lat0, lon0)
            pts.SetPoint(i, e, n, u * zexag + z_offset)
        cells = vtk.vtkCellArray()
        for tri in idx:
            cells.InsertNextCell(3, [int(tri[0]), int(tri[1]), int(tri[2])])
        clat = 0.5 * (self.bbox[1] + self.bbox[3])
        clon = 0.5 * (self.bbox[0] + self.bbox[2])
        ce, cn, cu = enu(clat, clon, float(np.mean(self.verts[:, 2])), lat0, lon0)
        self.center = (ce, cn, cu * zexag + z_offset)

        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetPolys(cells)
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.SplittingOff()
        normals.Update()
        self.geo = normals.GetOutput()
        self.tcoords = vtk.vtkFloatArray()
        self.tcoords.SetNumberOfComponents(2)
        self.tcoords.SetNumberOfTuples(nv)
        self.geo.GetPointData().SetTCoords(self.tcoords)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.geo)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetAmbient(0.35)
        self.actor.GetProperty().SetDiffuse(0.8)
        self.actor.GetProperty().SetColor(0.55, 0.6, 0.5)   # muted land until textured
        self.tex_key = None     # currently applied drape key
        self.want_key = None    # most recently requested drape key

    def texture_request(self, campos, foclat, foclon, ext_m, screen_h, mt, fov_deg=30.0):
        '''compute the desired drape (cheap, main thread). Returns (key, params)
        or None if unchanged.'''
        W, S, E, N = self.bbox
        ext_lat = math.degrees(ext_m / R)
        ext_lon = math.degrees(ext_m / (R * math.cos(math.radians(foclat))))
        vw, ve = max(W, foclon - ext_lon), min(E, foclon + ext_lon)
        vs, vn = max(S, foclat - ext_lat), min(N, foclat + ext_lat)
        if ve <= vw or vn <= vs:
            vw, ve, vs, vn = W, E, S, N
        mid = 0.5 * (vn + vs)
        dist = math.sqrt(sum((campos[i] - self.center[i]) ** 2 for i in range(3)))
        gsd = max(0.3, dist * 2.0 * math.tan(math.radians(fov_deg) / 2.0) / screen_h)
        img_zoom = int(round(math.log2(
            2 * math.pi * R * math.cos(math.radians(mid)) / (gsd * 256))))
        img_zoom = max(mt.min_zoom, min(mt.max_zoom, img_zoom))
        sub_w_m = math.radians(ve - vw) * R * math.cos(math.radians(mid))
        tex_w = min(1024, max(256, int(sub_w_m / gsd)))
        key = (round(vw, 4), round(vs, 4), round(ve, 4), round(vn, 4), img_zoom, tex_w)
        if key == self.tex_key:
            return None
        return key, (vw, vs, ve, vn, img_zoom, tex_w)

    def apply_texture(self, img, vw, vs, ve, vn, tex_w, key):
        '''main thread: attach the draped image and recompute UVs'''
        dlon = math.radians(ve - vw)
        C = tex_w / dlon
        tex_h = img.shape[0]
        myn = mp_tile.mercator_y(vn)
        for i in range(len(self.verts)):
            lon, lat, _ = self.verts[i]
            x = C * math.radians(lon - vw)
            y = C * (myn - mp_tile.mercator_y(lat))
            u = min(1.0, max(0.0, x / tex_w))
            v = min(1.0, max(0.0, 1.0 - y / tex_h))
            self.tcoords.SetTuple2(i, u, v)
        self.tcoords.Modified()
        self.actor.SetTexture(np_rgb_to_texture(img))
        self.tex_key = key


def build_texture_image(mt, lock, vw, vs, ve, vn, img_zoom, tex_w):
    '''worker thread: assemble the draped Mercator image for a sub-rect. Does NOT
    block on downloads (mp_tile fetches imagery on its own thread); missing
    imagery comes back as placeholders and is refreshed once downloads settle.'''
    dlon = math.radians(ve - vw)
    C = tex_w / dlon
    tex_h = max(1, int(round(C * (mp_tile.mercator_y(vn) - mp_tile.mercator_y(vs)))))
    gw = R * math.cos(math.radians(vn)) * dlon
    with lock:   # mp_tile's in-memory cache is not thread-safe
        img = mt.area_to_image(vn, vw, tex_w, tex_h, gw, zoom=img_zoom)
    return img


class TerrainManager:
    def __init__(self, renderer, mt, lat0, lon0, zexag=1.0,
                 zoom_fine=12, lod_min=8, ring=1, fine_radius=2, screen_h=800):
        self.ren = renderer
        self.mt = mt
        self.lat0 = lat0
        self.lon0 = lon0
        self.zexag = zexag
        self.zoom_fine = zoom_fine
        self.lod_min = lod_min
        self.ring = ring
        self.fine_radius = fine_radius
        self.screen_h = screen_h
        self.g = GlobalGeodetic(True)
        self.tiles = {}                 # (z,x,y) -> Tile
        self.inflight = set()           # jobs queued/running
        self.jobs = queue.Queue()
        self.results = queue.Queue()
        self.tex_lock = threading.Lock()
        self.prev_pending = 0
        self.last_tc = None
        self.stop = False
        self.workers = [threading.Thread(target=self._worker, daemon=True)
                        for _ in range(6)]
        for w in self.workers:
            w.start()

    def _worker(self):
        while not self.stop:
            try:
                kind, jid, payload = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == "decode":
                    (z, x, y) = payload
                    self.results.put(("decode", jid, decode_terrain(z, x, y)))
                elif kind == "texture":
                    (_tilekey, _tkey, params) = payload
                    (vw, vs, ve, vn, img_zoom, tex_w) = params
                    img = build_texture_image(self.mt, self.tex_lock,
                                              vw, vs, ve, vn, img_zoom, tex_w)
                    self.results.put(("texture", jid, (img, payload)))
            except Exception as e:
                self.results.put(("error", jid, e))

    def focal_latlon(self, tc):
        foclat = self.lat0 + math.degrees(tc.focal[1] / R)
        foclon = self.lon0 + math.degrees(tc.focal[0] / (R * math.cos(math.radians(self.lat0))))
        return foclat, foclon

    def desired_set(self, foclat, foclon):
        fx, fy = self.g.LonLatToTile(foclon, foclat, self.zoom_fine)
        r = self.fine_radius
        z_min = self.lod_min if self.ring > 0 else self.zoom_fine
        raw = lod_tile_set(self.g, self.zoom_fine, fx - r, fx + r, fy - r, fy + r,
                           z_min, self.ring)
        # clamp/wrap to the valid EPSG:4326 TMS tile range (y bounded, x wraps)
        out = set()
        for (z, x, y) in raw:
            ny = self.g.GetNumberOfYTilesAtZoom(z)
            nx = self.g.GetNumberOfXTilesAtZoom(z)
            if y < 0 or y >= ny:
                continue
            out.add((z, x % nx, y))
        return out

    def update(self, tc):
        '''called on camera settle: page in/out terrain + request textures'''
        foclat, foclon = self.focal_latlon(tc)
        want = self.desired_set(foclat, foclon)
        # page in missing terrain
        for (z, x, y) in want:
            jid = ("D", z, x, y)
            if (z, x, y) not in self.tiles and jid not in self.inflight:
                self.inflight.add(jid)
                self.jobs.put(("decode", jid, (z, x, y)))
        # page out far tiles
        for key in list(self.tiles.keys()):
            if key not in want:
                self.ren.RemoveActor(self.tiles[key].actor)
                del self.tiles[key]
        # request textures for present tiles (at most one outstanding per tile)
        ext_m = tc.dist * 0.8
        for key, tile in self.tiles.items():
            req = tile.texture_request(tc.pos, foclat, foclon, ext_m, self.screen_h, self.mt)
            if req is None:
                continue
            tkey, params = req
            jid = ("T", key)
            if jid in self.inflight:
                continue
            tile.want_key = tkey
            self.inflight.add(jid)
            self.jobs.put(("texture", jid, (key, tkey, params)))

    def process(self, tc):
        '''main thread: drain worker results, build/update VTK. Returns True if
        anything changed (caller should render).'''
        self.last_tc = tc
        changed = False
        # once imagery downloads settle, re-drape so placeholder tiles sharpen
        pend = self.mt.tiles_pending()
        if self.prev_pending > 0 and pend == 0:
            for t in self.tiles.values():
                t.tex_key = None
            self.update(tc)
        self.prev_pending = pend
        want = self.desired_set(*self.focal_latlon(tc)) if tc is not None else None
        for _ in range(64):
            try:
                kind, jid, payload = self.results.get_nowait()
            except queue.Empty:
                break
            self.inflight.discard(jid)
            if kind == "decode":
                (z, x, y) = jid[1:]
                if want is not None and (z, x, y) not in want:
                    continue   # camera moved on; don't build a tile we'll drop
                zoff = -(self.zoom_fine - z) * 3.0
                try:
                    tile = Tile(payload, self.lat0, self.lon0, self.zexag, zoff)
                except Exception:
                    continue
                self.tiles[(z, x, y)] = tile
                self.ren.AddActor(tile.actor)
                changed = True
            elif kind == "texture":
                img, (tilekey, tkey, params) = payload
                (vw, vs, ve, vn, img_zoom, tex_w) = params
                tile = self.tiles.get(tilekey)
                if tile is not None and tkey == tile.want_key:
                    tile.apply_texture(img, vw, vs, ve, vn, tex_w, tkey)
                    changed = True
        # after building new tiles, request their textures next update
        if changed and tc is not None:
            self.update(tc)
        return changed

    def shutdown(self):
        self.stop = True
