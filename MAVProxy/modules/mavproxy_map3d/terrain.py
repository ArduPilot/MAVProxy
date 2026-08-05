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
import warnings

import numpy as np
import vtk
# import via vtkmodules, not the vtk.* aliases: vtk is a plain module that fakes
# a package with __path__, which pyinstaller cannot follow into a frozen build
from vtkmodules.util import numpy_support

from quantized_mesh_tile import decode as qmt_decode
from quantized_mesh_tile.global_geodetic import GlobalGeodetic

from MAVProxy.modules.mavproxy_map import mp_tile

R = 6378137.0
QUANTIZED_BASE = "https://plot.ardupilot.org/quantized"
CACHE_DIR = os.path.join(mp_tile.default_cache_path(), "quantized")

# a 3D view drapes hundreds of imagery tiles before it looks like anything, and
# the fetches are latency bound rather than bandwidth bound, so run them in
# parallel. One at a time takes minutes on a high latency link such as WSL2.
TILE_DOWNLOAD_THREADS = 8

# ArduPilot tiles include the optional lighting extension. This decoder warns
# when it skips that extension, but map3d computes its own VTK normals anyway.
warnings.filterwarnings(
    "ignore",
    message=r"Skipping unsupported terrain tile extension \(id=1, length=\d+ bytes\).*",
    category=UserWarning,
    module=r"quantized_mesh_tile\.terrain")


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
        # publish atomically under a unique name: the viewer child process and
        # the module share this cache, so a reader must never see a partial tile
        tmppath = "%s.tmp.%u.%u" % (path, os.getpid(), threading.get_ident())
        with open(tmppath, "wb") as f:
            f.write(data)
        os.replace(tmppath, path)
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


def sample_terrain(lat, lon, zoom=12, cache_only=False):
    '''return terrain elevation (m AMSL) at lat/lon from the quantized mesh
    (same source we render), or None. Decoded tiles are cached. Assumes the
    regular grid mesh ArduPilot publishes; falls back to the tile mean for a
    non-grid tile.

    cache_only returns None rather than fetching/decoding a missing tile, so
    callers on a latency-sensitive thread can defer the work.'''
    g = GlobalGeodetic(True)
    x, y = g.LonLatToTile(lon, lat, zoom)
    key = (zoom, x, y)
    dec = _sample_cache.get(key)
    if dec is None:
        if cache_only:
            return None
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
    def __init__(self, decoded, lat0, lon0, zexag, z_offset,
                 brightness=1.25, shading=True, wireframe=False):
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
        self.locator = None
        bounds = self.geo.GetBounds()
        self.z_min = bounds[4]
        self.z_max = bounds[5]
        self.tcoords = vtk.vtkFloatArray()
        self.tcoords.SetNumberOfComponents(2)
        self.tcoords.SetNumberOfTuples(nv)
        self.geo.GetPointData().SetTCoords(self.tcoords)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.geo)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.apply_render_settings(brightness, shading, wireframe)
        self.tex_key = None     # currently applied drape key
        self.want_key = None    # most recently requested drape key

    def apply_render_settings(self, brightness, shading, wireframe):
        prop = self.actor.GetProperty()
        brightness = min(2.0, max(0.25, float(brightness)))
        prop.SetAmbient(min(1.0, 0.35 * brightness))
        prop.SetDiffuse(min(1.0, 0.8 * brightness))
        prop.SetColor(*(min(1.0, c * brightness)
                        for c in (0.55, 0.6, 0.5)))
        prop.SetLighting(bool(shading))
        if shading:
            prop.SetInterpolationToPhong()
        if wireframe:
            prop.SetRepresentationToWireframe()
        else:
            prop.SetRepresentationToSurface()

    def height_at(self, e, n):
        '''Return the rendered world Z at an east/north point, or None.'''
        if self.locator is None:
            self.locator = vtk.vtkStaticCellLocator()
            self.locator.SetDataSet(self.geo)
            self.locator.BuildLocator()
        intersections = vtk.vtkPoints()
        cell_ids = vtk.vtkIdList()
        margin = max(1000.0, self.z_max - self.z_min)
        found = self.locator.IntersectWithLine(
            (e, n, self.z_max + margin),
            (e, n, self.z_min - margin),
            1.0e-6, intersections, cell_ids)
        if not found or intersections.GetNumberOfPoints() == 0:
            return None
        return max(intersections.GetPoint(i)[2]
                   for i in range(intersections.GetNumberOfPoints()))

    def texture_request(self, campos, screen_h, mt, fov_deg=30.0):
        '''Compute the desired full-tile drape (cheap, main thread).

        A texture must cover the complete mesh tile. Applying imagery for only
        the camera-visible subsection makes VTK clamp the texture coordinates
        of the remaining vertices to an edge pixel, producing long stretched
        streaks in low-angle and FPV views.
        '''
        W, S, E, N = self.bbox
        mid = 0.5 * (N + S)
        dist = math.sqrt(sum((campos[i] - self.center[i]) ** 2 for i in range(3)))
        gsd = max(0.3, dist * 2.0 * math.tan(math.radians(fov_deg) / 2.0) / screen_h)
        img_zoom = int(round(math.log2(
            2 * math.pi * R * math.cos(math.radians(mid)) / (gsd * 256))))
        img_zoom = max(mt.min_zoom, min(mt.max_zoom, img_zoom))
        tile_w_m = math.radians(E - W) * R * math.cos(math.radians(mid))
        tex_w = min(1024, max(256, int(tile_w_m / gsd)))
        key = (img_zoom, tex_w)
        if key == self.tex_key:
            return None
        return key, (W, S, E, N, img_zoom, tex_w)

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


def build_texture_image(mt, lock, vw, vs, ve, vn, img_zoom, tex_w, priority=0.0):
    '''worker thread: assemble the draped Mercator image for a tile. Does NOT
    block on downloads (mp_tile fetches imagery on its own thread); missing
    imagery comes back as placeholders and is refreshed once downloads settle.

    priority is the tile's rank from the camera, so the imagery queue drains
    from the middle of the view outwards however the workers interleave.'''
    dlon = math.radians(ve - vw)
    C = tex_w / dlon
    tex_h = max(1, int(round(C * (mp_tile.mercator_y(vn) - mp_tile.mercator_y(vs)))))
    gw = R * math.cos(math.radians(vn)) * dlon
    with lock:   # mp_tile's in-memory cache is not thread-safe
        img = mt.area_to_image(vn, vw, tex_w, tex_h, gw, zoom=img_zoom,
                               priority=priority)
    return img


class TerrainManager:
    def __init__(self, renderer, mt, lat0, lon0, zexag=1.0,
                 zoom_fine=12, lod_min=8, ring=1, fine_radius=2, screen_h=800,
                 brightness=1.25, shading=True, wireframe=False):
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
        self.brightness = brightness
        self.shading = shading
        self.wireframe = wireframe
        self.mesh_revision = 0
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

    def set_render_settings(self, brightness, shading, wireframe):
        self.brightness = min(2.0, max(0.25, float(brightness)))
        self.shading = bool(shading)
        self.wireframe = bool(wireframe)
        for tile in self.tiles.values():
            tile.apply_render_settings(self.brightness, self.shading,
                                       self.wireframe)

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
                    (_tilekey, _tkey, params, priority) = payload
                    (vw, vs, ve, vn, img_zoom, tex_w) = params
                    img = build_texture_image(self.mt, self.tex_lock,
                                              vw, vs, ve, vn, img_zoom, tex_w,
                                              priority)
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

    def height_at(self, lat, lon):
        '''Return rendered terrain world Z at lat/lon, preferring fine tiles.'''
        e, n, _ = enu(lat, lon, 0.0, self.lat0, self.lon0)
        for key, tile in sorted(self.tiles.items(), reverse=True):
            west, south, east, north = tile.bbox
            if not (west <= lon <= east and south <= lat <= north):
                continue
            height = tile.height_at(e, n)
            if height is not None:
                return height
        return None

    def update(self, tc):
        '''called on camera settle: page in/out terrain + request textures'''
        foclat, foclon = self.focal_latlon(tc)
        want = self.desired_set(foclat, foclon)
        # page in missing terrain

        def tile_priority(key):
            z, x, y = key
            west, south, east, north = self.g.TileBounds(x, y, z)
            clat = 0.5 * (south + north)
            clon = 0.5 * (west + east)
            dlat = clat - foclat
            dlon = (clon - foclon) * math.cos(math.radians(foclat))
            return (-z, dlat * dlat + dlon * dlon)

        for (z, x, y) in sorted(want, key=tile_priority):
            jid = ("D", z, x, y)
            if (z, x, y) not in self.tiles and jid not in self.inflight:
                self.inflight.add(jid)
                self.jobs.put(("decode", jid, (z, x, y)))
        # page out far tiles
        removed_tile = False
        for key in list(self.tiles.keys()):
            if key not in want:
                self.ren.RemoveActor(self.tiles[key].actor)
                del self.tiles[key]
                removed_tile = True
        if removed_tile:
            self.mesh_revision += 1
        # request textures for present tiles (at most one outstanding per tile),
        # nearest the camera first. The rank goes down to mp_tile so the imagery
        # queue is ordered too: several workers fetch tiles concurrently, so the
        # order jobs are queued in does not by itself decide the download order
        fov_deg = tc.cam.GetViewAngle()
        for rank, key in enumerate(sorted(self.tiles.keys(), key=tile_priority)):
            tile = self.tiles[key]
            req = tile.texture_request(tc.pos, self.screen_h, self.mt, fov_deg)
            if req is None:
                continue
            tkey, params = req
            jid = ("T", key)
            if jid in self.inflight:
                continue
            tile.want_key = tkey
            self.inflight.add(jid)
            self.jobs.put(("texture", jid, (key, tkey, params, rank)))

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
                    tile = Tile(payload, self.lat0, self.lon0, self.zexag, zoff,
                                self.brightness, self.shading, self.wireframe)
                except Exception:
                    continue
                self.tiles[(z, x, y)] = tile
                self.ren.AddActor(tile.actor)
                self.mesh_revision += 1
                changed = True
            elif kind == "texture":
                img, (tilekey, tkey, params, _priority) = payload
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
