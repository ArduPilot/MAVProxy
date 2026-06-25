'''
Standalone test/verify harness for the 3D map rendering core.

Exercises terrain.py + elements.py + camera.py directly (in-process) and renders
to a PNG offscreen, or opens the real multiprocess viewer with --gui.

  PYTHONPATH=$PWD python3 -m MAVProxy.modules.mavproxy_map3d.map3d_test \
      --log example_logs/V95_56.bin --out /tmp/map3d_module.png
'''

import argparse
import math
import time

import vtk

from pymavlink import mavutil
from MAVProxy.modules.mavproxy_map import mp_tile
from MAVProxy.modules.mavproxy_map3d.terrain import TerrainManager, R
from MAVProxy.modules.mavproxy_map3d.elements import ElementManager
from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera


def load_log(path):
    m = mavutil.mavlink_connection(path)
    pts = []
    mission = []
    while True:
        msg = m.recv_match(type=['POS', 'CMD'], blocking=False)
        if msg is None:
            break
        if msg.get_type() == 'POS':
            pts.append((msg.Lat, msg.Lng, msg.Alt))
        elif msg.get_type() == 'CMD':
            if msg.Lat != 0 or msg.Lng != 0:
                mission.append((msg.Lat, msg.Lng, msg.Alt, getattr(msg, 'Frame', 3),
                                msg.CId, msg.CNum))
    return pts, mission


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="example_logs/V95_56.bin")
    ap.add_argument("--service", default="MicrosoftSat")
    ap.add_argument("--zexag", type=float, default=1.0)
    ap.add_argument("--pitch", type=float, default=28.0)
    ap.add_argument("--width", type=int, default=1100)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--settle", type=float, default=40.0, help="seconds to wait for tiles")
    ap.add_argument("--out", default="/tmp/map3d_module.png")
    ap.add_argument("--gui", action="store_true")
    args = ap.parse_args()

    pts, mission = load_log(args.log)
    if not pts:
        print("no POS in", args.log)
        return
    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    ground0 = min(p[2] for p in pts)
    minlat, maxlat = min(p[0] for p in pts), max(p[0] for p in pts)
    minlon, maxlon = min(p[1] for p in pts), max(p[1] for p in pts)
    spanlon = math.radians(maxlon - minlon) * R * math.cos(math.radians(lat0))
    spanlat = math.radians(maxlat - minlat) * R
    span = max(spanlon, spanlat, 1000.0)
    print("path=%d mission=%d centroid=(%.5f,%.5f) span=%.0fm" %
          (len(pts), len(mission), lat0, lon0, span))

    if args.gui:
        from MAVProxy.modules.mavproxy_map3d.map3d import Map3D
        m = Map3D(title="map3d test", service=args.service, zexag=args.zexag,
                  width=args.width, height=args.height)
        time.sleep(1.0)
        m.set_origin(lat0, lon0, ground0)
        m.set_home(ground0)
        m.set_path(pts)
        m.set_mission(mission)
        m.look_at(lat0, lon0, ground0, dist=1.6 * span)
        try:
            while m.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            m.close()
        return

    # offscreen: drive the core directly
    ren = vtk.vtkRenderer()
    ren.SetBackground(0.45, 0.62, 0.85)
    mt = mp_tile.MPTile(service=args.service, tile_delay=0.02)
    terrain = TerrainManager(ren, mt, lat0, lon0, zexag=args.zexag,
                             fine_radius=3, screen_h=args.height)
    elements = ElementManager(ren, lat0, lon0, args.zexag)
    elements.set_home(ground0)
    elements.set_path(pts)
    elements.set_mission(mission)

    tc = TerrainCamera(ren.GetActiveCamera(),
                       focal=(0.0, 0.0, ground0 * args.zexag),
                       dist=1.6 * span, yaw=0.0, pitch=args.pitch)
    terrain.update(tc)

    deadline = time.time() + args.settle
    last_change = time.time()
    while time.time() < deadline:
        if terrain.process(tc):
            last_change = time.time()
        settled = (not terrain.inflight and mt.tiles_pending() == 0)
        if settled and time.time() - last_change > 2.0:
            break
        time.sleep(0.1)
    print("tiles=%d inflight=%d pending=%d" %
          (len(terrain.tiles), len(terrain.inflight), mt.tiles_pending()))

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.SetSize(args.width, args.height)
    rw.AddRenderer(ren)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(args.out)
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    terrain.shutdown()
    print("wrote", args.out)


if __name__ == "__main__":
    main()
