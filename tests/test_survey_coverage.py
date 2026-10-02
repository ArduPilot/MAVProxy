"""Exposure geometry pairing and duplicate/out-of-order capture coverage."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map.mp_slipmap_util import SlipPolygon
from MAVProxy.modules.mavproxy_camera.survey import SurveyCoverage, footprint


def message(kind, stamp=123, **kwargs):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: 1,
                           get_srcComponent=lambda: 100, time_boot_ms=stamp, **kwargs)


def fov(stamp=123):
    return message('CAMERA_FOV_STATUS', stamp, q=[math.sqrt(.5),0,-math.sqrt(.5),0],
                   hfov=24.2,vfov=19.4,alt_camera=1000000,alt_image=600000,
                   lat_camera=-350000000,lon_camera=1490000000)


def capture(stamp=123, result=1):
    return message('CAMERA_IMAGE_CAPTURED',stamp,capture_result=result,time_utc=987654321,image_index=1)


class SurveyTests(unittest.TestCase):
    def test_geometry(self):
        points=footprint(fov())
        self.assertEqual(len(points),5)
        width=abs(points[1][1]-points[0][1])*111319.5*math.cos(math.radians(35))
        self.assertAlmostEqual(width,171.51,delta=.2)
        invalid=fov(); invalid.q=[1,0,0,0]
        self.assertIsNone(footprint(invalid))
        invalid=fov(); invalid.alt_image=invalid.alt_camera
        self.assertIsNone(footprint(invalid))

    def test_pairing(self):
        module=SimpleNamespace(mpstate=SimpleNamespace(map=None))
        coverage=SurveyCoverage(module); coverage.draw=Mock()
        coverage.packet(capture()); self.assertFalse(coverage.images)
        coverage.packet(fov()); self.assertEqual(len(coverage.images),1)
        coverage.packet(fov()); coverage.packet(capture())
        coverage.draw.assert_called_once()
        coverage.packet(fov(124)); coverage.packet(capture(125))
        self.assertEqual(len(coverage.images),1)
        coverage.packet(capture(124,0)); self.assertEqual(len(coverage.images),1)
        coverage.limit=1
        coverage.packet(fov(125)); self.assertEqual(len(coverage.images),1)
        self.assertEqual(next(iter(coverage.images))[3],125)

class SurveyRenderingTests(unittest.TestCase):
    def setUp(self):
        self.module=SimpleNamespace(mpstate=SimpleNamespace(map=None),
            _selected_camera=lambda: SimpleNamespace(system_id=1,component_id=100))
        self.coverage=SurveyCoverage(self.module)
        self.wx=patch.object(mp_util,'has_wxpython',True)
        self.wx.start()
        self.addCleanup(self.wx.stop)

    def test_map_opened_after_capture_and_replaced(self):
        self.coverage.packet(fov()); self.coverage.packet(capture())
        self.assertEqual(len(self.coverage.images),1)
        display=Mock(); self.module.mpstate.map=display
        self.coverage.idle()
        display.add_object.assert_called_once()
        polygon=display.add_object.call_args.args[0]
        self.assertIsInstance(polygon,SlipPolygon)
        self.assertEqual(polygon.fill_alpha,.18)
        self.assertTrue(polygon._showlines)
        self.coverage.idle()
        display.add_object.assert_called_once()
        replacement=Mock(); self.module.mpstate.map=replacement
        self.coverage.idle(); replacement.add_object.assert_called_once()
        self.coverage.command(['hide'])
        replacement.remove_object.assert_called_once()
        self.coverage.command(['alpha','.5'])
        replacement.add_object.assert_called_once()  # hidden stays hidden
        self.coverage.command(['show'])
        self.assertEqual(replacement.add_object.call_args.args[0].fill_alpha,.5)
        for value in ('nan','-1','2'):
            with self.assertRaises(ValueError): self.coverage.command(['alpha',value])

    def test_delivered_polygon_changes_map_pixels(self):
        display=Mock(); self.module.mpstate.map=display
        self.coverage.packet(capture()); self.coverage.packet(fov())
        polygon=display.add_object.call_args.args[0]
        minlat,minlon,dlat,dlon=polygon.bounds()
        mapper=lambda p: (int(10+60*(p[1]-minlon)/dlon), int(10+60*(p[0]-minlat)/dlat))
        img=np.full((80,80,3),(40,80,40),dtype=np.uint8)
        original=img.copy()
        polygon.draw(img,mapper,polygon.bounds())
        self.assertFalse(np.array_equal(img[40,40],original[40,40]))
        np.testing.assert_array_equal(img[0,0],original[0,0])
        once=img[40,40].copy(); polygon.draw(img,mapper,polygon.bounds())
        self.assertFalse(np.array_equal(img[40,40],once))  # overlapping captures accumulate
        # Same renderer must safely clip footprints crossing the map edge.
        polygon.draw(img,lambda p:(mapper(p)[0]-30,mapper(p)[1]-30),polygon.bounds())

if __name__=='__main__': unittest.main()
