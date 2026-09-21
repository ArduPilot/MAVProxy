#!/usr/bin/env python3
"""Radiometric decoding contract, projection freshness and display preservation."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from MAVProxy.modules.mavproxy_camera.thermal_stream import parse_metadata, projection_pose
from MAVProxy.modules.mavproxy_camera.thermal_view import ThermalView
from MAVProxy.modules.lib.mp_menu import MPMenuItem


def metadata():
    return dict(schema='apcg.thermal.v1', width=640, height=512, pixel_format='gray16le',
                bits_per_sample=16, frame_id=1, capture_monotonic_us=123456,
                temperature_scale_k=1/64, temperature_offset_k=0, hfov_deg=24.2,
                minimum_c=-273.15, maximum_c=750.834375, rotation_deg=180,
                altitude_datum='AMSL', gimbal_frame='roll_pitch_level_yaw_vehicle',
                telemetry=dict(schema='apcg.telemetry.v1',
                    position=dict(lat_e7=-350000000, lon_e7=1490000000, alt_amsl_m=650, age_ms=100),
                    vehicle_attitude=dict(roll_rad=0, pitch_rad=0, yaw_rad=1, yaw_rate_rad_s=.2, age_ms=100),
                    gimbal_attitude=dict(roll_rad=0, pitch_rad=-.7, yaw_rad=.1, yaw_rate_rad_s=-.1, age_ms=100),
                    velocity=dict(vn_m_s=10, ve_m_s=0, vd_m_s=2, age_ms=100)))


class ThermalTests(unittest.TestCase):
    def test_block_mapping_and_validation(self):
        m=metadata()
        def encoded(m): return (0x41504347).to_bytes(8,'big')+json.dumps(m).encode()
        self.assertEqual(parse_metadata(encoded(m)),m)
        for key,value in [('schema','bad'),('width',320),('bits_per_sample',8),
                          ('frame_id',True),('capture_monotonic_us',0),('hfov_deg',math.nan),
                          ('temperature_scale_k',0),('rotation_deg',90)]:
            bad=copy.deepcopy(m); bad[key]=value
            with self.subTest(key=key), self.assertRaises(ValueError): parse_metadata(encoded(bad))
        for data in [b'', b'\0'*8+b'{}', (2).to_bytes(8,'big')+json.dumps(m).encode(), b'x'*65545]:
            with self.assertRaises(ValueError): parse_metadata(data)

    def test_projection_is_frozen_and_refuses_stale_sources(self):
        m=metadata(); original=copy.deepcopy(m)
        lat,lon,alt,roll,pitch,yaw=projection_pose(m)
        self.assertGreater(lat,-35)
        self.assertAlmostEqual(lon,149)
        self.assertAlmostEqual(alt,649.8)
        self.assertAlmostEqual(yaw,math.degrees(1.11))
        self.assertEqual(m,original)
        for key in ('position','vehicle_attitude','gimbal_attitude'):
            for bad in (None,dict(age_ms=251),dict(age_ms=-1),dict(age_ms=math.nan)):
                m=metadata();m['telemetry'][key]=bad
                self.assertIsNone(projection_pose(m))

    def test_display_and_save_preserve_native_samples(self):
        # Exercise actual viewer methods without launching a GUI process.
        view=ThermalView.__new__(ThermalView)
        view.camera=SimpleNamespace(system_id=42, component_id=100, label=lambda:'test')
        view.image=mock.Mock(); view.image.is_alive.return_value=True; view.image.events.return_value=[]
        import threading
        view.lock=threading.Lock(); view.paused=False; view.error=None; view.last_key=None; view.shown=None
        view.palette='Greyscale'
        view.cursor=None; view.last_readout=None; view.last_title=None
        pixels=np.arange(640*512,dtype=np.uint32).astype(np.uint16).reshape(512,640)
        original=pixels.copy(); m=metadata(); view.latest=(pixels,m)
        view.check_events()
        self.assertEqual(view.image.set_title.call_args.args[0], 'test - Raw Thermal')
        self.assertIn('Min: -273.15 C    Max: 750.83 C',view.image.set_status_text.call_args.args[0])
        np.testing.assert_array_equal(pixels,original)
        display=view.image.set_image.call_args.args[0]
        self.assertEqual(display.dtype,np.uint8)
        self.assertEqual(display.shape,(512,640,3))
        np.testing.assert_array_equal(display[:,:,0],display[:,:,1])
        np.testing.assert_array_equal(display[:,:,1],display[:,:,2])
        greyscale=display.copy()
        self.assertAlmostEqual(view._pixel(0,0),int(pixels[-1,-1])/64-273.15)
        self.assertIn('Pixel (0, 0):',view.image.set_status_text.call_args.args[0])
        view.paused=True; view.latest=(pixels+1,dict(m,frame_id=2)); view.check_events()
        self.assertEqual(view.shown[1]['frame_id'],1)
        for palette in ('Inferno','Turbo','Greyscale'):
            view.image.events.return_value=[MPMenuItem(palette, returnkey='Thermal:Palette:'+palette)]
            view.check_events()
            display=view.image.set_image.call_args.args[0]
            self.assertEqual(view.shown[1]['frame_id'],1)
            np.testing.assert_array_equal(view.shown[0],original)
            if palette == 'Greyscale':
                np.testing.assert_array_equal(display,greyscale)
            else:
                self.assertFalse(np.array_equal(display[:,:,0],display[:,:,1]))
        view.image.events.return_value=[]
        view.paused=False
        view.check_events()
        self.assertEqual(view.shown[1]['frame_id'],2)
        self.assertIn('Pixel (0, 0): %.2f C' % (int(view.shown[0][-1,-1])/64-273.15),
                      view.image.set_status_text.call_args.args[0])
        self.assertIsNone(view._pixel(-1,0))
        self.assertIn('Hover over',view.image.set_status_text.call_args.args[0])
        view.shown=(original,m)
        with tempfile.TemporaryDirectory() as directory:
            view.module=SimpleNamespace(logdir=directory)
            view._save()
            raw=next(Path(directory).rglob('*.bin')); side=raw.with_suffix('.json')
            self.assertEqual(raw.read_bytes(),original.astype('<u2').tobytes())
            self.assertEqual(json.loads(side.read_text()),m)


if __name__=='__main__': unittest.main()
