#!/usr/bin/env python
'''
camera view module
Malcolm Gill
Feb 2014
'''

import math
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from cuav.lib import cuav_util
from cuav.camera.cam_params import CameraParams

# documented in common.xml, can't find these constants in code
scale_latlon = 1e-7
scale_hdg = 1e-2
scale_relative_alt = 1e-3

from MAVProxy.modules.lib import mp_module

class CameraViewModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CameraViewModule, self).__init__(mpstate, "cameraview")
        self.add_command('cameraview', self.cmd_cameraview, "camera view")
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.mount_roll = 0
        self.mount_pitch = 0
        self.mount_yaw = 0
        self.height = 0
        self.lat = 0
        self.lon = 0
        self.home_height = 0
        self.hdg = 0
        self.elevation_model = mp_elevation.ElevationModel()
        self.camera_params = CameraParams() # TODO how to get actual camera params
        self.view_settings = mp_settings.MPSettings(
            [ ('r', float, 0.5),
              ('g', float, 0.5),
              ('b', float, 1.0),
            ])
        self.update_col()

    def update_col(self):
        self.col = tuple(int(255*c) for c in (self.view_settings.r, self.view_settings.g, self.view_settings.b))

    def cmd_cameraview(self, args):
        '''camera view commands'''
        state = self
        if args and args[0] == 'set':
            if len(args) < 3:
                state.view_settings.show_all()
            else:
                state.view_settings.set(args[1], args[2])
                state.update_col()
        else:
            print('usage: cameraview set')

    def unload(self):
        '''unload module'''
        pass

    def scale_rc(self, servo, min, max, param):
        '''scale a PWM value'''
        # default to servo range of 1000 to 2000
        min_pwm  = self.get_mav_param('%s_MIN'  % param, 0)
        max_pwm  = self.get_mav_param('%s_MAX'  % param, 0)
        if min_pwm == 0 or max_pwm == 0:
            return 0
        if max_pwm == min_pwm:
            p = 0.0
        else:
            p = (servo-min_pwm) / float(max_pwm-min_pwm)
        v = min + p*(max-min)
        if v < min:
            v = min
        if v > max:
            v = max
        return v

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        state = self
        if m.get_type() == 'GLOBAL_POSITION_INT':
            state.lat, state.lon = m.lat*scale_latlon, m.lon*scale_latlon
            state.hdg = m.hdg*scale_hdg
            agl = state.elevation_model.GetElevation(state.lat, state.lon)
            if agl is not None:
                state.height = m.relative_alt*scale_relative_alt + state.home_height - agl
        elif m.get_type() == 'ATTITUDE':
            state.roll, state.pitch, state.yaw = math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw)
        elif m.get_type() in ['GPS_RAW', 'GPS_RAW_INT']:
            if self.module('wp').wploader.count() > 0:
                home = self.module('wp').wploader.wp(0).x, self.module('wp').wploader.wp(0).y
            else:
                home = [self.master.field('HOME', c)*scale_latlon for c in ['lat', 'lon']]
            old = state.home_height # TODO TMP
            agl = state.elevation_model.GetElevation(*home)
            if agl is None:
                return
            state.home_height = agl

            # TODO TMP
            if state.home_height != old:
                # tridge said to get home pos from wploader,
                # but this is not the same as from master() below...!!
                # using master() gives the right coordinates
                # (i.e. matches GLOBAL_POSITION_INT coords, and $IMHOME in sim_arduplane.sh)
                # and wploader is a bit off
                print('home height changed from',old,'to',state.home_height)
        elif m.get_type() == 'SERVO_OUTPUT_RAW':
            for (axis, attr) in [('ROLL', 'mount_roll'), ('TILT', 'mount_pitch'), ('PAN', 'mount_yaw')]:
                channel = int(self.get_mav_param('MNT_RC_IN_{0}'.format(axis), 0))
                if self.get_mav_param('MNT_STAB_{0}'.format(axis), 0) and channel:
                    # enabled stabilisation on this axis
                    # TODO just guessing that RC_IN_ROLL gives the servo number, but no idea if this is really the case
                    servo = 'servo{0}_raw'.format(channel)
                    centidegrees = self.scale_rc(getattr(m, servo),
                                            self.get_mav_param('MNT_ANGMIN_{0}'.format(axis[:3])),
                                            self.get_mav_param('MNT_ANGMAX_{0}'.format(axis[:3])),
                                            param='RC{0}'.format(channel))
                    setattr(state, attr, centidegrees*0.01)
            #state.mount_roll = min(max(-state.roll,-45),45)#TODO TMP
            #state.mount_yaw = min(max(-state.yaw,-45),45)#TODO TMP
            #state.mount_pitch = min(max(-state.pitch,-45),45)#TODO TMP
        else:
            return
        if self.mpstate.map: # if the map module is loaded, redraw polygon
            # get rid of the old polygon
            self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('CameraView'))

            # camera view polygon determined by projecting corner pixels of the image onto the ground
            pixel_positions = [cuav_util.pixel_position(px[0],px[1], state.height, state.pitch+state.mount_pitch, state.roll+state.mount_roll, state.yaw+state.mount_yaw, state.camera_params) for px in [(0,0), (state.camera_params.xresolution,0), (state.camera_params.xresolution,state.camera_params.yresolution), (0,state.camera_params.yresolution)]]
            if any(pixel_position is None for pixel_position in pixel_positions):
                # at least one of the pixels is not on the ground
                # so it doesn't make sense to try to draw the polygon
                return
            gps_positions = [mp_util.gps_newpos(state.lat, state.lon, math.degrees(math.atan2(*pixel_position)), math.hypot(*pixel_position)) for pixel_position in pixel_positions]

            # draw new polygon
            self.mpstate.map.add_object(mp_slipmap.SlipPolygon('cameraview', gps_positions+[gps_positions[0]], # append first element to close polygon
                                                          layer='CameraView', linewidth=2, colour=state.col))

def init(mpstate):
    '''initialise module'''
    return CameraViewModule(mpstate)
