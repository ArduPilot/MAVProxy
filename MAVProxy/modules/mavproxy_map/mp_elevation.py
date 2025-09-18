#!/usr/bin/env python3

'''backwards-compatability module - ElevationModel was moved into lib
but we don't want to break existing users

AP_FLAKE8_CLEAN
'''


from MAVProxy.modules.lib.mp_elevation import *  # noqa:F401,F403
