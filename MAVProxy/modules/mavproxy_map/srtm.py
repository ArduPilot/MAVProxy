#!/usr/bin/env python3

'''backwards-compatability module - srtm was moved into lib
but we don't want to break existing users

AP_FLAKE8_CLEAN
'''


from MAVProxy.modules.lib.srtm import *  # noqa:F401,F403
