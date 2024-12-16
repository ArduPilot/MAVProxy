#!/usr/bin/env python3

'''backwards-compatability module - ElevationModel was moved into lib
but we don't want to break existing users
'''


from MAVProxy.modules.lib.mp_elevation import *
