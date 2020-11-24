'''
MAVProxy sailing dashboard module
Rhys Mainwaring
November 2020
'''

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib.wxsaildash import SailingDashboard
from MAVProxy.modules.lib.wxsaildash_util import WindReference, SpeedUnit, WindAngleAndSpeed, WaterSpeedAndHeading

import time

class SailModule(mp_module.MPModule):
    '''SailModule provides a dashboard to display sailing instrument data'''

    def __init__(self, mpstate):
        super(SailModule, self).__init__(mpstate, "sail", "sailing module")

        # dashboard GUI
        self.sail_dash = SailingDashboard(title="Sailing Dashboard")

        # mavlink messages
        self.wind = None
        self.vfr_hud = None

        # data
        self.wnd_ang_app = 0.0
        self.wnd_spd_app = 0.0
        self.wnd_dir_tru = 0.0
        self.wnd_ang_tru = 0.0
        self.wnd_spd_tru = 0.0
        self.heading = 0.0
        self.ground_speed = 0.0

        # control update rate to GUI
        self._msg_list = []
        self._fps = 10.0
        self._last_send = 0.0
        self._send_delay = (1.0/self._fps) * 0.9

        # commands
        self.add_command('sail', self.cmd_sail, "sailing dashboard")

    def cmd_sail(self, args):
        '''Control behaviour of the module'''

        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        else:
            print(self.usage())

    def usage(self):
        '''Show help on command line options'''

        return "Usage: sail <status>"

    def status(self):
        '''Returns information about the sailing state'''

        return({
            "SAIL": {
                "wnd_ang_app": self.wnd_ang_app,
                "wnd_spd_app": self.wnd_spd_app
                },
            "WIND": {
                "direction": self.wind.direction,
                "speed": self.wind.speed,
                "speed_z": self.wind.speed_z
                },
            "VFR_HUD": {
                "airspeed": self.vfr_hud.airspeed,
                "groundspeed": self.vfr_hud.groundspeed,
                "heading": self.vfr_hud.heading,
                "throttle": self.vfr_hud.throttle,
                "alt": self.vfr_hud.alt,
                "climb": self.vfr_hud.climb
                },
            })

    def mavlink_packet(self, m):
        '''Handle a mavlink packet'''

        if m.get_type() == 'NAMED_VALUE_FLOAT':
            if m.name == 'AppWndDir':
                self.wnd_ang_app = m.value
            elif m.name == 'AppWndSpd':
                self.wnd_spd_app = m.value

            # apparent wind: convert speed units and append to msg list
            angle = self.wnd_ang_app
            speed = self.wnd_spd_app
            reference = WindReference.RELATIVE
            unit = SpeedUnit.KNOTS
            speed = SailModule._convert_speed_units('m/s', SpeedUnit.KNOTS, speed)
            self._msg_list.append(WindAngleAndSpeed(reference, angle, speed, unit))

        elif m.get_type() == 'SAIL':
            self.sail = m
        elif m.get_type() == 'WIND':
            self.wind = m
            self.wnd_dir_tru = m.direction
            self.wnd_spd_tru = m.speed

            # convert wind direction to angle
            self.wnd_ang_tru = SailModule._wind_direction_to_angle(self.wnd_dir_tru, self.heading)

            # true wind: convert speed units and append to msg list
            angle = self.wnd_ang_tru
            speed = self.wnd_spd_tru
            reference = WindReference.TRUE
            unit = SpeedUnit.KNOTS
            speed = SailModule._convert_speed_units('m/s', SpeedUnit.KNOTS, speed)
            self._msg_list.append(WindAngleAndSpeed(reference, angle, speed, unit))

        elif m.get_type() == 'VFR_HUD':
            self.vfr_hud = m
            self.heading = m.heading
            self.ground_speed = m.groundspeed

            # water speed and true heading: convert speed units and append to msg list
            heading_true = self.heading
            water_speed = SailModule._convert_speed_units('m/s', SpeedUnit.KNOTS, self.ground_speed)
            self._msg_list.append(WaterSpeedAndHeading(water_speed, heading_true))

    def idle_task(self):
        '''Idle tasks
        
            - check if the GUI has received a close event
            - periodically send data to the GUI
        '''
        # tell MAVProxy to unload the module if the GUI is closed
        if self.sail_dash.close_event.wait(timeout=0.001):
            self.needs_unloading = True

        # send message list via pipe to gui at desired update rate
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to GUI
            self.sail_dash.parent_pipe_send.send(self._msg_list)

            # reset counters etc.
            self._msg_list = []
            self._last_send = time.time()

    def unload(self):
        '''Close the GUI and unload module'''

        # close the gui
        self.sail_dash.close()

    @staticmethod
    def _convert_speed_units(from_units, to_units, speed):
        '''Convert a speed to different units'''

        # use m/s as the reference
        to_speed = speed
        if from_units == 'm/s':
            pass
        elif from_units == 'kph':
            to_speed /= 3.6
        elif from_units == 'mph':
            to_speed /= 2.23694
        elif from_units == 'knots':
            to_speed /= 1.94384
        else:
            raise Exception('Invalid source speed units')

        if to_units == SpeedUnit.KPH:
            to_speed *= 3.6
        elif to_units == SpeedUnit.MPH:
            to_speed *= 2.23694
        elif to_units == SpeedUnit.KNOTS:
            to_speed *= 1.94384
        else:
            raise Exception('Invalid target speed units')

        # print('from_units: {}, to_units: {}, from_speed: {}, to_speed: {}'.format(
        #     from_units, to_units, speed, to_speed))

        return to_speed

    @staticmethod
    def _wind_direction_to_angle(wind_dir, heading):
        '''Convert a wind direction to a wind angle relative to a heading'''

        wind_angle = (wind_dir - heading) % 360
        wind_angle = wind_angle if wind_angle <= 180 else wind_angle - 360
        return wind_angle

def init(mpstate):
    ''' Initialise module'''

    return SailModule(mpstate)

