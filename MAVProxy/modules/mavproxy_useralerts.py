'''Advise user if there are applicable user alerts
This is a reference implementation based on the documentation at
https://github.com/ArduPilot/useralerts and
https://ardupilot.org/dev/docs/user-alerts-developer.html '''


import sys
import json
import string
from packaging import version
from pymavlink import mavutil

if sys.version_info.major < 3:
    from urllib2 import Request
    from urllib2 import urlopen
    from urllib2 import URLError
else:
    from urllib.request import Request
    from urllib.request import urlopen
    from urllib.error import URLError

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_util import decode_flight_sw_version

class UserAlertsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(UserAlertsModule, self).__init__(mpstate, "useralerts", public=True)

        self.url = "https://firmware.ardupilot.org/useralerts/manifest.json"
        self.testurl = "https://firmware.ardupilot.org/useralerts/examplemanifest.json"

        # The useTest setting allows the user to query the example database
        # instead of the real database
        # The filterBoard setting determines whether User Alerts are filtered
        # by the hardwareLimited field.
        self.alerts_settings = mp_settings.MPSettings(
            [('useTest', bool, False),
             ('filterBoard', bool, True)])
        self.add_completion_function('(ALERTSETTING)',
                                     self.alerts_settings.completion)

        self.add_command('useralerts', self.cmd_check, "Check User Alerts",
                         ["check",
                          "set (ALERTSETTING)"])

        self.board = None
        self.version = None

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype == "STATUSTEXT":
            #Looking for a response to MAV_CMD_DO_SEND_BANNER:
            #APM: KakuteF4 00390032 4B4B500F 20313548
            #no nice way to parse this. So assuming 4 words, the last 3 are 8 character hex values
            splitText = m.text.split(" ")
            if (len(splitText) == 4 and len(splitText[1]) == 8 and len(splitText[2]) == 8 and len(splitText[3]) == 8 and
                    all(c in string.hexdigits for c in splitText[1]) and all(c in string.hexdigits for c in splitText[2]) and
                    all(c in string.hexdigits for c in splitText[3])):
                self.board = splitText[0]
            if not self.alerts_settings.filterBoard:
                self.board = "N/A"

        elif mtype == "AUTOPILOT_VERSION":
            #Example output:
            #AUTOPILOT_VERSION {capabilities : 61935, flight_sw_version : 67174400, middleware_sw_version : 0, os_sw_version : 0,
            # board_version : 0, flight_custom_version : [54, 49, 51, 100, 50, 99, 50, 101],
            # middleware_custom_version : [0, 0, 0, 0, 0, 0, 0, 0], os_custom_version : [51, 51, 49, 102, 101, 55, 53, 100],
            # vendor_id : 0, product_id : 0, uid : 0, uid2 : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
            vMajor,vMinor,vPatch,vFwType = decode_flight_sw_version(m.flight_sw_version)
            self.version = "{0}.{1}.{2}".format(vMajor, vMinor, vPatch)

        if self.board and self.version:
            print("Checking Alerts for {1} version {0}, board {2}".format(self.version, self.mpstate.vehicle_type, self.board))

            #download user alerts
            if self.alerts_settings.useTest:
                req = Request(self.testurl)
            else:
                req = Request(self.url)

            try:
                r = urlopen(req).read()
            except URLError:
                print("Error: unable to download User Alerts")
                # reset for next time "check" is called
                self.version = None
                self.board = None
                return

            allAlerts = json.loads(r.decode('utf-8'))
            print("Downloaded {0} alerts".format(len(allAlerts)))

            # Check and print out applicable alerts
            for (key, alert) in allAlerts.items():
                #print("Checking: {0}".format(key))
                (valid, mitigation) = self.alert_is_applicable(alert, self.version, str(self.mpstate.vehicle_type), self.board)
                if valid:
                    print("******************")
                    print("User alert {0} is valid for this vehicle:".format(key))
                    print("Issue: {0}\nMitigation: {1}".format(alert['description'], mitigation))

            # reset for next time "check" is called
            self.version = None
            self.board = None

    def alert_is_applicable(self, alert, APversion, APfirmware, boardName):
        '''checks for alert validity given ArduPilot version, board and type
        returns (true, mitigationString) if alert is applicable, (false, None) otherwise'''

        # if affectedFirmware is not all AND does not contain the APFirmware, alert is not applicable
        if ('all' not in alert['affectedFirmware']) and (APfirmware not in alert['affectedFirmware']):
            #print("False on affectedFirmware: " + str(alert['affectedFirmware']))
            return (False, None)

        # Check hardwareLimited. If there is a list of boards, check if on list
        if len(alert['hardwareLimited']) > 0 and (boardName not in alert['hardwareLimited']) and boardName != "N/A":
            #print("False on hardwareLimited: " + str(alert['hardwareLimited']))
            return (False, None)

        #check that versionFrom is either blank or less-than-or-equal-to the reported firmware
        if len(alert['versionFrom']) == 0 or version.parse(alert['versionFrom'][APfirmware]) <= version.parse(APversion):
            # Check if there is a fixed version out
            if APfirmware in alert['versionFixed'] and version.parse(alert['versionFixed'][APfirmware]) <= version.parse(APversion):
                # User is already running fixed version (or higher)
                #print("False on versionFixed: " + str(alert['versionFixed'][APfirmware]) + ", " + APversion)
                return (False, None)
            elif APfirmware in alert['versionFixed']:
                # There is a fixed version and the user is not running it
                return (True, "Upgrade to V{0} (or later) of {1}".format(alert['versionFixed'][APfirmware], APfirmware))
            else:
                # There is not a fixed version out yet for this firmware
                return (True, alert['mitigation'])
        else:
            #print("False on versionFrom: " + str(alert['versionFrom'][APfirmware]) + ", " + APversion)
            return (False, None)

    def cmd_doCheck(self):
        '''Query ArduPilot for version and board information
        When the messages are returned via mavlink_packet(), perform
        the checks for applicable User Alerts'''
        self.board = None
        self.version = None
        for messageID in [mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, mavutil.mavlink.MAV_CMD_DO_SEND_BANNER]:
            self.master.mav.command_long_send(
                self.settings.target_system,  # target_system
                self.settings.target_component, # target_component
                messageID, # command
                1, # confirmation
                1, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        #self.master.mav.command_long_send(
        #    self.settings.target_system,  # target_system
        #    self.settings.target_component, # target_component
        #    mavutil.mavlink.MAV_CMD_DO_SEND_BANNER, # command
        #    1, # confirmation
        #    0, # param1
        #    0, # param2
        #    0, # param3
        #    0, # param4
        #    0, # param5
        #    0, # param6
        #    0) # param7

    def cmd_check(self, args):
        '''Useralert operations'''
        usage = "Usage: useralerts <check|set>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == 'check':
            self.cmd_doCheck()
        elif args[0] == "set":
            self.alerts_settings.command(args[1:])

def init(mpstate):
    '''initialise module'''
    return UserAlertsModule(mpstate)
