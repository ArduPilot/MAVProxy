"""
    MAVProxy help/versioning module

AP_FLAKE8_CLEAN
"""
import platform
import re
import requests

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
if mp_util.has_wxpython:
    import wx
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu
    from MAVProxy.modules.lib.mp_menu import MPMenuOpenWeblink
    from MAVProxy.modules.lib.mp_menu import MPMenuChildMessageDialog


class HelpModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(HelpModule, self).__init__(mpstate, "mavhelp", "Help and version information", public = True)  # noqa
        self.enabled = False
        self.add_command('mavhelp', self.cmd_help, "help and version information", "<about|site>")
        self.have_list = False

        #  versioning info
        #  pkg_resources doesn't work in the windows exe build, so read the version file
        try:
            import pkg_resources
            self.version = pkg_resources.Environment()["mavproxy"][0].version
        except Exception:
            start_script = mp_util.dot_mavproxy("version.txt")
            f = open(start_script, 'r')
            self.version = f.readline()
        self.host = platform.system() + platform.release()
        self.pythonversion = str(platform.python_version())
        if mp_util.has_wxpython:
            self.wxVersion = str(wx.__version__)
        else:
            self.wxVersion = ''

        #  check for updates, if able
        self.check_for_updates()

        if mp_util.has_wxpython:
            self.menu_added_console = False
            self.menu = MPMenuSubMenu(
                'Help',
                items=[
                    MPMenuItem(
                        'MAVProxy website',
                        'MAVProxy website',
                        '',
                        handler=MPMenuOpenWeblink('https://ardupilot.org/mavproxy/index.html'),
                    ),
                    MPMenuItem(
                        'Check for Updates',
                        'Check for Updates',
                        '',
                        handler=MPMenuChildMessageDialog(
                            title="Updates",
                            message=self.newversion,
                        )
                    ),
                    MPMenuItem(
                        'About',
                        'About',
                        '',
                        handler=MPMenuChildMessageDialog(
                            title="About MAVProxy",
                            message=self.about_string(),
                        ))
                ])

    def check_for_updates(self):
        url = "https://pypi.org/pypi/MAVProxy/json"
        try:
            response = requests.get(url)
        except requests.exceptions.RequestException as e:
            self.newversion = f"Error finding update: {e}"
            return

        if response.status_code != 200:
            self.newversion = f"HTTP error getting update ({response.status_code})"
            return

        self.newversion = response.json()['info']['version']

        #  and format the update string
        if not isinstance(self.newversion, str):
            self.newversion = "Error finding update"
        elif re.search('[a-zA-Z]', self.newversion):
            self.newversion = "Error finding update: " + self.newversion
        elif self.newversion.strip() == self.version.strip():
            self.newversion = "Running latest version"
        else:
            self.newversion = "New version " + self.newversion + " available (currently running " + self.version + ")"

    def about_string(self):
        bits = {
            "MAVProxy Version": self.version,
            "OS": self.host,
            "Python": self.pythonversion,
            "WXPython": self.wxVersion,
            "LatestVersion": self.newversion,
        }
        return "".join([f"{x[0]}: {x[1]}\n" for x in bits.items()])

    def idle_task(self):
        '''called on idle'''
        if self.module('console') is not None:
            if not self.menu_added_console:
                self.menu_added_console = True
                self.module('console').add_menu(self.menu)
        else:
            self.menu_added_console = False

    def cmd_help(self, args):
        '''help commands'''
        if len(args) < 1:
            self.print_usage()
            return

        if args[0] == "about":
            print(self.about_string())
        elif args[0] == "site":
            print("See https://ardupilot.org/mavproxy/index.html for documentation")
        else:
            self.print_usage()

    def mavlink_packet(self, m):
        '''handle and incoming mavlink packets'''

    def print_usage(self):
        print("usage: mavhelp <about|site>")


def init(mpstate):
    '''initialise module'''
    return HelpModule(mpstate)
