"""
    MAVProxy help/versioning module
"""
import os, time, platform, re, sys, socket
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
if mp_util.has_wxpython:
    import wx
    from MAVProxy.modules.lib.mp_menu import *

if sys.version_info.major < 3:
    from urllib2 import Request
    from urllib2 import urlopen
    from urllib2 import URLError, HTTPError
    import xmlrpclib
else:
    from urllib.request import Request
    from urllib.request import urlopen
    from urllib.error import URLError, HTTPError
    basestring = str
    import xmlrpc.client as xmlrpclib
 
class HelpModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(HelpModule, self).__init__(mpstate, "mavhelp", "Help and version information", public = True)
        self.enabled = False
        self.add_command('mavhelp', self.cmd_help, "help and version information", "<about|site>")
        self.have_list = False

        #versioning info
        #pkg_resources doesn't work in the windows exe build, so read the version file
        try:
            import pkg_resources
            self.version = pkg_resources.Environment()["mavproxy"][0].version
        except:
            start_script = mp_util.dot_mavproxy("version.txt")
            f = open(start_script, 'r')
            self.version = f.readline()
        self.host = platform.system() + platform.release()
        self.pythonversion = str(platform.python_version())
        if mp_util.has_wxpython:
            self.wxVersion = str(wx.__version__)
        else:
            self.wxVersion = ''

        #check for updates, if able
        pypi = xmlrpclib.ServerProxy('https://pypi.python.org/pypi')
        available = None
        try:
            available = pypi.package_releases('MAVProxy')
        except socket.gaierror:
            pass
            
        if available:
            self.newversion = available[0]
        else:
            self.newversion = 'Error finding update'
            
        #and format the update string
        if not isinstance(self.newversion, basestring):
            self.newversion = "Error finding update"
        elif re.search('[a-zA-Z]', self.newversion):
            self.newversion = "Error finding update: " + self.newversion
        elif self.newversion.strip() == self.version.strip():
            self.newversion = "Running latest version"
        else:
            self.newversion = "New version " + self.newversion + " available (currently running " + self.version + ")"

        if mp_util.has_wxpython:
            self.menu_added_console = False
            self.menu = MPMenuSubMenu('Help',
                                  items=[MPMenuItem('MAVProxy website', 'MAVProxy website', '', handler=MPMenuOpenWeblink('https://ardupilot.org/mavproxy/index.html')),
                                         MPMenuItem('Check for Updates', 'Check for Updates', '', handler=MPMenuChildMessageDialog(title="Updates", message=self.newversion)),
                                         MPMenuItem('About', 'About', '', handler=MPMenuChildMessageDialog(title="About MAVProxy", message=self.about_string()))])

    def about_string(self):
        return "MAVProxy Version " + self.version + "\nOS: " + self.host + "\nPython " +  self.pythonversion + "\nWXPython " + self.wxVersion

    #version number comparison for update checking
    def mycmp(self, version1, version2):
        def normalize(v):
            return [int(x) for x in re.sub(r'(\.0+)*$','', v).split(".")]
        return cmp(normalize(version1), normalize(version2))

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
