"""
    MAVProxy help/versioning module
"""
import os, time, platform, re
from urllib2 import Request, urlopen, URLError, HTTPError
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *
    import wxversion
        
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
            self.version = pkg_resources.require("mavproxy")[0].version
        except:
            start_script = os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy", "version.txt")
            f = open(start_script, 'r')
            self.version = f.readline()
        self.host = platform.system() + platform.release()
        self.pythonversion = str(platform.python_version())
        if mp_util.has_wxpython:
            self.wxVersion = str(wxversion.getInstalled())
        else:
            self.wxVersion = ''
            
        #check for updates, if able
        if platform.system() == 'Windows':
            req = Request('http://firmware.ardupilot.org/Tools/MAVProxy/')
            html = ''
            self.newversion = '1.0'
            try:
                filehandle = urlopen(req)
                html = filehandle.read()
            except HTTPError as e:
                self.newversion = 'Error: ', e.code
            except URLError as e:
                self.newversion =  'Error: ', e.reason
            else:
                #parse the webpage for the latest version
                begtags = [m.start() for m in re.finditer('>MAVProxySetup-', html)]
                for begtag in begtags:
                    endtag = html.find('.exe', begtag)
                    versiontag = html[begtag+15:endtag]
                    if not re.search('[a-zA-Z]', versiontag):
                        if self.mycmp(self.newversion, versiontag) < 0:
                            self.newversion = versiontag
        elif platform.system() == 'Linux':
            import xmlrpclib, pip
            pypi = xmlrpclib.ServerProxy('https://pypi.python.org/pypi')
            available = pypi.package_releases('MAVProxy')
            if not available:
                self.newversion = 'Error finding update'
            else:        
                self.newversion = available[0]
                
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
                                  items=[MPMenuItem('MAVProxy website', 'MAVProxy website', '', handler=MPMenuOpenWeblink('http://dronecode.github.io/MAVProxy/')),
                                         MPMenuItem('Check for Updates', 'Check for Updates', '', handler=MPMenuChildMessageDialog(title="Updates", message=self.newversion)), 
                                         MPMenuItem('About', 'About', '', handler=MPMenuChildMessageDialog(title="About MAVProxy", message="MAVProxy Version " + self.version + "\nOS: " + self.host + "\nPython " +  self.pythonversion + "\nWXPython " + self.wxVersion))])
    
    
    #version number comparison for update checking
    def mycmp(self, version1, version2):
        def normalize(v):
            return [int(x) for x in re.sub(r'(\.0+)*$','', v).split(".")]
        return cmp(normalize(version1), normalize(version2))
        
    def idle_task(self):
        '''called on idle'''
        if self.module('console') is not None and not self.menu_added_console:
            self.menu_added_console = True
            self.module('console').add_menu(self.menu)

    def cmd_help(self, args):
        '''help commands'''
        if len(args) < 1:
            self.print_usage()
            return

        if args[0] == "about":
            print("MAVProxy Version " + self.version + "\nOS: " + self.host + "\nPython " +  self.pythonversion)
        elif args[0] == "site":
            print("See http://dronecode.github.io/MAVProxy/ for documentation")
        else:
            self.print_usage()
            
    def mavlink_packet(self, m):
        '''handle and incoming mavlink packets'''

    def print_usage(self):
        print("usage: mavhelp <about|site>")

def init(mpstate):
    '''initialise module'''
    return HelpModule(mpstate)
