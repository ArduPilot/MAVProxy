# This script reads the setup.py and returns the current version number
# Used as part of building the WIndows setup file (MAVProxyWinBuild.bat)
# It assumes there is a line like this:
# version = "12344"

import sys

def get_version():
    with open("../setup.py") as f:
        searchlines = f.readlines()
        for i, line in enumerate(searchlines):
            if "version = " in line:
                return line[11:len(line)-2]
    return None

version = get_version()
if version is None:
    print("Unable to find version")
    sys.exit(1)

open("run_ISCC.cmd", "w").write('ISCC.exe /dMyAppVersion=%s mavproxy.iss\r\n' % version)

# also write to version.txt
open('version.txt', 'w').write(version)
