#!/usr/bin/env python3
'''download parameter XML files for windows build'''

import os
import sys

parms = {
    'http://autotest.ardupilot.org/Parameters/APMrover2/apm.pdef.xml' : 'APMrover2.xml',
    'http://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml' : 'ArduCopter.xml',
    'http://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml' : 'ArduPlane.xml',
    'http://autotest.ardupilot.org/Parameters/ArduSub/apm.pdef.xml' : 'ArduSub.xml',
    'http://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.xml' : 'AntennaTracker.xml',
    'http://autotest.ardupilot.org/APMrover2-defaults.parm' : 'APMrover2-defaults.parm',
    'http://autotest.ardupilot.org/ArduCopter-defaults.parm' : 'ArduCopter-defaults.parm',
    'http://autotest.ardupilot.org/ArduPlane-defaults.parm' : 'ArduPlane-defaults.parm',
    'http://autotest.ardupilot.org/ArduSub-defaults.parm' : 'ArduSub-defaults.parm',
}


try:
    os.mkdir('Parameters')
except Exception:
    pass

for url in parms.keys():
    try:
        fname = os.path.join('Parameters', parms[url])
        print("Downloading %s as %s" % (url, fname))
        if sys.version_info[0] >= 3:
            import urllib.request
            urllib.request.urlretrieve(url, fname)
        else:
            import urllib
            urllib.urlretrieve(url, fname)
    except Exception as ex:
        print(ex)
        pass
