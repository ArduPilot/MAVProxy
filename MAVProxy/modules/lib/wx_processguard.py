# Explicit information for the Python loader
# that this subprocess can load wx, which is not process-safe.

from MAVProxy.modules.lib import wx_util

wx_util.safe = True
