# Explicit information for the Python loader
# that this subprocess can load wx, which is not process-safe.

import wx_util

wx_util.safe = True
