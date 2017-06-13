# Maintain the guard for wx being able to be
# loaded on this process. This is only needed
# for Darwin, so to reduce error cases, only
# implement it on Darwin.

import sys
safe = sys.platform != 'darwin'

from distutils.version import StrictVersion
phoenix = lambda x, y: StrictVersion(wx.__version__) >= StrictVersion('4.0')