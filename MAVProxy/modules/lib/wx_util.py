# Maintain the guard for wx being able to be
# loaded on this process. This is only needed
# for Darwin, so to reduce error cases, only
# implement it on Darwin.

import sys
safe = sys.platform != 'darwin'

from distutils.version import LooseVersion
phoenix = lambda x, y: LooseVersion(wx.__version__) >= LooseVersion('4.0')