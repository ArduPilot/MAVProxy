# Maintain the guard for wx being able to be
# loaded on this process. This is only needed
# for Darwin, so to reduce error cases, only
# implement it on Darwin.

import sys
safe = sys.platform != 'darwin'
