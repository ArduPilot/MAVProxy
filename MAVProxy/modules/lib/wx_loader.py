import wx_util

if not wx_util.safe:
    print 'Cannot access wx from main thread.'
    import traceback
    print traceback.print_stack()
    raise Exception('Cannot access wx from main thread')
    
import wx
