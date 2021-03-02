from __future__ import division, print_function
import matplotlib
from matplotlib.figure import Figure

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.backends import backend_wx    # already uses wxversion.ensureMinimal('2.8')
from matplotlib.backends.backend_wx import error_msg_wx, NavigationToolbar2Wx
from matplotlib.backends.backend_wxagg import FigureManager, FigureCanvasWxAgg, \
    FigureFrameWx, draw_if_interactive, show, backend_version
import wx

class FigureFrameWxAgg(FigureFrameWx):
    def get_canvas(self, fig):
        return FigureCanvasWxAgg(self, -1, fig)

    def _get_toolbar(self, statbar):
        if matplotlib.rcParams['toolbar']=='classic':
            toolbar = NavigationToolbar2Wx(self.canvas, True)
        elif matplotlib.rcParams['toolbar']=='toolbar2':
            toolbar = NavigationToolbar2WxAgg(self.canvas)
            toolbar.set_status_bar(statbar)
        else:
            toolbar = None
        return toolbar

class NavigationToolbar2WxAgg(NavigationToolbar2Wx):
    def get_canvas(self, frame, fig):
        return FigureCanvasWxAgg(frame, -1, fig)


def new_figure_manager(num, *args, **kwargs):
    """
    Create a new figure manager instance
    """
    # in order to expose the Figure constructor to the pylab
    # interface we need to create the figure here
    backend_wx._create_wx_app()

    FigureClass = kwargs.pop('FigureClass', Figure)
    fig = FigureClass(*args, **kwargs)

    return new_figure_manager_given_figure(num, fig)

def new_figure_manager_given_figure(num, figure):
    """
    Create a new figure manager instance for the given figure.
    """
    frame = FigureFrameWxAgg(num, figure)
    figmgr = frame.get_figure_manager()
    if matplotlib.is_interactive():
        figmgr.frame.Show()
    return figmgr


#
# agg/wxPython image conversion functions (wxPython >= 2.8)
#

def _convert_agg_to_wx_image(agg, bbox):
    """
    Convert the region of the agg buffer bounded by bbox to a wx.Image.  If
    bbox is None, the entire buffer is converted.

    Note: agg must be a backend_agg.RendererAgg instance.
    """
    if bbox is None:
        # agg => rgb -> image
        image = wx.EmptyImage(int(agg.width), int(agg.height))
        image.SetData(agg.tostring_rgb())
        return image
    else:
        # agg => rgba buffer -> bitmap => clipped bitmap => image
        return wx.ImageFromBitmap(_WX28_clipped_agg_as_bitmap(agg, bbox))


def _convert_agg_to_wx_bitmap(agg, bbox):
    """
    Convert the region of the agg buffer bounded by bbox to a wx.Bitmap.  If
    bbox is None, the entire buffer is converted.

    Note: agg must be a backend_agg.RendererAgg instance.
    """
    if bbox is None:
        # agg => rgba buffer -> bitmap
        return wx.BitmapFromBufferRGBA(int(agg.width), int(agg.height),
            agg.buffer_rgba())
    else:
        # agg => rgba buffer -> bitmap => clipped bitmap
        return _WX28_clipped_agg_as_bitmap(agg, bbox)


def _WX28_clipped_agg_as_bitmap(agg, bbox):
    """
    Convert the region of a the agg buffer bounded by bbox to a wx.Bitmap.

    Note: agg must be a backend_agg.RendererAgg instance.
    """
    l, b, width, height = bbox.bounds
    r = l + width
    t = b + height

    srcBmp = wx.BitmapFromBufferRGBA(int(agg.width), int(agg.height),
        agg.buffer_rgba())
    srcDC = wx.MemoryDC()
    srcDC.SelectObject(srcBmp)

    destBmp = wx.EmptyBitmap(int(width), int(height))
    destDC = wx.MemoryDC()
    destDC.SelectObject(destBmp)

    destDC.BeginDrawing()
    x = int(l)
    y = int(int(agg.height) - t)
    destDC.Blit(0, 0, int(width), int(height), srcDC, x, y)
    destDC.EndDrawing()

    srcDC.SelectObject(wx.NullBitmap)
    destDC.SelectObject(wx.NullBitmap)

    return destBmp