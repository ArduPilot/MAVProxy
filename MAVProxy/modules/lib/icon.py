"""
  MAVProxy icons generator
"""
import numpy as np
from io import BytesIO
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from MAVProxy.modules.lib.wx_loader import wx


class SimpleIcon():
    """
    Generate icons with Matplotlib.
    """
    def __init__(self, text=None):
        LIGHT_BLUE_BACKGROUND = "#afceff"
        VIOLET_FONT = "#4800f0"
        t1 = np.arange(0.0, 5.0, 0.1)
        t2 = np.arange(0.0, 5.0, 0.02)

        def f(t):
            return np.exp(-t) * np.cos(2*np.pi*t)

        fig = Figure(figsize=(1.28, 1.28), facecolor=LIGHT_BLUE_BACKGROUND)
        spec = GridSpec(nrows=2, ncols=1, wspace=0, hspace=0.0, height_ratios=[1, 2])
        ax = fig.add_subplot(spec[0])
        ax.text(0, 0.8, 'MAV', style='italic', fontsize=20, weight='bold', color=VIOLET_FONT)
        if text is None:
            text = "GRAPH"
        ax.text(0, 0.1, text.upper(), style='italic', fontsize=14, weight='bold', color=VIOLET_FONT)
        ax.axis('off')

        ax = fig.add_subplot(spec[1], facecolor=LIGHT_BLUE_BACKGROUND)
        if text == "GRAPH" or text == "EXPLORER":
            ax.plot(t1, f(t1), color='tab:green', marker='o')
            ax.plot(t2, f(t2), color='red')
            ax.set_xticks(t1, minor=True)
            ax.grid(True, which='both', linestyle="--")
        if text == "CONSOLE":
            ax.plot([0, 2.5], [2.5, 1.25], color='red', linewidth=4)
            ax.plot([0, 2.5], [0, 1.25], color='red', linewidth=4)
            ax.plot([3, 5], [0, 0], color='red', linewidth=4)
            ax.axis('off')
        if text == "MAP":
            path_data = [
                (mpath.Path.MOVETO, [2.5, 0]),
                (mpath.Path.LINETO, [1.25, 2.5]),
                (mpath.Path.CURVE3, [2.5, 5]),
                (mpath.Path.CURVE3, [3.75, 2.5]),
                (mpath.Path.LINETO, [3.75, 2.5]),
                (mpath.Path.CLOSEPOLY, [2.5, 0])]
            codes, verts = zip(*path_data)
            path = mpath.Path(verts, codes)

            patch = mpatches.PathPatch(path, facecolor='red', lw=2)
            ax.add_patch(patch)
            circle_neg = mpatches.Circle((2.5, 2.5), 0.5, color='white')
            ax.add_patch(circle_neg)

            ax.plot([0, 5/3.0, 10/3.0, 5], [5, 4, 5, 4], color='black', linewidth=2)
            ax.plot([0, 5/3.0, 10/3.0, 5], [-1, -2, -1, -2], color='black', linewidth=2)
            ax.plot([0, 0], [-1, 5], color='black', linewidth=2)
            ax.plot([5/3.0, 5/3.0], [-2, 4], color='black', linewidth=2)
            ax.plot([10/3.0, 10/3.0], [-1, 5], color='black', linewidth=2)
            ax.plot([5, 5], [-2, 4], color='black', linewidth=2)
            ax.set_xlim(-0.5, 5.5)
            ax.set_ylim(-2.5, 5.5)
            ax.axis('off')

        buf = BytesIO()
        fig.savefig(buf, format='png')
        buf.seek(0)
        svgimg = wx.Image(buf, wx.BITMAP_TYPE_PNG)
        self.img = svgimg

    def get_ico(self):
        """Get the ico from the image bitmap.
        need to be call after the wx.app is created."""
        return wx.Icon(wx.Bitmap(self.img))
