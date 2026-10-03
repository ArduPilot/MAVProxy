Installing Linux shortcuts for MAVProxy, MAVExplorer and MAVPicViewer with icons

The easiest way to install these is to run the helper script from a terminal:

```
python3 -m MAVProxy.tools.install_desktop_icons
```

This copies the .desktop files to `$XDG_DATA_HOME/applications` and the .png
icons to `$XDG_DATA_HOME/icons/hicolor/256x256/apps` (`$XDG_DATA_HOME`
defaults to `~/.local/share`), then best-effort refreshes the desktop and
icon caches. The hicolor path is the standard freedesktop icon theme
location and is required for GNOME/GTK to find the icon - a non-standard
location such as `~/.icon` is never searched.

The script is not run automatically by `pip install`, since pip cannot
reliably write to these XDG directories (especially when installing into a
venv), so run it yourself once after installing MAVProxy.

To install manually instead:

1. Copy the .desktop files to the .local/share/applications directory in your home directory. You may need to use CRTL-H to see the .local directory in your file manager.
2. Copy the .png icon files to the .local/share/icons/hicolor/256x256/apps directory in your home directory (create it if it doesn't exist).
3. Open the Software search page (Show Applications button on launcher bar)
4. Find either MAVProxy or MAVExplorer and add to favorites to have them in the launch bar
5. If you rather have a desktop shortcut, copy the appropriate desktop file to the Desktop. You may be asked to verify that this is a trusted application, if so, do so.

Why an installed .desktop file is needed
----------------------------------------

On GNOME/GTK3 the taskbar/Activities icon for a running window does not come
from the icon the application sets at runtime. GNOME Shell matches the
window's `WM_CLASS` against the `StartupWMClass` of an installed .desktop
file and paints that file's `Icon=` - see
https://github.com/wxWidgets/wxWidgets/issues/24034. Without an installed,
matching .desktop file there is nothing to match and the window falls back to
a generic icon.

The `StartupWMClass` values shipped here are the `WM_CLASS` instance names
wxWidgets derives from each script's name:

| .desktop | WM_CLASS | StartupWMClass |
| --- | --- | --- |
| mavproxy.desktop | `"mavproxy.py", "Mavproxy"` | `mavproxy.py` |
| mavexplorer.desktop | `"MAVExplorer.py", "Mavexplorer"` | `MAVExplorer.py` |
| mavpicviewer.desktop | `"mavpicviewer.py", "Mavpicviewer"` | `mavpicviewer.py` |

Note that MAVProxy's console, map and mission editor windows all run in
`multiprocessing.Process` children forked from `mavproxy.py`, so they inherit
its `WM_CLASS`. GNOME therefore groups them all under `mavproxy.desktop` and
paints `Icon=mavproxy` for every one of them - the per-window icons the code
sets with `SetIcon()` still apply wherever `_NET_WM_ICON` is used instead
(window lists, and other desktops such as KDE).

GNOME Shell typically only picks up a newly-installed .desktop file for
windows that already existed once its own caches are refreshed - in testing,
the icon did not update immediately after running the installer while
MAVProxy was already running. If it doesn't appear right away, restart GNOME
Shell (on X11: press Alt+F2, type `r`, press Enter; on Wayland: log out and
back in) and relaunch MAVProxy.

Notes: the MAVProxy shortcut will store its data files (mav.param, mav.tlog, etc.) in a directory called MAVProxy_data n the home directory, if it exists, or the home directory if it does not...you can edit the mavproxy.desktop to change the directory if you wish.

If you place a copy of the mavexplorer.desktop file on the desktop, you can drag and drop a log file onto it and it will open using it.
