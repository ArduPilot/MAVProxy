Installing Linux shortcuts for MAVPRoxy and MAVExplorer with icons

The easiest way to install these is to run the helper script from a terminal:

```
python3 -m MAVProxy.tools.install_desktop_icons
```

This copies the .desktop files to `~/.local/share/applications` and the
.png icons to `~/.local/share/icons/hicolor/256x256/apps` (the standard
freedesktop icon theme location, required for GNOME/GTK to pick up the
icon for the taskbar/Activities view - a non-standard location such as
`~/.icon` will not be found), then best-effort refreshes the desktop and
icon caches. It is not run automatically by `pip install`, since pip
cannot reliably write to these XDG directories (especially when installed
into a venv), so run it yourself once after installing MAVProxy.

To install manually instead:

1. Copy the .desktop files to the .local/share/applications directory in your home directory. You may need to use CRTL-H to see the .local directory in your file manager.
2. Copy the .png icon files to the .local/share/icons/hicolor/256x256/apps directory in your home directory (create it if it doesn't exist).
3. Open the Software search page (Show Applications button on launcher bar)
4. Find either MAVProxy or MAVExplorer and add to favorites to have them in the launch bar
5. If you rather have a desktop shortcut, copy the appropriate desktop file to the Desktop. You may be asked to verify that this is a trusted application, if so, do so.

Note: on some GNOME/GTK3 desktops, the taskbar/Activities icon for a
running window is looked up from the installed .desktop file's WM_CLASS
match rather than the icon the application sets at runtime - see
https://github.com/wxWidgets/wxWidgets/issues/24034. GNOME Shell typically
only picks up a newly-installed .desktop file for windows that already
existed, or after its own caches are refreshed - in testing, the icon did
not update immediately after running the installer while MAVProxy was
already running. If it doesn't appear right away, restart GNOME Shell
(on X11: press Alt+F2, type `r`, press Enter; on Wayland: log out and back
in) and relaunch MAVProxy.

Notes: the MAVProxy shortcut will store its data files (mav.param, mav.tlog, etc.) in a directory called MAVProxy_data n the home directory, if it exists, or the home directory if it does not...you can edit the mavproxy.desktop to change the directory if you wish.

If you place a copy of the mavexplorer.desktop file on the desktop, you can drag and drop a log file onto it and it will open using it.
