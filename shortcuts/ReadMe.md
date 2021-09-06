Installing Linux shortcuts for MAVPRoxy and MAVExplorer with icons

1. Copy the .desktop files to the .local/share/applications directory in your home directory. You may need to use CRTL-H to see the .local directory in your file manager.
2. Copy the .png icon files to the .icon directory in your home directory.
3. Open the Software search page (Show Applications button on launcher bar)
4. Find either MAVProxy or MAVExplorer and add to favorites to have them in the launch bar
5. If you rather have a desktop shortcut, copy the appropriate desktop file to the Desktop. You may be asked to verify that this is a trusted application, if so, do so.

Notes: the MAVProxy shortcut will store its data files (mav.param, mav.tlog, etc.) in a directory called MAVProxy_data n the home directory, if it exists, or the home directory if it does not...you can edit the mavproxy.desktop to change the directory if you wish.

If you place a copy of the mavexplorer.desktop file on the desktop, you can drag and drop a log file onto it and it will open using it.
