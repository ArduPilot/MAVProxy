#!/usr/bin/env python3
'''
Install the MAVProxy/MAVExplorer/MAVPicViewer .desktop launchers and icons
into the user's XDG directories, so Linux desktops (GNOME, KDE, etc.) show
the correct taskbar/Activities icon instead of a generic one.

This is not run automatically by "pip install", since pip cannot reliably
write to these directories (particularly when installing into a venv), so
run it manually once after installing MAVProxy:

    python3 -m MAVProxy.tools.install_desktop_icons

See shortcuts/ReadMe.md for background, and
https://github.com/wxWidgets/wxWidgets/issues/24034 for why GTK3/GNOME
needs an installed .desktop file to show a window's taskbar icon at all.
'''
import os
import shutil
import subprocess

SHORTCUTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'shortcuts')

APPLICATIONS_DIR = os.path.expanduser('~/.local/share/applications')
ICONS_DIR = os.path.expanduser('~/.local/share/icons/hicolor/256x256/apps')


def install():
    if not os.path.isdir(SHORTCUTS_DIR):
        print("shortcuts directory not found at %s" % SHORTCUTS_DIR)
        return

    os.makedirs(APPLICATIONS_DIR, exist_ok=True)
    os.makedirs(ICONS_DIR, exist_ok=True)

    for filename in sorted(os.listdir(SHORTCUTS_DIR)):
        src = os.path.join(SHORTCUTS_DIR, filename)
        if filename.endswith('.desktop'):
            dest = os.path.join(APPLICATIONS_DIR, filename)
            shutil.copy(src, dest)
            os.chmod(dest, 0o755)
            print("Installed %s" % dest)
        elif filename.endswith('.png'):
            dest = os.path.join(ICONS_DIR, filename)
            shutil.copy(src, dest)
            print("Installed %s" % dest)

    for cmd in (['update-desktop-database', APPLICATIONS_DIR],
                ['gtk-update-icon-cache', os.path.expanduser('~/.local/share/icons/hicolor')]):
        try:
            subprocess.run(cmd, capture_output=True, check=False)
        except FileNotFoundError:
            pass

    print("Done. You may need to log out and back in (or restart the "
          "desktop shell) for the taskbar icon to update.")


if __name__ == '__main__':
    install()
