#!/usr/bin/env python3
'''
Install the MAVProxy/MAVExplorer/MAVPicViewer .desktop launchers and icons
into the user's XDG directories, so freedesktop desktops (GNOME, KDE, etc.)
show the correct taskbar/Activities icon instead of a generic one.

This is not run automatically by "pip install", since pip cannot reliably
write to these directories (particularly when installing into a venv), so
run it manually once after installing MAVProxy:

    python3 -m MAVProxy.tools.install_desktop_icons

See MAVProxy/data/shortcuts/ReadMe.md for background, and
https://github.com/wxWidgets/wxWidgets/issues/24034 for why GTK3/GNOME
needs an installed .desktop file to show a window's taskbar icon at all.

AP_FLAKE8_CLEAN
'''
import importlib.resources
import os
import struct
import subprocess
import sys

# the size the shipped icons are, and so the hicolor theme directory they
# belong in. Icons of any other size are skipped rather than installed into
# a directory that claims the wrong size.
ICON_SIZE = 256

PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def shortcuts_dir():
    '''the packaged shortcuts directory, as an importlib Traversable'''
    return importlib.resources.files('MAVProxy').joinpath('data').joinpath('shortcuts')


def xdg_data_home():
    '''$XDG_DATA_HOME, or its default. Per the XDG basedir spec a relative
    path is invalid and is ignored'''
    path = os.environ.get('XDG_DATA_HOME', '')
    if not os.path.isabs(path):
        path = os.path.join(os.path.expanduser('~'), '.local', 'share')
    return path


def png_size(data):
    '''(width, height) of a PNG held in bytes, or None if it is not a PNG'''
    if len(data) < 24 or data[:8] != PNG_SIGNATURE or data[12:16] != b'IHDR':
        return None
    return struct.unpack('>II', data[16:24])


def write_file(dest, data, mode=None):
    with open(dest, 'wb') as f:
        f.write(data)
    if mode is not None:
        os.chmod(dest, mode)
    print("Installed %s" % dest)


def install():
    if sys.platform in ('win32', 'cygwin', 'darwin'):
        print("This installer only applies to freedesktop desktops (Linux/BSD); "
              "nothing to do on %s" % sys.platform)
        return 1

    data_home = xdg_data_home()
    applications_dir = os.path.join(data_home, 'applications')
    icon_theme_dir = os.path.join(data_home, 'icons', 'hicolor')
    icons_dir = os.path.join(icon_theme_dir, '%dx%d' % (ICON_SIZE, ICON_SIZE), 'apps')

    shortcuts = shortcuts_dir()
    entries = sorted([e for e in shortcuts.iterdir() if e.is_file()], key=lambda e: e.name)
    if not entries:
        print("No shortcut files found in the MAVProxy package - the install looks incomplete")
        return 1

    os.makedirs(applications_dir, exist_ok=True)
    os.makedirs(icons_dir, exist_ok=True)

    installed = 0
    for entry in entries:
        if entry.name.endswith('.desktop'):
            write_file(os.path.join(applications_dir, entry.name), entry.read_bytes(), 0o755)
            installed += 1
        elif entry.name.endswith('.png'):
            data = entry.read_bytes()
            size = png_size(data)
            if size != (ICON_SIZE, ICON_SIZE):
                print("Skipping %s: expected a %dx%d icon, found %s"
                      % (entry.name, ICON_SIZE, ICON_SIZE,
                         "%dx%d" % size if size else "an unreadable PNG header"))
                continue
            write_file(os.path.join(icons_dir, entry.name), data)
            installed += 1

    if installed == 0:
        print("No .desktop or icon files were installed")
        return 1

    for cmd in (['update-desktop-database', applications_dir],
                ['gtk-update-icon-cache', icon_theme_dir]):
        try:
            subprocess.run(cmd, capture_output=True, check=False)
        except FileNotFoundError:
            pass

    print("Done. You may need to log out and back in (or restart the "
          "desktop shell) for the taskbar icon to update.")
    return 0


if __name__ == '__main__':
    sys.exit(install())
