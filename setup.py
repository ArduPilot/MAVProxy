from setuptools import setup
import os, platform, sys

version = "1.8.74"

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

package_data = ['modules/mavproxy_map/data/*.jpg', 
                'modules/mavproxy_map/data/*.png',
                'modules/mavproxy_mmap/mmap_app/*',
                'modules/mavproxy_joystick/joysticks/*.yml',
                'modules/mavproxy_magical/data/*.mtl',
                'modules/mavproxy_magical/data/*.obj',
                'modules/mavproxy_fieldcheck/*.txt',
                'tools/graphs/*.xml',
                'data/shortcuts/*',
]

package_data.extend(package_files('MAVProxy/modules/mavproxy_cesium/app'))

# note that we do not include all the real dependencies here (like matplotlib etc)
# as that breaks the pip install. It seems that pip is not smart enough to
# use the system versions of these dependencies, so it tries to download and install
# large numbers of modules like numpy etc which may be already installed
requirements=['pymavlink>=2.4.14',
              'pyserial>=3.0',
              'numpy',
              'pynmeagps']

# the map3d module needs these, and we want a plain "pip install -U MAVProxy"
# to bring them in rather than leaving the module dead until someone finds the
# extra. quantized-mesh-tile requires numpy>=2, which leaves a distro opencv or
# matplotlib built against numpy 1 unimportable, so the floors are here to pull
# in builds that match. The floors are needed because pip leaves an unpinned
# requirement alone when the distro build already satisfies it.
#
# These are guarded because a requirement pip cannot satisfy fails the whole
# MAVProxy install, not just map3d. The marker names exactly the platforms vtk
# publishes wheels for: it has no sdist, so anywhere else pip has nothing to
# fall back on. matplotlib>=3.9 sets the python floor. Where the marker excludes
# it, MAVProxy still installs and map3d stays dormant, printing its own install
# message if loaded. Raise the python_version bound when vtk publishes wheels
# for a newer python.
#
# It has to be a PEP 508 marker rather than a platform.system() test like the
# ones below: MAVProxy ships a py3-none-any wheel, so anything decided here at
# build time is baked in on the build machine, while a marker is evaluated on
# the installing machine. Equality tests, not "in": the marker "in" operator is
# substring containment, so platform_machine values of "64" or "" would match.
#
# The one gap markers cannot express is the C library, so musl (Alpine) still
# matches without having vtk wheels. Use the extra there instead.
MAP3D_MARKER = ('platform_python_implementation == "CPython"'
                ' and python_version >= "3.9" and python_version < "3.15"'
                ' and (sys_platform == "linux" or sys_platform == "darwin"'
                ' or sys_platform == "win32")'
                ' and (platform_machine == "x86_64"'
                ' or platform_machine == "AMD64"'
                ' or platform_machine == "aarch64"'
                ' or platform_machine == "arm64")')
map3d_requirements = ['vtk', 'quantized-mesh-tile',
                      'opencv-python>=4.10', 'matplotlib>=3.9']
requirements.extend(['%s; %s' % (r, MAP3D_MARKER) for r in map3d_requirements])

if platform.system() == "Darwin":
    # on MacOS we can have a more complete requirements list
    requirements.extend(['billiard>=3.5.0',
                         'gnureadline',
                         'matplotlib',
                         'opencv-python',
                         'lxml',
                         'wxPython'])

if platform.system() == "Windows" and sys.version_info >= (3, 0):
    # on Windows we can have a more complete requirements list
    requirements.extend(['prompt_toolkit'])
    requirements.append('requests')
elif platform.system() == "Windows":
    requirements.extend(['pyreadline'])

setup(name='MAVProxy',
      version=version,
      zip_safe=True,
      description='MAVProxy MAVLink ground station',
      long_description='''A MAVLink protocol proxy and ground station. MAVProxy
is oriented towards command line operation, and is suitable for embedding in
small autonomous vehicles or for using on ground control stations. It also
features a number of graphical tools such as a slipmap for satellite mapping
view of the vehicles location, and status console and several useful vehicle
control modules. MAVProxy is extensible via a modules system - see the modules
subdirectory for some example modules. MAVProxy was developed by CanberraUAV
for use in the 2012 Outback Challenge, and includes a module for the
CanberraUAV search and rescue system. See
https://ardupilot.org/mavproxy/index.html for more information
on how to use MAVProxy.''',
      url='https://github.com/ArduPilot/MAVProxy',
      author='Andrew Tridgell',
      author_email='andrew@tridgell.net',
      classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering'],
      license='GPLv3',
      packages=['MAVProxy',
                'MAVProxy.tools',
                'MAVProxy.modules',
                'MAVProxy.modules.mavproxy_anufireproject',
                'MAVProxy.modules.mavproxy_fieldcheck',
                'MAVProxy.modules.mavproxy_map',
                'MAVProxy.modules.mavproxy_map3d',
                'MAVProxy.modules.mavproxy_mmap',
                'MAVProxy.modules.mavproxy_misseditor',
                'MAVProxy.modules.mavproxy_paramedit',
                'MAVProxy.modules.mavproxy_smartcamera',
                'MAVProxy.modules.mavproxy_cesium',
                'MAVProxy.modules.mavproxy_joystick',
                'MAVProxy.modules.mavproxy_magical',
                'MAVProxy.modules.mavproxy_optitrack',
                'MAVProxy.modules.mavproxy_nokov',
                'MAVProxy.modules.mavproxy_SIYI',
                'MAVProxy.modules.mavproxy_camera',
                'MAVProxy.modules.mavproxy_chat',
                'MAVProxy.modules.lib',
                'MAVProxy.modules.lib.ANUGA',
                'MAVProxy.modules.lib.MacOS',
                'MAVProxy.modules.lib.optparse_gui'],
      install_requires=requirements,
      extras_require={
        'cesium': ['tornado'],
        # map3d is in the base requirements now. The extra is kept so the
        # documented MAVProxy[map3d] still works, and asks for the packages
        # without the platform marker: someone naming it explicitly wants a
        # loud failure rather than a silently dormant module
        'map3d': map3d_requirements,
        # restserver module
        'server': ['flask'],
        'recommended': ['flask', 'PyYAML', 'lxml', 'wxpython',
                        'pymonocypher', 'openai', 'paho-mqtt',
                        'piexif', 'pynmea2', 'Pygame', 'Pillow']
      },
      scripts=['MAVProxy/mavproxy.py',
               'MAVProxy/tools/mavflightview.py',
               'MAVProxy/tools/MAVExplorer.py',
               'MAVProxy/tools/mavpicviewer/mavpicviewer.py',
               'MAVProxy/modules/mavproxy_map/mp_slipmap.py',
               'MAVProxy/modules/mavproxy_map/mp_tile.py'],
      package_data={'MAVProxy':
                    package_data}
    )
