from setuptools import setup
import os, platform, sys

version = "1.8.23"

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
]

package_data.extend(package_files('MAVProxy/modules/mavproxy_cesium/app'))

# note that we do not include all the real dependencies here (like matplotlib etc)
# as that breaks the pip install. It seems that pip is not smart enough to
# use the system versions of these dependencies, so it tries to download and install
# large numbers of modules like numpy etc which may be already installed
requirements=['pymavlink>=2.3.3',
              'pyserial>=3.0']

if platform.system() == "Darwin":
    # on MacOS we can have a more complete requirements list
    requirements.extend(['billiard>=3.5.0',
                         'gnureadline',
                         'matplotlib',
                         'numpy',
                         'opencv-python',
                         'lxml',
                         'future',
                         'wxPython'])

if platform.system() == "Windows" and sys.version_info >= (3, 0):
    # on MacOS we can have a more complete requirements list
    requirements.extend(['prompt_toolkit'])
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
                'MAVProxy.modules',
                'MAVProxy.modules.mavproxy_fieldcheck',
                'MAVProxy.modules.mavproxy_map',
                'MAVProxy.modules.mavproxy_mmap',
                'MAVProxy.modules.mavproxy_misseditor',
                'MAVProxy.modules.mavproxy_paramedit',
                'MAVProxy.modules.mavproxy_smartcamera',
                'MAVProxy.modules.mavproxy_cesium',
                'MAVProxy.modules.mavproxy_joystick',
                'MAVProxy.modules.mavproxy_magical',
                'MAVProxy.modules.lib',
                'MAVProxy.modules.lib.ANUGA',
                'MAVProxy.modules.lib.MacOS',
                'MAVProxy.modules.lib.optparse_gui'],
      install_requires=requirements,
      extras_require={
        # restserver module
        'server': ['flask'],
      },
      scripts=['MAVProxy/mavproxy.py',
               'MAVProxy/tools/mavflightview.py',
               'MAVProxy/tools/MAVExplorer.py',
               'MAVProxy/modules/mavproxy_map/mp_slipmap.py',
               'MAVProxy/modules/mavproxy_map/mp_tile.py'],
      package_data={'MAVProxy':
                    package_data}
    )
