try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

version = "1.0.8"

setup(name='MAVProxy',
      version=version,
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
http://www.qgroundcontrol.org/mavlink/mavproxy_startpage for more information
on how to use MAVProxy.''',
      url='https://github.com/tridge/MAVProxy',
      author='Andrew Tridgell',
      author_email='andrew@tridgell.net',
      classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Topic :: Scientific/Engineering'],
      license='GPLv3',
      package_dir={'MAVProxy': '.'},
      packages=['MAVProxy',
                'MAVProxy.modules',
                'MAVProxy.modules.mavproxy_map',
                'MAVProxy.modules.lib',
                'MAVProxy.modules.lib.ANUGA',
                'MAVProxy.modules.lib.optparse_gui',
                'MAVProxy.modules.mavproxy_CUAV'],
      install_requires=['pymavlink>=1.1.2',
                        'pyserial>=2.6'],
      scripts=['mavproxy.py', 'tools/mavflightview.py'],
      package_data={'MAVProxy.modules.mavproxy_map':
                    ['data/*.jpg', 'data/*.png']}
    )
