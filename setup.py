from distutils.core import setup, Extension

setup (name = 'MAVProxy',
       version = '1.0.4',
       description = 'MAVProxy MAVLink ground station',
       url = 'https://github.com/tridge/MAVProxy',
       author = 'Andrew Tridgell',
       author_email = 'andrew@tridgell.net',
       package_dir = { 'MAVProxy' : '.' },
       packages = ['MAVProxy',
                   'MAVProxy.modules',
                   'MAVProxy.modules.mavproxy_map',
                   'MAVProxy.modules.lib',
                   'MAVProxy.modules.lib.ANUGA',
                   'MAVProxy.modules.mavproxy_CUAV' ],
       requires = [ 'pymavlink (>=1.1.2)' ],
       scripts = [ 'mavproxy.py', 'tools/mavflightview.py' ],
       package_data = { 'MAVProxy.modules.mavproxy_map' :
                        [ 'data/*.jpg', 'data/*.png' ]}
       )
