"""
MAVProxy KML Generator Module
Generates mission KML and a wrapper KML with refresh capability.
Starts a local HTTP server to serve the files for live viewing in Google Earth.

To use, load the kmlgen module, then run "kmlgen start" after setting parameters

As soon as mission waypoints are updated a mission.kml and networklink.kml file
is generated in the log directory (eg. in test/logs/2025-07-10/flight3/ )

Open google earth (desktop version only) and open the networklink.kml file. It
will display the wayponts, and will auto-update as you edit the mission
"""

import os
import time
import threading
import hashlib
import http.server
import socketserver
import functools

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink.dialects.v20 import common as mavlink2


class KMLGenModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(KMLGenModule, self).__init__(mpstate, "kmlgen", "KML mission viewer")

        self.kml_settings = mp_settings.MPSettings([
            ('port', int, 8007),
            ('refresh_interval', int, 2),
            ('mission_filename', str, 'mission.kml'),
            ('networklink_filename', str, 'networklink.kml')
        ])

        self.add_command('kmlgen', self.cmd_kmlgen, 'KML mission output',
                         ["<start>", "<set (KMLGENSETTING)>", "<status>"])
        self.add_completion_function('(KMLGENSETTING)', self.kml_settings.completion)

        self.server_thread = None
        self.running = False
        self.last_hash = None
        self.last_update = 0
        self.update_interval = 2  # seconds
        self.start_pending = False

        self.kml_path = None
        self.wrapper_path = None

    def cmd_kmlgen(self, args):
        if len(args) == 0:
            print("Usage: kmlgen <start|set|status>")
            return
        if args[0] == 'start':
            self.start_kmlgen()
        elif args[0] == 'set':
            self.kml_settings.command(args[1:])
        elif args[0] == 'status':
            self.kml_status()

    def start_kmlgen(self):
        if self.running:
            print("KMLGen already running")
            return

        if self.logdir is None:
            print("KMLGen needs logdir set, start with --aircraft option")
            return

        self.running = True
        self.update_interval = self.kml_settings.refresh_interval
        self.kml_path = os.path.join(self.logdir, self.kml_settings.mission_filename)
        self.wrapper_path = os.path.join(self.logdir, self.kml_settings.networklink_filename)

        self.write_wrapper_kml()
        self.start_pending = True
        print(f"KMLGen started: open {self.wrapper_path} in Google Earth")

    def kml_status(self):
        if not self.running:
            print("KMLGen not started")
        else:
            print(f"KMLGen running at port {self.kml_settings.port}, refresh {self.kml_settings.refresh_interval}s")
            print(f"KML output: {self.kml_path}")

    def idle_task(self):
        if not self.running:
            return

        now = time.time()
        if now - self.last_update < self.update_interval:
            return

        wp_module = self.mpstate.module("wp")
        if wp_module is None:
            return

        loader = wp_module.wploader
        if loader.count() == 0:
            return

        if self.start_pending:
            self.start_pending = False
            self.start_http_server()

        kml = self.generate_mission_kml(loader)
        current_hash = hashlib.md5(kml.encode("utf-8")).hexdigest()
        if current_hash != self.last_hash:
            self.write_file(self.kml_path, kml)
            self.last_hash = current_hash
            self.say(f"Updated {self.kml_settings.mission_filename} with {loader.count()} waypoints")

        self.last_update = now

    def generate_mission_kml(self, loader):
        home_alt = self.get_home_alt()
        coords = []
        placemarks = []
        indexes = loader.view_indexes()

        for idx in indexes:
            wp = loader.wp(idx)
            alt = self.resolve_altitude(wp, home_alt)
            coords.append(f"{wp.y},{wp.x},{alt}")
            placemarks.append(f'''
            <Placemark>
              <name>WP{idx}</name>
              <Point>
               <altitudeMode>absolute</altitudeMode>
               <coordinates>{wp.y},{wp.x},{alt}</coordinates>
              </Point>
            </Placemark>
            ''')

        path = '\n'.join(coords)
        placemark_str = '\n'.join(placemarks)

        return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <altitudeMode>absolute</altitudeMode>
    {placemark_str}
    <Placemark>
      <name>Mission Path</name>
      <Style><LineStyle><color>ff0000ff</color><width>3</width></LineStyle></Style>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
          {path}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
'''

    def resolve_altitude(self, wp, home_alt):
        if wp.frame == mavlink2.MAV_FRAME_GLOBAL:
            return wp.z
        elif wp.frame == mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT:
            return home_alt + wp.z
        elif wp.frame == mavlink2.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            terrain_alt = self.module('terrain').ElevationModel.GetElevation(wp.x, wp.y)
            return terrain_alt + wp.z
        else:
            return wp.z

    def get_home_alt(self):
        if 'HOME_POSITION' not in self.master.messages:
            return 0

        home_position = self.master.messages['HOME_POSITION']
        return home_position.altitude*0.001

    def write_file(self, path, content):
        try:
            with open(path, "w") as f:
                f.write(content)
        except Exception as e:
            self.say(f"Failed to write {path}: {e}")

    def write_wrapper_kml(self):
        url = f"http://localhost:{self.kml_settings.port}/{self.kml_settings.mission_filename}"
        content = f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <NetworkLink>
    <name>Live Mission</name>
    <Link>
      <href>{url}</href>
      <refreshMode>onInterval</refreshMode>
      <refreshInterval>{self.kml_settings.refresh_interval}</refreshInterval>
    </Link>
  </NetworkLink>
</kml>
'''
        self.write_file(self.wrapper_path, content)

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # suppress all logging

    class ReusableTCPServer(socketserver.TCPServer):
        allow_reuse_address = True

    def start_http_server(self):
        def run_server():
            try:
                handler = functools.partial(self.QuietHandler, directory=self.logdir)
                with self.ReusableTCPServer(("", self.kml_settings.port), handler) as httpd:
                    httpd.serve_forever()
            except Exception as e:
                self.say(f"HTTP server failed: {e}")

        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()
        self.say(f"Serving KML on http://localhost:{self.kml_settings.port}/")


def init(mpstate):
    return KMLGenModule(mpstate)
