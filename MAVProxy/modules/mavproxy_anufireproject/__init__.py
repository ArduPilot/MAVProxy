'''
Add functionality to help with the ANU Fire Project
e.g. download data from ACT government website and create KML layers from same

AP_FLAKE8_CLEAN

'''

import json
import math
import os
import pathlib
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util


class ANUFireProject(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ANUFireProject, self).__init__(mpstate, "anufireproject", "")

        self.kml_module_initialised = False

        # stores the directory into which our persistent state should
        # be dropped:
        self.dot_mavproxy_path = mp_util.dot_mavproxy("anufp")
        pathlib.Path(self.dot_mavproxy_path).mkdir(parents=True, exist_ok=True)

        self.add_command(
            'anufp',
            self.cmd,
            "ANU FireProject utility functions",
            ["<burnkml>", "<burnkml> (BURNID)"],
        )

        self.burn_data_url = 'https://services1.arcgis.com/E5n4f1VY84i0xSjy/arcgis/rest/services/Prescribed_Burns_Public/FeatureServer/0/query?f=json&where=1%3D1&spatialRel=esriSpatialRelIntersects&geometry=%7B%22xmin%22%3A16505903.50821903%2C%22ymin%22%3A-4198481.646243979%2C%22xmax%22%3A16627897.005362216%2C%22ymax%22%3A-4144822.852387765%2C%22spatialReference%22%3A%7B%22wkid%22%3A102100%7D%7D&geometryType=esriGeometryEnvelope&inSR=102100&outFields=BURN_NAME%2CBOP_ID%2CHECTARES%2CWORKS_DESCRIPTION%2CBURN_TIMEFRAME%2CESRI_OID&orderByFields=ESRI_OID%20ASC&outSR=102100'  # noqa

        self.burn_json = None
        self.burn_json_time = 0
        self.burn_json_lifetime = 600  # seconds we cache the json for
        self.burn_json_refresh_attempt_time = 0

        try:
            # try not to set one variable without setting the other here.
            cache_path = self.burn_data_cache_path()
            content = cache_path.read_bytes()
            mtime = cache_path.stat().st_mtime
            self.burn_json = content
            self.burn_json_time = mtime
        except FileNotFoundError:
            pass

    def burn_data_cache_path(self):
        '''returns a pathlib object corresponding to the filepath where we
        cache the downloaded burn JSON'''
        return pathlib.Path(os.path.join(self.dot_mavproxy_path, "burn_json.txt"))

    def usage(self):
        '''returns usage string'''
        return "Usage: anufp <burnkml (BURNID)>"

    def cmd(self, args):
        '''handle commands from stdin'''
        if len(args) < 1:
            print(self.usage())
            return

        if args[0] == "burnkml":
            return self.cmd_burnkml(args[1:])

        print(self.usage())

    def get_burn_json_object(self):
        '''returns a json object containing the burn data'''
        self.ensure_burn_json()
        return json.loads(self.burn_json)

    def cmd_burnkml(self, args):
        '''command to trigger creation of a KML layer from a burn ID (e.g. FB511)'''
        kml = self.module('kmlread')
        if kml is None:
            self.message("Need kmlread module")
            return

        if len(args) != 1:
            print(self.usage())
            print("BURNID is usually of the form FBxxx e.g. FB511")
            return
        burn_name = args[0]

        burn_json = self.get_burn_json_object()
        if burn_json is None:
            return

        found_feature = None
        seen_bops = []
        for feature in burn_json["features"]:
            bop_id = feature["attributes"]["BOP_ID"]
            seen_bops.append(bop_id)
            if bop_id == burn_name:
                found_feature = feature
                break
        if found_feature is None:
            self.message(f"Burn {burn_name} not found in downloaded JSON (found {seen_bops})")
            return

        if "geometry" not in feature:
            self.message(f"geometry missing from feature {burn_name}")
            return

        rings = feature["geometry"]["rings"]

        for ring in rings:
            coordinates = []
            for item in ring:
                lat, lon = self.MetersToLatLon(item[0], item[1])
                coordinates.append((lat, lon))

        kml.add_polygon(burn_name, coordinates)

        # from: https://gist.githubusercontent.com/maptiler/fddb5ce33ba995d5523de9afdf8ef118/raw/d7565390d2480bfed3c439df5826f1d9e4b41761/globalmaptiles.py  # noqa
    def MetersToLatLon(self, mx, my):
        "Converts XY point from Spherical Mercator EPSG:900913 to lat/lon in WGS84 Datum"
        originShift = 2 * math.pi * 6378137 / 2.0
        # 20037508.342789244

        lon = (mx / originShift) * 180.0
        lat = (my / originShift) * 180.0

        lat = 180 / math.pi * (2 * math.atan(math.exp(lat * math.pi / 180.0)) - math.pi / 2.0)
        return lat, lon

    def should_refresh_burn_json(self):
        '''returns true if we should download the data from the ACT government
        website again'''
        now = time.time()
        if now - self.burn_json_refresh_attempt_time < 60:
            return False

        if self.burn_json is None:
            return True

        if now - self.burn_json_time > self.burn_json_lifetime:
            return True

        return False

    def message(self, msg):
        '''simply emit msg prefixed with an identifying string'''
        print(f"anufp: {msg}")

    def refresh_burn_json(self):
        '''downloads content from ACT Government URL, caches it in the
        filesystem'''
        tstart = time.time()
        self.message("Refreshing burn data JSON")
        self.burn_json_refresh_attempt_time = tstart
        content = mp_util.download_url(self.burn_data_url)
        if content is None:
            return
        self.burn_json = content
        tstop = time.time()
        self.burn_json_time = tstop

        # cache the data:
        cache_path = self.burn_data_cache_path()
        cache_path.write_bytes(content)

        self.message(f"Refreshed burn data JSON ({tstop-tstart}s)")

    def ensure_burn_json(self):
        '''checks to see if we should refresh the JSON data, and does that if
        required'''
        if not self.should_refresh_burn_json():
            return
        self.refresh_burn_json()

    def idle_task(self):
        # download the burn JSON once at startup to avoid delay on
        # first kml command.
        if self.burn_json is None:
            self.ensure_burn_json()


def init(mpstate):
    '''initialise module'''
    return ANUFireProject(mpstate)
