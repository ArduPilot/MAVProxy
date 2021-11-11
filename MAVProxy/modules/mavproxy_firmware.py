#!/usr/bin/env python

from __future__ import print_function

'''firmware handling'''

import time, os, fnmatch
import json
import threading
import re
import subprocess

from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import multiproc
from MAVProxy.modules.lib import mp_settings

class FirmwareModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(FirmwareModule, self).__init__(mpstate, "firmware", "firmware handling", public = True)
        self.firmware_settings = mp_settings.MPSettings(
            [('uploader', str, "uploader.py"),
            ])
        self.add_command('fw', self.cmd_fw, "firmware handling",
                         ["<manifest> (OPT)",
                          "list <filterterm...>",
                          "flash <filterterm...>",
                          "set (FIRMWARESETTING)",
                          "flashfile FILENAME"])
        self.add_completion_function('(FIRMWARESETTING)',
                                     self.firmware_settings.completion)
        self.downloaders_lock = threading.Lock()
        self.downloaders = {}
        self.manifests_parse()

    def usage(self):
        '''show help on a command line options'''
        return '''Usage: fw <manifest|set|list|flash|flashfile>

# Use the column headings from "list" as filter terms

# e.g. list latest releases for PX4-v2 platform:
fw list latest=1 platform=PX4-v2

# e.g. download most recent official PX4-v2 release for quadcopters:
fw download releasetype=OFFICIAL frame=quad platform=PX4-v2
'''

    def cmd_fw_help(self):
        '''show help on fw command'''
        print(self.usage())

    def cmd_fw(self, args):
        '''execute command defined in args'''
        if len(args) == 0:
            print(self.usage())
            return
        rest = args[1:]
        if args[0] == "manifest":
            self.cmd_fw_manifest(rest)
        elif args[0] == "list":
            self.cmd_fw_list(rest)
        elif args[0] == "flash":
            self.cmd_flash(rest)
        elif args[0] == "set":
            self.firmware_settings.command(args[1:])
        elif args[0] == "download":
            self.cmd_fw_download(rest)
        elif args[0] in ["help","usage"]:
            self.cmd_fw_help(rest)
        elif args[0] == "flashfile":
            self.cmd_flashfile(rest)
        else:
            print(self.usage())

    def frame_from_firmware(self, firmware):
        '''extract information from firmware, return pretty string to user'''
        # see Tools/scripts/generate-manifest for this map:
        frame_to_mavlink_dict = {
            "quad": "QUADROTOR",
            "hexa": "HEXAROTOR",
            "y6": "ARDUPILOT_Y6",
            "tri": "TRICOPTER",
            "octa": "OCTOROTOR",
            "octa-quad": "ARDUPILOT_OCTAQUAD",
            "heli": "HELICOPTER",
            "Plane": "FIXED_WING",
            "Tracker": "ANTENNA_TRACKER",
            "Rover": "GROUND_ROVER",
            "PX4IO": "ARDUPILOT_PX4IO",
        }
        mavlink_to_frame_dict = { v : k  for k,v in frame_to_mavlink_dict.items() }
        x = firmware["mav-type"]
        if firmware["mav-autopilot"] != "ARDUPILOTMEGA":
            return x
        if x in mavlink_to_frame_dict:
            return mavlink_to_frame_dict[x]

        return x

    def semver_from_firmware(self, firmware):
        '''Extract a tuple of (major,minor,patch) from a firmware instance'''
        version = firmware.get("mav-firmware-version-major",None)
        if version is None:
            return (None,None,None)
        return (firmware["mav-firmware-version-major"],
                firmware["mav-firmware-version-minor"],
                firmware["mav-firmware-version-patch"])

    def row_is_filtered(self, row_subs, filters):
        '''returns True if row should NOT be included according to filters'''
        for filtername in filters:
            filtervalue = filters[filtername]
            if filtername in row_subs:
                row_subs_value = row_subs[filtername]
                if str(row_subs_value) != str(filtervalue):
                    return True
            else:
                print("fw: Unknown filter keyword (%s)" % (filtername,))
        return False

    def filters_from_args(self, args):
        '''take any argument of the form name=value anmd put it into a dict; return that and the remaining arguments'''
        filters = dict()
        remainder = []
        for arg in args:
            try:
                equals = arg.index('=')
                # anything ofthe form key-value is taken as a filter
                filters[arg[0:equals]] = arg[equals+1:];
            except ValueError:
                remainder.append(arg)
        return (filters,remainder)

    def all_firmwares(self):
        ''' return firmware entries from all manifests'''
        all = []
        for manifest in self.manifests:
            for firmware in manifest["firmware"]:
                all.append(firmware)
        return all

    def rows_for_firmwares(self, firmwares):
        '''provide user-readable text for a firmware entry'''
        rows = []
        i = 0
        for firmware in firmwares:
            frame = self.frame_from_firmware(firmware)
            row = {
                "seq": i,
                "platform": firmware["platform"],
                "frame": frame,
#                "type": firmware["mav-type"],
                "releasetype": firmware["mav-firmware-version-type"],
                "latest": firmware["latest"],
                "git-sha": firmware["git-sha"][0:7],
                "format": firmware["format"],
                "_firmware": firmware,
            }
            (major,minor,patch) = self.semver_from_firmware(firmware)
            if major is None:
                row["version"] = ""
                row["major"] = ""
                row["minor"] = ""
                row["patch"] = ""
            else:
                row["version"] = firmware["mav-firmware-version"]
                row["major"] = major
                row["minor"] = minor
                row["patch"] = patch

            i += 1
            rows.append(row)

        return rows

    def filter_rows(self, filters, rows):
        '''returns rows as filtered by filters'''
        ret = []
        for row in rows:
            if not self.row_is_filtered(row, filters):
                ret.append(row)
        return ret

    def filtered_rows_from_args(self, args):
        '''extracts filters from args, rows from manifests, returns filtered rows'''
        if len(self.manifests) == 0:
            print("fw: No manifests downloaded.  Try 'fw manifest download'")
            return None

        (filters,remainder) = self.filters_from_args(args)
        all = self.all_firmwares()
        rows = self.rows_for_firmwares(all)
        filtered = self.filter_rows(filters, rows)
        return (filtered, remainder)

    def cmd_flashfile(self, args):
        filename = args[0]
        subprocess.check_call([self.firmware_settings.uploader, filename])

    def cmd_flash(self, args):
        '''cmd handler for flash'''
        stuff = self.filtered_rows_from_args(args)
        (filtered, remainder) = stuff
        if stuff is None:
            print("Nothing returned from filter")
            return

        (filtered, remainder) = stuff

        if len(filtered) != 1:
            print("Filter must return single firmware")
            self.list_firmwares(filtered)
            print("Filter must return single firmware")
            return

        firmware = filtered[0]["_firmware"]

        try:
            filename = self.download_firmware(firmware)
        except Exception as e:
            print("fw: download failed")
            print(e)
            return

        subprocess.check_call([self.firmware_settings.uploader, filename])
        return


    def cmd_fw_list(self, args):
        '''cmd handler for list'''
        stuff = self.filtered_rows_from_args(args)
        if stuff is None:
            return
        (filtered, remainder) = stuff
        self.list_firmwares(filtered)

    def list_firmwares(self, filtered):
        print("")
        print(" seq platform frame    major.minor.patch releasetype latest git-sha format")
        for row in filtered:
            print("{seq:>5} {platform:<13} {frame:<10} {version:<10} {releasetype:<9} {latest:<6} {git-sha} {format}".format(**row))
        print(" seq platform frame    major.minor.patch releasetype latest git-sha format")

    def cmd_fw_download(self, args):
        '''cmd handler for downloading firmware'''
        stuff = self.filtered_rows_from_args(args)
        if stuff is None:
            return
        (filtered, remainder) = stuff
        if len(filtered) == 0:
            print("fw: No firmware specified")
            return
        if len(filtered) > 1:
            print("fw: No single firmware specified")
            return

        try:
            self.download_firmware(filtered[0]["_firmware"])
        except Exception as e:
            print("fw: download failed")
            print(e)

    def download_firmware(self, firmware, filename=None):
        url = firmware["url"]

        print("fw: URL: %s"  % (url,))
        if filename is None:
            filename = os.path.basename(url)
        files = []
        files.append((url,filename))
        child = multiproc.Process(target=mp_util.download_files, args=(files,))
        child.start()
        tstart = time.time()
        while True:
            if time.time() - tstart > 60:
                print("Download timeout")
            if not child.is_alive():
                break
            print("Waiting for download to compliete...")
            time.sleep(1)

        return filename

    def fw_manifest_usage(self):
        '''return help on manifest subcommand'''
        return("Usage: fw manifest <list|download|purge>")

    def cmd_fw_manifest_help(self):
        '''show help on manifest subcommand'''
        print(self.fw_manifest_usage())

    def find_manifests(self):
        '''locate manifests and return filepaths thereof'''
        manifest_dir = mp_util.dot_mavproxy()

        ret = []
        for file in os.listdir(manifest_dir):
            try:
                file.index("manifest")
                ret.append(os.path.join(manifest_dir,file))
            except ValueError:
                pass
        return ret

    def cmd_fw_manifest_list(self):
        '''list manifests'''
        for filepath in self.find_manifests():
            print(filepath)

    def cmd_fw_manifest_load(self):
        '''handle command to load manifests - hidden command since these should only change with fw commands'''
        self.manifests_parse()

    def cmd_fw_manifest_purge(self):
        '''remove all downloaded manifests'''
        for filepath in self.find_manifests():
            os.unlink(filepath)
        self.manifests_parse()

    def cmd_fw_manifest(self, args):
        '''cmd handler for manipulating manifests'''
        if len(args) == 0:
            print(self.fw_manifest_usage())
            return
        rest = args[1:]
        if args[0] == "download":
            return self.manifest_download()
        if args[0] == "list":
            return self.cmd_fw_manifest_list()
        if args[0] == "load":
            return self.cmd_fw_manifest_load()
        if args[0] == "purge":
            return self.cmd_fw_manifest_purge()
        if args[0] == "help":
            return self.cmd_fw_manifest_help()
        else:
            print("fw: Unknown manifest option (%s)" % args[0])
            print(fw_manifest_usage())

    def manifest_parse(self, path):
        '''parse manifest at path, return JSON object'''
        print("fw: parsing manifests")
        content = open(path).read()
        return json.loads(content)

    def semver_major(self,semver):
        '''return major part of semver version number. Avoids "import semver"'''
        return int(semver[0:semver.index(".")])

    def manifest_path_is_old(self, path):
        '''return true if path is more than a day old'''
        mtime = os.path.getmtime(path)
        return (time.time() - mtime) > 24*60*60

    def manifests_parse(self):
        '''parse manifests present on system'''
        self.manifests = []
        for manifest_path in self.find_manifests():
            if self.manifest_path_is_old(manifest_path):
                print("fw: Manifest (%s) is old; consider 'fw manifest download'" % (manifest_path))
            manifest = self.manifest_parse(manifest_path)
            if self.semver_major(manifest["format-version"]) != 1:
                print("fw: Manifest (%s) has major version %d; MAVProxy only understands version 1" % (manifest_path,manifest["format-version"]))
                continue
            self.manifests.append(manifest)

    def download_url(self, url, path):
        mp_util.download_files([(url,path)])

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if self.downloaders_lock.acquire(False):
            removed_one = False
            for url in self.downloaders.keys():
                if not self.downloaders[url].is_alive():
                    print("fw: Download thread for (%s) done" % url)
                    del self.downloaders[url]
                    removed_one = True
            if removed_one and not self.downloaders.keys():
                # all downloads finished - parse them
                self.manifests_parse()

            self.downloaders_lock.release()

    def make_safe_filename_from_url(self, url):
        '''return a version of url safe for use as a filename'''
        r = re.compile("([^a-zA-Z0-9_.-])")
        filename = r.sub(lambda m : "%" + str(hex(ord(str(m.group(1))))).upper(), url)
        return filename

    def manifest_download(self):
        '''download manifest files'''
        if self.downloaders_lock.acquire(False):
            if len(self.downloaders):
                # there already exist downloader threads
                self.downloaders_lock.release()
                return

            for url in ['https://firmware.ardupilot.org/manifest.json']:
                filename = self.make_safe_filename_from_url(url)
                path = mp_util.dot_mavproxy("manifest-%s" % filename)
                self.downloaders[url] = threading.Thread(target=self.download_url, args=(url, path))
                self.downloaders[url].start()
            self.downloaders_lock.release()
        else:
            print("fw: Failed to acquire download lock")

def init(mpstate):
    '''initialise module'''
    return FirmwareModule(mpstate)
