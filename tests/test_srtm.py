"""Recovery of invalid terrain cache entries and atomic tile downloads."""
import io
import os
from pathlib import Path
import pickle
import sys
import tempfile
import unittest
from unittest import mock
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from MAVProxy.modules.lib import srtm


class InlineProcess:
    def __init__(self, target, args):
        self.target = target
        self.args = args

    def start(self):
        self.target(*self.args)

    def is_alive(self):
        return False


class SRTMCacheTests(unittest.TestCase):
    filename = 'S36E148.hgt.zip'
    latitude = -35.27845654154013
    longitude = 148.95347139693465

    @classmethod
    def setUpClass(cls):
        cls.samples = (600).to_bytes(2, 'big', signed=True) * (1201 * 1201)
        cls.valid_zip = cls.make_zip(cls.samples)

    @staticmethod
    def make_zip(samples, compression=zipfile.ZIP_DEFLATED):
        output = io.BytesIO()
        with zipfile.ZipFile(output, 'w', compression=compression) as archive:
            archive.writestr('S36E148.hgt', samples)
        return output.getvalue()

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.cachedir = Path(directory.name)
        self.tilepath = self.cachedir / self.filename
        self.downloader = srtm.SRTMDownloader(cachedir=str(self.cachedir))
        self.downloader.filelist = {(-36, 148): ('Australia/', self.filename)}
        Path(self.downloader.filelist_file).write_bytes(pickle.dumps(self.downloader.filelist))
        for mapping in (srtm.childTileDownload, srtm.childFileListDownload):
            patch = mock.patch.dict(mapping, clear=True)
            patch.start()
            self.addCleanup(patch.stop)
        patch = mock.patch.object(srtm.mp_util, 'child_close_fds')
        patch.start()
        self.addCleanup(patch.stop)

    def test_cached_http_error_is_replaced_and_elevation_recovers(self):
        self.tilepath.write_bytes(b'<html><title>404 Not Found</title></html>')
        with mock.patch.object(srtm.multiproc, 'Process', side_effect=InlineProcess) as process, \
                mock.patch.object(self.downloader, 'getURIWithRedirect', return_value=self.valid_zip) as fetch:
            self.assertEqual(self.downloader.getTile(-36, 148), 0)
            tile = self.downloader.getTile(-36, 148)
            self.assertEqual(tile.getAltitudeFromLatLon(self.latitude, self.longitude), 600)
            process.assert_called_once()
            fetch.assert_called_once_with('/SRTM3/Australia/' + self.filename)
        self.assertEqual(self.tilepath.read_bytes(), self.valid_zip)

    def test_corrupt_zip_payload_is_replaced(self):
        contents = bytearray(self.make_zip(self.samples, zipfile.ZIP_STORED))
        # Damage a sample while leaving the ZIP structure intact (CRC failure).
        contents[30 + len('S36E148.hgt')] ^= 1
        self.tilepath.write_bytes(contents)
        with mock.patch.object(srtm.multiproc, 'Process', side_effect=InlineProcess), \
                mock.patch.object(self.downloader, 'getURIWithRedirect', return_value=self.valid_zip):
            self.assertEqual(self.downloader.getTile(-36, 148), 0)
            self.assertIsInstance(self.downloader.getTile(-36, 148), srtm.SRTMTile)

    def test_invalid_cache_does_not_download_offline(self):
        self.tilepath.write_bytes(b'bad tile')
        self.downloader.offline = 1
        with mock.patch.object(srtm.multiproc, 'Process') as process:
            self.assertEqual(self.downloader.getTile(-36, 148), 0)
            process.assert_not_called()
        self.assertEqual(self.tilepath.read_bytes(), b'bad tile')

    def test_failed_download_preserves_existing_cache(self):
        for response in (None, b'<html>error</html>', self.valid_zip[:100],
                         self.make_zip(self.samples + b'x')):
            with self.subTest(response_size=None if response is None else len(response)):
                self.tilepath.write_bytes(self.valid_zip)
                with mock.patch.object(self.downloader, 'getURIWithRedirect', return_value=response), \
                        mock.patch('builtins.print'):
                    self.downloader.downloadTile('Australia/', self.filename)
                self.assertEqual(self.tilepath.read_bytes(), self.valid_zip)
                self.assertEqual(list(self.cachedir.glob('*.tmp')), [])

    def test_replacement_is_validated_before_becoming_visible(self):
        self.tilepath.write_bytes(b'old invalid tile')
        replace = os.replace

        def check_replace(source, destination):
            self.assertEqual(self.tilepath.read_bytes(), b'old invalid tile')
            tile = srtm.SRTMTile(source, -36, 148)
            self.assertEqual(tile.getAltitudeFromLatLon(self.latitude, self.longitude), 600)
            replace(source, destination)

        with mock.patch.object(self.downloader, 'getURIWithRedirect', return_value=self.valid_zip), \
                mock.patch.object(srtm.os, 'replace', side_effect=check_replace) as publish:
            self.downloader.downloadTile('Australia/', self.filename)
            publish.assert_called_once()
        self.assertEqual(self.tilepath.read_bytes(), self.valid_zip)
        self.assertEqual(list(self.cachedir.glob('*.tmp')), [])


if __name__ == '__main__':
    unittest.main()
