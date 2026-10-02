"""Mission FTP results must not overwrite edits made during a download."""
import types
import unittest
from unittest import mock

try:
    import wx  # noqa: F401
except ImportError:
    raise unittest.SkipTest('requires wxPython (no display needed)')

from MAVProxy.modules.lib import wx_util
with mock.patch.object(wx_util, 'safe', True):
    from MAVProxy.modules.mavproxy_misseditor import missionEditorFrame as frame
from MAVProxy.modules.mavproxy_misseditor import me_event
from pymavlink import mavwp


class MissionFTPResultTests(unittest.TestCase):
    def setUp(self):
        self.editor = types.SimpleNamespace(
            button_read_wps=mock.Mock(), button_write_wps=mock.Mock(),
            mission_revision=2, ftp_revision=2, load_wploader=mock.Mock(),
            set_modified_state=mock.Mock(), SetStatusText=mock.Mock())
        self.loader = mavwp.MAVWPLoader()
        self.event = me_event.MissionEditorEvent(me_event.MEGE_FTP_MISSION, wploader=self.loader)

    def test_download_replaces_grid_when_no_local_edits_were_made(self):
        frame.MissionEditorFrame.process_gui_event(self.editor, self.event)
        self.editor.load_wploader.assert_called_once_with(self.loader)
        self.editor.SetStatusText.assert_called_once_with('MAVFTP: Read succeeded (0 waypoints)')
        self.editor.button_read_wps.Enable.assert_called_once()
        self.editor.button_write_wps.Enable.assert_called_once()

    def test_download_keeps_newer_local_edits_and_modified_state(self):
        self.editor.mission_revision += 1
        frame.MissionEditorFrame.process_gui_event(self.editor, self.event)
        self.editor.load_wploader.assert_not_called()
        self.editor.set_modified_state.assert_not_called()
        self.assertIn('local edits kept', self.editor.SetStatusText.call_args.args[0])
        self.editor.button_read_wps.Enable.assert_called_once()
        self.editor.button_write_wps.Enable.assert_called_once()

    def test_write_completion_only_syncs_the_revision_that_was_uploaded(self):
        event = me_event.MissionEditorEvent(me_event.MEGE_FTP_TRANSFER,
                                            success=True, message='MAVFTP: Write succeeded')
        self.editor.mission_revision += 1
        frame.MissionEditorFrame.process_gui_event(self.editor, event)
        self.editor.set_modified_state.assert_not_called()
        self.editor.mission_revision = self.editor.ftp_revision
        frame.MissionEditorFrame.process_gui_event(self.editor, event)
        self.editor.set_modified_state.assert_called_once_with(False)


if __name__ == '__main__':
    unittest.main()
