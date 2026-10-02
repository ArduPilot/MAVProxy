"""Parameter filtering without a display, using the frame's real filter methods."""
import types
import unittest
from unittest import mock

try:
    from MAVProxy.modules.mavproxy_paramedit import param_editor_frame as frame
except ImportError:
    raise unittest.SkipTest('requires wxPython (no display needed)')
from MAVProxy.modules.mavproxy_paramedit import ph_event


class ParamFilterTests(unittest.TestCase):
    def setUp(self):
        self.editor = types.SimpleNamespace(
            search_choices=['All:', 'Non Default:', 'Tuning:ATC_,PILOT_'],
            search_list=mock.Mock(), search_key=mock.Mock(),
            param_received={'ATC_EQUAL': 1.0, 'ATC_DIFFERENT': 2.0,
                            'PILOT_VALUE': 3.0, 'UNKNOWN': 4.0},
            default_params={'ATC_EQUAL': 1.0, 'ATC_DIFFERENT': 1.0, 'PILOT_VALUE': 0.0},
            htree={}, vehicle_name='ArduCopter', redraw_grid=mock.Mock(),
            SetStatusText=mock.Mock(), GetStatusBar=mock.Mock())
        self.editor.search_key.GetValue.return_value = ''
        self.editor.key_redraw = types.MethodType(frame.ParamEditorFrame.key_redraw, self.editor)

    def select(self, index):
        self.editor.search_list.GetSelection.return_value = index
        self.editor.search_list.GetStringSelection.return_value = self.editor.search_choices[index].split(':')[0]
        self.editor.key_redraw()
        return self.editor.redraw_grid.call_args.args[0]

    def test_non_default_excludes_equal_and_unknown_defaults(self):
        self.assertEqual(self.select(1), {'ATC_DIFFERENT': 2.0, 'PILOT_VALUE': 3.0})

    def test_search_is_applied_to_non_default_subset(self):
        self.editor.search_key.GetValue.return_value = 'atc'
        self.assertEqual(self.select(1), {'ATC_DIFFERENT': 2.0})
        self.editor.htree = {'PILOT_VALUE': {'documentation': 'Climb speed', 'humanName': 'Speed'}}
        self.editor.search_key.GetValue.return_value = 'climb'
        self.assertEqual(self.select(1), {'PILOT_VALUE': 3.0})

    def test_current_values_and_new_defaults_are_reflected_on_redraw(self):
        self.select(1)
        self.editor.param_received['ATC_DIFFERENT'] = 1.0
        self.editor.param_received['ATC_EQUAL'] = 2.0
        self.assertEqual(self.select(1), {'ATC_EQUAL': 2.0, 'PILOT_VALUE': 3.0})
        frame.ParamEditorFrame.process_gui_event(self.editor, ph_event.ParamEditorEvent(
            ph_event.PEGE_DEFAULTS, defaults=dict(self.editor.param_received)))
        self.assertTrue(self.editor.requires_redraw)
        self.assertEqual(self.select(1), {})

    def test_all_and_prefix_categories_still_work(self):
        self.assertEqual(self.select(0), self.editor.param_received)
        self.assertEqual(self.select(2), {'ATC_EQUAL': 1.0, 'ATC_DIFFERENT': 2.0, 'PILOT_VALUE': 3.0})

    def test_missing_defaults_shows_empty_list_and_fetch_hint(self):
        self.editor.default_params = {}
        self.assertEqual(self.select(1), {})
        self.editor.SetStatusText.assert_called_with(frame.DEFAULTS_UNAVAILABLE)
        self.editor.GetStatusBar.return_value.GetStatusText.return_value = frame.DEFAULTS_UNAVAILABLE
        self.select(0)
        self.editor.SetStatusText.assert_called_with('')


if __name__ == '__main__':
    unittest.main()
