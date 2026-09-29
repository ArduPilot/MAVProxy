import importlib.util
from pathlib import Path
import types
import unittest


# Load the worktree file explicitly so the tests always exercise the code
# they accompany, not an installed MAVProxy.
RLINE_PATH = (Path(__file__).resolve().parents[1] /
              'MAVProxy/modules/lib/rline.py')
RLINE_SPEC = importlib.util.spec_from_file_location(
    'rline_under_test', RLINE_PATH)
rline = importlib.util.module_from_spec(RLINE_SPEC)
RLINE_SPEC.loader.exec_module(rline)


class CompleteRulesTest(unittest.TestCase):
    # complete_rules() returns candidates for the current position; complete()
    # filters them by the typed prefix afterwards

    def setUp(self):
        rline.rline_mpstate = types.SimpleNamespace(completion_functions={
            '(LOGNUM)': lambda text: [n for n in ['1', '12', '2'] if n.startswith(text)],
        })

    def test_short_rule_does_not_hide_longer_rules(self):
        # '<foo|bar>' has one component, so it has nothing to complete for a
        # second word, but it must not stop 'foo baz' from completing
        rules = ['<foo|bar>', 'foo baz']
        self.assertEqual(rline.complete_rules(rules, ['foo', 'b']), ['baz'])
        self.assertEqual(rline.complete_rules(rules, ['foo', 'baz', 'q']), [])

    def test_log_download_rules(self):
        # the rule shape from issue #1706
        rules = ['<download|status|erase|resume|cancel|list>',
                 'download all',
                 'download latest',
                 'download range (LOGNUM) (LOGNUM)',
                 'download from (LOGNUM)']
        self.assertEqual(rline.complete_rules(rules, ['download', 'a']),
                         ['all', 'latest', 'range', 'from'])
        self.assertEqual(rline.complete_rules(rules, ['download', 'range', '1']),
                         ['1', '12'])
        self.assertEqual(rline.complete_rules(rules, ['download', 'from', '1', 'x']), [])

    def test_unchanged_behaviour(self):
        rules = ['<status|list>', 'set (LOGNUM)']
        self.assertEqual(rline.complete_rules(rules, []), ['status', 'list', 'set'])
        self.assertEqual(rline.complete_rules(rules, ['s']), ['status', 'list', 'set'])
        self.assertEqual(rline.complete_rules(rules, ['set', '1']), ['1', '12'])
        self.assertEqual(rline.complete_rules(rules, ['other', '1']), [])


if __name__ == '__main__':
    unittest.main()
