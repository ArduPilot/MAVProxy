# pyke_demo_bc.py

from __future__ import with_statement
import itertools
from pyke import contexts, pattern, bc_rule

pyke_version = '1.1.1'
compiler_version = 1

def knows_pattern_matching(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        engine.get_ke('questions', 'pat_master').reset()
        with engine.prove(rule.rule_base.root_name, 'knows_pat_master', context,
                          ()) \
          as gen_2:
          for x_2 in gen_2:
            assert x_2 is None, \
              "pyke_demo.knows_pattern_matching: got unexpected plan from when clause 2"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_pattern_matching_2(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'learned_pattern_matching', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_pattern_matching_2: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_pat_master(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_pat_master: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def learned_pattern_matching(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'taught_pattern_matching', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.learned_pattern_matching: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_pattern_matching', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.learned_pattern_matching: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def taught_pattern_matching_1_2_4_16(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.taught_pattern_matching_1_2_4_16: got unexpected plan from when clause 1"
            if context.lookup_data('ans') in (1, 2, 4, 16):
              with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_syntax', context,
                                ()) \
                as gen_3:
                for x_3 in gen_3:
                  assert x_3 is None, \
                    "pyke_demo.taught_pattern_matching_1_2_4_16: got unexpected plan from when clause 3"
                  with engine.prove(rule.rule_base.root_name, 'knows_patterns', context,
                                    ()) \
                    as gen_4:
                    for x_4 in gen_4:
                      assert x_4 is None, \
                        "pyke_demo.taught_pattern_matching_1_2_4_16: got unexpected plan from when clause 4"
                      with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                                        ()) \
                        as gen_5:
                        for x_5 in gen_5:
                          assert x_5 is None, \
                            "pyke_demo.taught_pattern_matching_1_2_4_16: got unexpected plan from when clause 5"
                          with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                                            ()) \
                            as gen_6:
                            for x_6 in gen_6:
                              assert x_6 is None, \
                                "pyke_demo.taught_pattern_matching_1_2_4_16: got unexpected plan from when clause 6"
                              rule.rule_base.num_bc_rule_successes += 1
                              yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def taught_pattern_matching_3_5_6_9_15(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.taught_pattern_matching_3_5_6_9_15: got unexpected plan from when clause 1"
            if context.lookup_data('ans') in (3, 5, 6, 9, 15):
              with engine.prove(rule.rule_base.root_name, 'knows_tuple_patterns', context,
                                ()) \
                as gen_3:
                for x_3 in gen_3:
                  assert x_3 is None, \
                    "pyke_demo.taught_pattern_matching_3_5_6_9_15: got unexpected plan from when clause 3"
                  rule.rule_base.num_bc_rule_successes += 1
                  yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def taught_pattern_matching_7_8_10(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.taught_pattern_matching_7_8_10: got unexpected plan from when clause 1"
            if context.lookup_data('ans') in (7, 8, 10):
              with engine.prove(rule.rule_base.root_name, 'knows_rest_variable', context,
                                ()) \
                as gen_3:
                for x_3 in gen_3:
                  assert x_3 is None, \
                    "pyke_demo.taught_pattern_matching_7_8_10: got unexpected plan from when clause 3"
                  rule.rule_base.num_bc_rule_successes += 1
                  yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def taught_pattern_matching_11_12(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.taught_pattern_matching_11_12: got unexpected plan from when clause 1"
            if context.lookup_data('ans') in (11, 12):
              with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                                ()) \
                as gen_3:
                for x_3 in gen_3:
                  assert x_3 is None, \
                    "pyke_demo.taught_pattern_matching_11_12: got unexpected plan from when clause 3"
                  rule.rule_base.num_bc_rule_successes += 1
                  yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def taught_pattern_matching_14(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_master', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.taught_pattern_matching_14: got unexpected plan from when clause 1"
            if context.lookup_data('ans') in (14,):
              with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                                ()) \
                as gen_3:
                for x_3 in gen_3:
                  assert x_3 is None, \
                    "pyke_demo.taught_pattern_matching_14: got unexpected plan from when clause 3"
                  with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                                    ()) \
                    as gen_4:
                    for x_4 in gen_4:
                      assert x_4 is None, \
                        "pyke_demo.taught_pattern_matching_14: got unexpected plan from when clause 4"
                      rule.rule_base.num_bc_rule_successes += 1
                      yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_pattern_variable_syntax(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pat_var_syntax', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_pattern_variable_syntax: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def reset_if_wrong__right(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('special', 'claim_goal', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.reset_if_wrong__right: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def reset_if_wrong__wrong(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        engine.get_ke('questions', context.lookup_data('question')).reset()
        rule.rule_base.num_bc_rule_successes += 1
        yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_patterns(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_literal_patterns', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_patterns: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_pattern_variables', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_patterns: got unexpected plan from when clause 2"
                with engine.prove(rule.rule_base.root_name, 'knows_tuple_patterns', context,
                                  ()) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "pyke_demo.knows_patterns: got unexpected plan from when clause 3"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_literal_patterns(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'ask_until_correct', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_literal_patterns: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def ask_until_correct__ask(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', context.lookup_data('question'), context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.ask_until_correct__ask: got unexpected plan from when clause 1"
            with engine.prove('special', 'claim_goal', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.ask_until_correct__ask: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def ask_until_correct__try_again(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        engine.get_ke('questions', context.lookup_data('question')).reset()
        print "Try again:"
        with engine.prove(rule.rule_base.root_name, 'ask_until_correct', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_3:
          for x_3 in gen_3:
            assert x_3 is None, \
              "pyke_demo.ask_until_correct__try_again: got unexpected plan from when clause 3"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_pattern_variables(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_syntax', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_pattern_variables: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_pattern_variables: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_pattern_variable_scope(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'ask_until_correct', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_pattern_variable_scope: got unexpected plan from when clause 1"
            with engine.prove('questions', 'anonymous_syntax', context,
                              (rule.pattern(2),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_pattern_variable_scope: got unexpected plan from when clause 2"
                with engine.prove(rule.rule_base.root_name, 'ask_until_correct', context,
                                  (rule.pattern(3),
                                   rule.pattern(4),)) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "pyke_demo.knows_pattern_variable_scope: got unexpected plan from when clause 3"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_how_patterns_match(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'pattern_scope', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_how_patterns_match: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'post_process_pattern_scope', context,
                              (rule.pattern(0),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_how_patterns_match: got unexpected plan from when clause 2"
                with engine.prove(rule.rule_base.root_name, 'ask_until_correct', context,
                                  (rule.pattern(1),
                                   rule.pattern(2),)) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "pyke_demo.knows_how_patterns_match: got unexpected plan from when clause 3"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_1(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_literal_patterns', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.post_process_pattern_scope_1: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_syntax', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.post_process_pattern_scope_1: got unexpected plan from when clause 2"
                engine.get_ke('questions', 'pattern_scope').reset()
                with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                                  ()) \
                  as gen_4:
                  for x_4 in gen_4:
                    assert x_4 is None, \
                      "pyke_demo.post_process_pattern_scope_1: got unexpected plan from when clause 4"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_2(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.post_process_pattern_scope_2: got unexpected plan from when clause 1"
            engine.get_ke('questions', 'pattern_scope').reset()
            with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                              ()) \
              as gen_3:
              for x_3 in gen_3:
                assert x_3 is None, \
                  "pyke_demo.post_process_pattern_scope_2: got unexpected plan from when clause 3"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_3(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        rule.rule_base.num_bc_rule_successes += 1
        yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_4(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.post_process_pattern_scope_4: got unexpected plan from when clause 1"
            with engine.prove('questions', 'same_var_different_rules', context,
                              (rule.pattern(0),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.post_process_pattern_scope_4: got unexpected plan from when clause 2"
                engine.get_ke('questions', 'pattern_scope').reset()
                with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                                  ()) \
                  as gen_4:
                  for x_4 in gen_4:
                    assert x_4 is None, \
                      "pyke_demo.post_process_pattern_scope_4: got unexpected plan from when clause 4"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_5(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'same_var_different_rules', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.post_process_pattern_scope_5: got unexpected plan from when clause 1"
            engine.get_ke('questions', 'pattern_scope').reset()
            with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                              ()) \
              as gen_3:
              for x_3 in gen_3:
                assert x_3 is None, \
                  "pyke_demo.post_process_pattern_scope_5: got unexpected plan from when clause 3"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_pattern_scope_6(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'knows_patterns', context,
                          ()) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.post_process_pattern_scope_6: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_pattern_variable_scope', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.post_process_pattern_scope_6: got unexpected plan from when clause 2"
                engine.get_ke('questions', 'pattern_scope').reset()
                with engine.prove(rule.rule_base.root_name, 'knows_how_patterns_match', context,
                                  ()) \
                  as gen_4:
                  for x_4 in gen_4:
                    assert x_4 is None, \
                      "pyke_demo.post_process_pattern_scope_6: got unexpected plan from when clause 4"
                    rule.rule_base.num_bc_rule_successes += 1
                    yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_rest_variable(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'rest_pattern_variable_syntax', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_rest_variable: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_rest_match', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_rest_variable: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_rest_match(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'rest_match', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_rest_match: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'post_process_rest_match', context,
                              (rule.pattern(0),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_rest_match: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_rest_match_1(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        engine.get_ke('questions', 'rest_match').reset()
        with engine.prove(rule.rule_base.root_name, 'knows_rest_match', context,
                          ()) \
          as gen_2:
          for x_2 in gen_2:
            assert x_2 is None, \
              "pyke_demo.post_process_rest_match_1: got unexpected plan from when clause 2"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_rest_match_4_5(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        if context.lookup_data('ans') in (4, 5):
          with engine.prove('special', 'claim_goal', context,
                            ()) \
            as gen_2:
            for x_2 in gen_2:
              assert x_2 is None, \
                "pyke_demo.post_process_rest_match_4_5: got unexpected plan from when clause 2"
              engine.get_ke('questions', 'rest_match').reset()
              with engine.prove(rule.rule_base.root_name, 'knows_rest_variable', context,
                                ()) \
                as gen_4:
                for x_4 in gen_4:
                  assert x_4 is None, \
                    "pyke_demo.post_process_rest_match_4_5: got unexpected plan from when clause 4"
                  rule.rule_base.num_bc_rule_successes += 1
                  yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def post_process_rest_match_correct(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        if context.lookup_data('ans') in (2, 3):
          rule.rule_base.num_bc_rule_successes += 1
          yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def knows_tuple_patterns(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(itertools.imap(lambda pat, arg:
                              pat.match_pattern(context, context,
                                                arg, arg_context),
                            patterns,
                            arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('questions', 'tuple_pattern_syntax', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "pyke_demo.knows_tuple_patterns: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'knows_rest_variable', context,
                              ()) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "pyke_demo.knows_tuple_patterns: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def populate(engine):
  This_rule_base = engine.get_create('pyke_demo')
  
  bc_rule.bc_rule('knows_pattern_matching', This_rule_base, 'knows_pattern_matching',
                  knows_pattern_matching, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_pattern_matching_2', This_rule_base, 'knows_pattern_matching',
                  knows_pattern_matching_2, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_pat_master', This_rule_base, 'knows_pat_master',
                  knows_pat_master, None,
                  (),
                  (),
                  (pattern.pattern_literal(13),))
  
  bc_rule.bc_rule('learned_pattern_matching', This_rule_base, 'learned_pattern_matching',
                  learned_pattern_matching, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('taught_pattern_matching_1_2_4_16', This_rule_base, 'taught_pattern_matching',
                  taught_pattern_matching_1_2_4_16, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('taught_pattern_matching_3_5_6_9_15', This_rule_base, 'taught_pattern_matching',
                  taught_pattern_matching_3_5_6_9_15, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('taught_pattern_matching_7_8_10', This_rule_base, 'taught_pattern_matching',
                  taught_pattern_matching_7_8_10, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('taught_pattern_matching_11_12', This_rule_base, 'taught_pattern_matching',
                  taught_pattern_matching_11_12, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('taught_pattern_matching_14', This_rule_base, 'taught_pattern_matching',
                  taught_pattern_matching_14, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('knows_pattern_variable_syntax', This_rule_base, 'knows_pattern_variable_syntax',
                  knows_pattern_variable_syntax, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('reset_if_wrong__right', This_rule_base, 'reset_if_wrong',
                  reset_if_wrong__right, None,
                  (contexts.anonymous('_'),
                   contexts.variable('right_ans'),
                   contexts.variable('right_ans'),),
                  (),
                  ())
  
  bc_rule.bc_rule('reset_if_wrong__wrong', This_rule_base, 'reset_if_wrong',
                  reset_if_wrong__wrong, None,
                  (contexts.variable('question'),
                   contexts.anonymous('_'),
                   contexts.anonymous('_'),),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_patterns', This_rule_base, 'knows_patterns',
                  knows_patterns, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_literal_patterns', This_rule_base, 'knows_literal_patterns',
                  knows_literal_patterns, None,
                  (),
                  (),
                  (pattern.pattern_literal('pat_literal'),
                   pattern.pattern_literal(6),))
  
  bc_rule.bc_rule('ask_until_correct__ask', This_rule_base, 'ask_until_correct',
                  ask_until_correct__ask, None,
                  (contexts.variable('question'),
                   contexts.variable('right_ans'),),
                  (),
                  (contexts.variable('right_ans'),))
  
  bc_rule.bc_rule('ask_until_correct__try_again', This_rule_base, 'ask_until_correct',
                  ask_until_correct__try_again, None,
                  (contexts.variable('question'),
                   contexts.variable('right_ans'),),
                  (),
                  (contexts.variable('question'),
                   contexts.variable('right_ans'),))
  
  bc_rule.bc_rule('knows_pattern_variables', This_rule_base, 'knows_pattern_variables',
                  knows_pattern_variables, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_pattern_variable_scope', This_rule_base, 'knows_pattern_variable_scope',
                  knows_pattern_variable_scope, None,
                  (),
                  (),
                  (pattern.pattern_literal('multiple_matching'),
                   pattern.pattern_literal(4),
                   contexts.anonymous('_'),
                   pattern.pattern_literal('anonymous_matching'),
                   pattern.pattern_literal(3),))
  
  bc_rule.bc_rule('knows_how_patterns_match', This_rule_base, 'knows_how_patterns_match',
                  knows_how_patterns_match, None,
                  (),
                  (),
                  (contexts.variable('ans'),
                   pattern.pattern_literal('anonymous_matching'),
                   pattern.pattern_literal(3),))
  
  bc_rule.bc_rule('post_process_pattern_scope_1', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_1, None,
                  (pattern.pattern_literal(1),),
                  (),
                  ())
  
  bc_rule.bc_rule('post_process_pattern_scope_2', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_2, None,
                  (pattern.pattern_literal(2),),
                  (),
                  ())
  
  bc_rule.bc_rule('post_process_pattern_scope_3', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_3, None,
                  (pattern.pattern_literal(3),),
                  (),
                  ())
  
  bc_rule.bc_rule('post_process_pattern_scope_4', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_4, None,
                  (pattern.pattern_literal(4),),
                  (),
                  (contexts.anonymous('_'),))
  
  bc_rule.bc_rule('post_process_pattern_scope_5', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_5, None,
                  (pattern.pattern_literal(5),),
                  (),
                  (contexts.anonymous('_'),))
  
  bc_rule.bc_rule('post_process_pattern_scope_6', This_rule_base, 'post_process_pattern_scope',
                  post_process_pattern_scope_6, None,
                  (pattern.pattern_literal(6),),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_rest_variable', This_rule_base, 'knows_rest_variable',
                  knows_rest_variable, None,
                  (),
                  (),
                  (contexts.anonymous('_'),))
  
  bc_rule.bc_rule('knows_rest_match', This_rule_base, 'knows_rest_match',
                  knows_rest_match, None,
                  (),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('post_process_rest_match_1', This_rule_base, 'post_process_rest_match',
                  post_process_rest_match_1, None,
                  (pattern.pattern_literal(1),),
                  (),
                  ())
  
  bc_rule.bc_rule('post_process_rest_match_4_5', This_rule_base, 'post_process_rest_match',
                  post_process_rest_match_4_5, None,
                  (contexts.variable('ans'),),
                  (),
                  ())
  
  bc_rule.bc_rule('post_process_rest_match_correct', This_rule_base, 'post_process_rest_match',
                  post_process_rest_match_correct, None,
                  (contexts.variable('ans'),),
                  (),
                  ())
  
  bc_rule.bc_rule('knows_tuple_patterns', This_rule_base, 'knows_tuple_patterns',
                  knows_tuple_patterns, None,
                  (),
                  (),
                  (contexts.anonymous('_'),))


Krb_filename = '../pyke_demo.krb'
Krb_lineno_map = (
    ((16, 20), (3, 3)),
    ((22, 22), (5, 5)),
    ((23, 28), (6, 6)),
    ((41, 45), (9, 9)),
    ((47, 52), (11, 11)),
    ((65, 69), (14, 14)),
    ((71, 76), (16, 16)),
    ((89, 93), (19, 19)),
    ((95, 100), (21, 21)),
    ((101, 106), (22, 22)),
    ((119, 123), (25, 25)),
    ((125, 130), (27, 27)),
    ((131, 131), (28, 28)),
    ((132, 137), (29, 29)),
    ((138, 143), (30, 30)),
    ((144, 149), (31, 31)),
    ((150, 155), (32, 32)),
    ((168, 172), (35, 35)),
    ((174, 179), (37, 37)),
    ((180, 180), (38, 38)),
    ((181, 186), (39, 39)),
    ((199, 203), (42, 42)),
    ((205, 210), (44, 44)),
    ((211, 211), (45, 45)),
    ((212, 217), (46, 46)),
    ((230, 234), (49, 49)),
    ((236, 241), (51, 51)),
    ((242, 242), (52, 52)),
    ((243, 248), (53, 53)),
    ((261, 265), (56, 56)),
    ((267, 272), (58, 58)),
    ((273, 273), (59, 59)),
    ((274, 279), (60, 60)),
    ((280, 285), (61, 61)),
    ((298, 302), (64, 64)),
    ((304, 309), (66, 66)),
    ((322, 326), (70, 70)),
    ((328, 333), (72, 72)),
    ((346, 350), (75, 75)),
    ((352, 352), (77, 77)),
    ((365, 369), (80, 80)),
    ((371, 376), (82, 82)),
    ((377, 382), (83, 83)),
    ((383, 388), (84, 84)),
    ((401, 405), (87, 87)),
    ((407, 413), (89, 89)),
    ((426, 430), (92, 92)),
    ((432, 437), (94, 94)),
    ((438, 443), (95, 95)),
    ((456, 460), (98, 98)),
    ((462, 462), (100, 100)),
    ((463, 463), (101, 101)),
    ((464, 470), (102, 102)),
    ((483, 487), (105, 105)),
    ((489, 494), (107, 107)),
    ((495, 500), (108, 108)),
    ((513, 517), (111, 111)),
    ((519, 525), (113, 113)),
    ((526, 531), (114, 114)),
    ((532, 538), (115, 115)),
    ((551, 555), (118, 118)),
    ((557, 562), (120, 120)),
    ((563, 568), (121, 121)),
    ((569, 575), (122, 122)),
    ((588, 592), (125, 125)),
    ((594, 599), (127, 127)),
    ((600, 605), (128, 128)),
    ((606, 606), (129, 129)),
    ((607, 612), (130, 130)),
    ((625, 629), (133, 133)),
    ((631, 636), (135, 135)),
    ((637, 637), (136, 136)),
    ((638, 643), (137, 137)),
    ((656, 660), (140, 140)),
    ((674, 678), (143, 143)),
    ((680, 685), (145, 145)),
    ((686, 691), (146, 146)),
    ((692, 692), (147, 147)),
    ((693, 698), (148, 148)),
    ((711, 715), (151, 151)),
    ((717, 722), (153, 153)),
    ((723, 723), (154, 154)),
    ((724, 729), (155, 155)),
    ((742, 746), (158, 158)),
    ((748, 753), (160, 160)),
    ((754, 759), (161, 161)),
    ((760, 760), (162, 162)),
    ((761, 766), (163, 163)),
    ((779, 783), (166, 166)),
    ((785, 790), (168, 168)),
    ((791, 796), (169, 169)),
    ((809, 813), (172, 172)),
    ((815, 820), (174, 174)),
    ((821, 826), (175, 175)),
    ((839, 843), (178, 178)),
    ((845, 845), (180, 180)),
    ((846, 851), (181, 181)),
    ((864, 868), (184, 184)),
    ((870, 870), (186, 186)),
    ((871, 876), (187, 187)),
    ((877, 877), (188, 188)),
    ((878, 883), (189, 189)),
    ((896, 900), (192, 192)),
    ((902, 902), (194, 194)),
    ((915, 919), (197, 197)),
    ((921, 926), (199, 199)),
    ((927, 932), (200, 200)),
)
