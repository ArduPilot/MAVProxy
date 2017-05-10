# preflight_bc.py

from __future__ import with_statement
import itertools
from pyke import contexts, pattern, bc_rule

pyke_version = '1.1.1'
compiler_version = 1

def clear_to_takeoff(rule, arg_patterns, arg_context):
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
        engine.get_ke('questions', 'takeoff_clearnace').reset()
        with engine.prove(rule.rule_base.root_name, 'clearance', context,
                          ()) \
          as gen_2:
          for x_2 in gen_2:
            assert x_2 is None, \
              "preflight.clear_to_takeoff: got unexpected plan from when clause 2"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def clearance(rule, arg_patterns, arg_context):
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
        with engine.prove('questions', 'takeoff_clearance', context,
                          (rule.pattern(0),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "preflight.clearance: got unexpected plan from when clause 1"
            rule.rule_base.num_bc_rule_successes += 1
            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def populate(engine):
  This_rule_base = engine.get_create('preflight')
  
  bc_rule.bc_rule('clear_to_takeoff', This_rule_base, 'clear_to_takeoff',
                  clear_to_takeoff, None,
                  (),
                  (),
                  ())
  
  bc_rule.bc_rule('clearance', This_rule_base, 'clearance',
                  clearance, None,
                  (),
                  (),
                  (pattern.pattern_literal(2),))


Krb_filename = '../preflight.krb'
Krb_lineno_map = (
    ((16, 20), (2, 2)),
    ((22, 22), (5, 5)),
    ((23, 28), (6, 6)),
    ((41, 45), (9, 9)),
    ((47, 52), (11, 11)),
)
