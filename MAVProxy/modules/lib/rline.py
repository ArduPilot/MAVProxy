'''
readline handling for mavproxy
'''

import sys, glob, os, platform
from future.builtins import input
import re
from pymavlink import mavutil

# some python distributions don't have readline, so handle that case
# with a try/except
if platform.system() == 'Darwin':
    import gnureadline as readline
elif platform.system() == 'Windows' and sys.version_info >= (3, 0):
    # Python 3.8 defaults to Proactor event loop, which doesn't work well
    if sys.version_info >= (3, 8):
        import asyncio
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    from prompt_toolkit import prompt, PromptSession
    from prompt_toolkit.completion import Completer, Completion
    from prompt_toolkit.shortcuts import CompleteStyle
    from prompt_toolkit.patch_stdout import patch_stdout
else:
    try:
        import readline
    except ImportError:
        import pyreadline as readline
    
rline_mpstate = None
#redisplay = None

class rline(object):
    '''async readline abstraction'''
    def __init__(self, prompt, mpstate):
        global rline_mpstate
        self.prompt = prompt
        rline_mpstate = mpstate
        
        # other modules can add their own completion functions
        mpstate.completion_functions = {
            '(FILENAME)' : complete_filename,
            '(PARAMETER)' : complete_parameter,
            '(VARIABLE)' : complete_variable,
            '(SETTING)' : rline_mpstate.settings.completion,
            '(COMMAND)' : complete_command,
            '(ALIAS)' : complete_alias,
            '(AVAILMODULES)' : complete_modules,
            '(LOADEDMODULES)' : complete_loadedmodules
            }

        if platform.system() == 'Windows' and sys.version_info >= (3, 0):
            # Create key bindings registry with a custom binding for the Tab key that
            # displays completions like GNU readline.
            self.session = PromptSession()
            mpstate.completor = MAVPromptCompleter()

    def set_prompt(self, prompt):
        if prompt != self.prompt:
            self.prompt = prompt
            if platform.system() != 'Windows' or sys.version_info < (3, 0):
                sys.stdout.write(prompt)
                self.redisplay()

    def add_history(self, line):
        '''add a line to the history'''
        if platform.system() == 'Windows' and sys.version_info >= (3, 0):
            self.session.history.append_string(line)
        else:
            readline.add_history(line)
        self.redisplay()

    def redisplay(self):
        '''redisplay prompt'''
        try:
            redisplay()
        except Exception as ex:
            pass
            
    def get_prompt(self):
        '''return the current prompt'''
        return self.prompt
            
    def input(self):
        ''' get user input'''
        ret = ""
        if platform.system() == 'Windows' and sys.version_info >= (3, 0):
            global rline_mpstate
            with patch_stdout():
                return self.session.prompt(self.get_prompt,completer=rline_mpstate.completor, complete_while_typing=False, complete_style=CompleteStyle.READLINE_LIKE, refresh_interval=0.5)
        else:
            return input(self.get_prompt())

if platform.system() == 'Windows' and sys.version_info >= (3, 0):
    # This is the prompt-toolkit setup for Windows
    class MAVPromptCompleter(Completer):
        '''Completion generator for commands'''
        def __init__(self, ):
            #global rline_mpstate
            #self.mpstate = rline_mpstate
            self.last_clist = None
            # other modules can add their own completion functions

        def get_completions(self, document, complete_event):
            '''yield all the possible completion strings'''
            global rline_mpstate
            text = document.text_before_cursor
            #cmd = text.split(' ')
            
            cmd = re.split(" +", text)
            final_completor = cmd[-1]

            if len(cmd) != 0 and cmd[0] in rline_mpstate.completions:
                # we have a completion rule for this command
                self.last_clist = complete_rules(rline_mpstate.completions[cmd[0]], cmd[1:])
            elif len(cmd) == 0 or len(cmd) == 1:
                # if on first part then complete on commands and aliases
                self.last_clist = complete_command(text) + complete_alias(text)
            else:
                # assume completion by filename
                self.last_clist = glob.glob(text+'*')
            ret = []
            for c in self.last_clist:
                if c.startswith(final_completor) or c.startswith(final_completor.upper()):
                    ret.append(c)
            if len(ret) == 0:
                # if we had no matches then try case insensitively
                final_completor = final_completor.lower()
                for c in self.last_clist:
                    if c.lower().startswith(final_completor):
                        ret.append(c)

            for c in ret:
                yield Completion(c, start_position=-len(final_completor))

def complete_alias(text):
    '''return list of aliases'''
    global rline_mpstate
    return list(rline_mpstate.aliases.keys())

def complete_command(text):
    '''return list of commands'''
    global rline_mpstate
    return list(rline_mpstate.command_map.keys())

def complete_loadedmodules(text):
    global rline_mpstate
    return [ m.name for (m,pm) in rline_mpstate.modules ]

def complete_modules(text):
    '''complete mavproxy module names'''
    import MAVProxy.modules, pkgutil
    modlist = [x[1] for x in pkgutil.iter_modules(MAVProxy.modules.__path__)]
    ret = []
    loaded = set(complete_loadedmodules(''))
    for m in modlist:
        if not m.startswith("mavproxy_"):
            continue
        name = m[9:]
        if not name in loaded:
            ret.append(name)
    return ret

def complete_filename(text):
    '''complete a filename'''

    #ensure directories have trailing slashes:
    list = glob.glob(text+'*')
    for idx, val in enumerate(list):
        if os.path.isdir(val):
            list[idx] = (val + os.path.sep)

    return list

def complete_parameter(text):
    '''complete a parameter'''
    global rline_mpstate
    return list(rline_mpstate.mav_param.keys())

def complete_variable(text):
    '''complete a MAVLink variable or expression'''
    global rline_mpstate
    if text == '':
        return list(rline_mpstate.status.msgs.keys())

    if text.endswith(":2"):
        suffix = ":2"
        text = text[:-2]
    else:
        suffix = ''

    m1 = re.match("^(.*?)([A-Z0-9][A-Z0-9_]*(\[[0-9A-Z_]+\])?)[.]([A-Za-z0-9_]*)$", text)
    if m1 is not None:
        prefix = m1.group(1)
        mtype = m1.group(2)
        instance = m1.group(3)
        fname = m1.group(4)
        if mtype in rline_mpstate.status.msgs:
            ret = []
            for f in rline_mpstate.status.msgs[mtype].get_fieldnames():
                if f.startswith(fname):
                    ret.append(prefix + mtype + '.' + f + suffix)
            return ret
        return []

    # handle partially open arrays, like 'NAMED_VALUE_FLOAT[A'
    m1 = re.match("^(.*?)([A-Z0-9][A-Z0-9_]*(\[[0-9A-Z_]+))$", text)
    if m1 is not None:
        prefix = m1.group(1)
        mtype = m1.group(2)
        ret = []
        for k in list(rline_mpstate.status.msgs.keys()):
            if k.startswith(mtype):
                ret.append(prefix + k + suffix)
        if len(ret):
            return ret

    m2 = re.match("^(.*?)([A-Z0-9][A-Z0-9_]*(\[[0-9A-Z_]+\])?)$", text)
    prefix = m2.group(1)
    mtype = m2.group(2)
    ret = []
    for k in list(rline_mpstate.status.msgs.keys()):
        if k.startswith(mtype):
            ret.append(prefix + k + suffix)
    if len(ret):
        return ret

    try:
        if mavutil.evaluate_expression(text, rline_mpstate.status.msgs) is not None:
            return [text+suffix]
    except Exception as ex:
        pass

    return []

def rule_expand(component, text):
    '''expand one rule component'''
    global rline_mpstate
    if component[0] == '<' and component[-1] == '>':
        return component[1:-1].split('|')
    if component in rline_mpstate.completion_functions:
        return rline_mpstate.completion_functions[component](text)
    return [component]

def rule_match(component, cmd):
    '''see if one rule component matches'''
    if component == cmd:
        return True
    expanded = rule_expand(component, cmd)
    if cmd in expanded:
        return True
    return False

def complete_rule(rule, cmd):
    '''complete using one rule'''
    global rline_mpstate
    rule_components = rule.split(' ')
    
    #  complete the empty string (e.g "graph <TAB><TAB>")
    if len(cmd) == 0:
        return rule_expand(rule_components[0], "")

    # check it matches so far
    for i in range(len(cmd)-1):
        if not rule_match(rule_components[i], cmd[i]):
            return []

    # expand the next rule component
    expanded = []
    if platform.system() == 'Windows' and sys.version_info >= (3, 0):
        if len(rule_components) >= len(cmd):
            expanded = rule_expand(rule_components[len(cmd)-1], cmd[-1])
    else:
        expanded = rule_expand(rule_components[len(cmd)-1], cmd[-1])
    return expanded


def complete_rules(rules, cmd):
    '''complete using a list of completion rules'''
    if not isinstance(rules, list):
        rules = [rules]
    ret = []
    for r in rules:
        ret += complete_rule(r, cmd)
    return ret


last_clist = None

def complete(text, state):
    '''completion routine for when user presses tab'''
    global last_clist
    global rline_mpstate
    if state != 0 and last_clist is not None:
        return last_clist[state]

    # split the command so far
    cmd = re.split(" +", readline.get_line_buffer())

    if len(cmd) != 0 and cmd[0] in rline_mpstate.completions:
        # we have a completion rule for this command
        last_clist = complete_rules(rline_mpstate.completions[cmd[0]], cmd[1:])
    elif len(cmd) == 0 or len(cmd) == 1:
        # if on first part then complete on commands and aliases
        last_clist = complete_command(text) + complete_alias(text)
    else:
        # assume completion by filename
        last_clist = glob.glob(text+'*')
    ret = []
    for c in last_clist:
        if c.startswith(text) or c.startswith(text.upper()):
            ret.append(c)
    if len(ret) == 0:
        # if we had no matches then try case insensitively
        text = text.lower()
        for c in last_clist:
            if c.lower().startswith(text):
                ret.append(c)
    ret.append(None)
    last_clist = ret
    return last_clist[state]

# Configure readline for linux/mac systems
if platform.system() != 'Windows' or sys.version_info < (3, 0):
    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)
    #redisplay = readline.redisplay

if __name__ == "__main__":
    from mp_settings import MPSettings, MPSetting

    class mystate(object):
        def __init__(self):
            self.settings = MPSettings(
            [ MPSetting('foo', int, 1, 'foo int', tab='Link', range=(0,4), increment=1),
              MPSetting('bar', float, 4, 'bar float', range=(-1,20), increment=1)])
            self.completions = {
                "script" : ["(FILENAME)"],
                "set"    : ["(SETTING)"]
                }
            self.command_map = {
                'script'  : (None,   'run a script of MAVProxy commands'),
                'set'     : (None,   'mavproxy settings'),
                }
            self.aliases = {}

    state = mystate()
    rl = rline("test> ", state)
    while True:
        line = raw_input(rl.prompt)
        print("Got: %s" % line)
