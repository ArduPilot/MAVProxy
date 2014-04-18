'''
readline handling for mavproxy
'''

import sys, glob, os

rline_mpstate = None

class rline(object):
    '''async readline abstraction'''
    def __init__(self, prompt, mpstate):
        import threading
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
            '(ALIAS)' : complete_alias
            }
        
    def set_prompt(self, prompt):
        if prompt != self.prompt:
            self.prompt = prompt
            sys.stdout.write(prompt)


def complete_alias(text):
    '''return list of aliases'''
    global rline_mpstate
    return rline_mpstate.aliases.keys()

def complete_command(text):
    '''return list of commands'''
    global rline_mpstate
    return rline_mpstate.command_map.keys()

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
    return rline_mpstate.mav_param.keys()

def complete_variable(text):
    '''complete a MAVLink variable'''
    if text.find('.') != -1:
        var = text.split('.')[0]
        if var in rline_mpstate.status.msgs:
            ret = []
            for f in rline_mpstate.status.msgs[var].get_fieldnames():
                ret.append(var + '.' + f)
            return ret
        return []
    return rline_mpstate.status.msgs.keys()

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

    # check it matches so far
    for i in range(len(cmd)-1):
        if not rule_match(rule_components[i], cmd[i]):
            return []

    # expand the next rule component
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
    cmd = readline.get_line_buffer().split(' ')

    if len(cmd) == 1:
        # if on first part then complete on commands and aliases
        last_clist = complete_command(text) + complete_alias(text)
    elif cmd[0] in rline_mpstate.completions:
        # we have a completion rule for this command
        last_clist = complete_rules(rline_mpstate.completions[cmd[0]], cmd[1:])
    else:
        # assume completion by filename
        last_clist = glob.glob(text+'*')
    ret = []
    for c in last_clist:
        if c.startswith(text):
            ret.append(c)
    ret.append(None)
    last_clist = ret
    return last_clist[state]
    
    

# some python distributions don't have readline, so handle that case
# with a try/except
try:
    import readline
    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)
except Exception:
    pass
