'''
Command validator and command completion classes
for prompt_toolkit
'''
from __future__ import unicode_literals
import sys, glob, os, platform
from prompt_toolkit import prompt
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.validation import Validator, ValidationError
from prompt_toolkit.token import Token

class MAVValidator(Validator):
    '''Input validator for Prompt_Toolkit'''
    def __init__(self, mpstate):
        self.state = mpstate
    
    def validate(self, document):
        #need to handle null input (prompt toolkit crashes with null input) - substitute in a space instead
        if document.text == "":
            self.state.input_queue.put(" ")

class MAVPromptToken():
    '''Get the current flight mode as the prompt'''
    def __init__(self, mpstate):
        self.state = mpstate
        
    def get_prompt_token(self, cli):
        " Tokens to be shown before the prompt. "
        if self.state.status.prompt:
            return [(Token.Prompt, self.state.status.prompt), (Token.Prompt, '> ')]
        else:
            return [(Token.Prompt, "MAV"), (Token.Prompt, "> ")]

class MAVPromptCompleter(Completer):
    '''Completion generator for commands'''
    def __init__(self, mpstate):
        self.mpstate = mpstate
        self.last_clist = None
        # other modules can add their own completion functions
        
        self.completion_functions = {
            '(FILENAME)' : self.complete_filename,
            '(PARAMETER)' : self.complete_parameter,
            '(VARIABLE)' : self.complete_variable,
            '(SETTING)' : self.mpstate.settings.completion,
            '(COMMAND)' : self.complete_command,
            '(ALIAS)' : self.complete_alias,
            '(AVAILMODULES)' : self.complete_modules,
            '(LOADEDMODULES)' : self.complete_loadedmodules
            }  
            
    def get_completions(self, document, complete_event):
        '''yield all the possible completion strings'''
        text = document.text_before_cursor
        cmd = text.split(' ')
        final_completor = cmd[-1]

        if len(cmd) != 0 and cmd[0] in self.mpstate.completions:
            # we have a completion rule for this command
            self.last_clist = self.complete_rules(self.mpstate.completions[cmd[0]], cmd[1:])
        elif len(cmd) == 0 or len(cmd) == 1:
            # if on first part then complete on commands and aliases
            self.last_clist = self.complete_command(text) + self.complete_alias(text)
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
        

    def complete_alias(self, text):
        '''return list of aliases'''
        return self.mpstate.aliases.keys()

    def complete_command(self, text):
        '''return list of commands'''
        return self.mpstate.command_map.keys()

    def complete_loadedmodules(self, text):
        return [ m.name for (m,pm) in self.mpstate.modules ]

    def complete_modules(self, text):
        '''complete mavproxy module names'''
        import MAVProxy.modules, pkgutil
        modlist = [x[1] for x in pkgutil.iter_modules(MAVProxy.modules.__path__)]
        ret = []
        loaded = set(self.complete_loadedmodules(''))
        for m in modlist:
            if not m.startswith("mavproxy_"):
                continue
            name = m[9:]
            if not name in loaded:
                ret.append(name)
        return ret

    def complete_filename(self, text):
        '''complete a filename'''

        #ensure directories have trailing slashes:
        list = glob.glob(text+'*')
        for idx, val in enumerate(list):
            if os.path.isdir(val):
                list[idx] = (val + os.path.sep)

        return list

    def complete_parameter(self, text):
        '''complete a parameter'''
        return self.mpstate.mav_param.keys()

    def complete_variable(self, text):
        '''complete a MAVLink variable'''
        if text.find('.') != -1:
            var = text.split('.')[0]
            if var in self.mpstate.status.msgs:
                ret = []
                for f in self.mpstate.status.msgs[var].get_fieldnames():
                    ret.append(var + '.' + f)
                return ret
            return []
        return self.mpstate.status.msgs.keys()

    def rule_expand(self, component, text):
        '''expand one rule component'''
        if component[0] == '<' and component[-1] == '>':
            return component[1:-1].split('|')
        if component in self.completion_functions:
            return self.completion_functions[component](text)
        return [component]

    def rule_match(self, component, cmd):
        '''see if one rule component matches'''
        if component == cmd:
            return True
        expanded = self.rule_expand(component, cmd)
        if cmd in expanded:
            return True
        return False

    def complete_rule(self, rule, cmd):
        '''complete using one rule'''
        rule_components = rule.split(' ')

        if len(cmd) > len(rule_components):
            return []
        #  complete the empty string (e.g "graph <TAB><TAB>")
        if len(cmd) == 0:
            return self.rule_expand(rule_components[0], "")
        # check it matches so far
        for i in range(len(cmd)-1):
            if not self.rule_match(rule_components[i], cmd[i]):
                return []
        # expand the next rule component
        expanded = self.rule_expand(rule_components[len(cmd)-1], cmd[-1])
        return expanded


    def complete_rules(self, rules, cmd):
        '''complete using a list of completion rules'''
        if not isinstance(rules, list):
            rules = [rules]
        ret = []
        for r in rules:
            ret += self.complete_rule(r, cmd)
        return ret


if __name__ == "__main__":
    from prompt_toolkit import prompt
    from prompt_toolkit.history import InMemoryHistory
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
            
            self.completor = MAVPromptCompleter(self)
            self.validator = MAVValidator(self) 
            self.consoleHistory = InMemoryHistory()

    state = mystate()
    while True:
        line = prompt("test" + "> ", history=state.consoleHistory, patch_stdout=True, validator=state.validator, completer=state.completor)
        print("Got: %s" % line)
