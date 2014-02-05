'''
readline handling for mavproxy
'''

import sys, glob

class rline(object):
    '''async readline abstraction'''
    def __init__(self, prompt):
        import threading
        self.prompt = prompt
        self.line = None

    def set_prompt(self, prompt):
        if prompt != self.prompt:
            self.prompt = prompt
            sys.stdout.write(prompt)


def complete(text, state):
    return (glob.glob(text+'*')+[None])[state]

# some python distributions don't have readline, so handle that case
# with a try/except
try:
    import readline
    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)
except Exception:
    pass
