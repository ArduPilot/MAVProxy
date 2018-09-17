'''
variable substitution in strings using a dictionary
'''

import sys

class MAVSubstituteError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()
    def __str__(self):
        return self.message

class MAVSubstitute(object):
    '''simple templating system'''
    def __init__(self,
                 start_var_token="${", 
                 end_var_token="}", 
                 checkmissing=True):
        self.start_var_token = start_var_token
        self.end_var_token = end_var_token
        self.checkmissing = checkmissing

    def find_end(self, text, start_token, end_token, ignore_end_token=None):
        '''find the of a token.
        Returns the offset in the string immediately after the matching end_token'''
        if not text.startswith(start_token):
            raise MAVSubstituteError("invalid token start")
        offset = len(start_token)
        nesting = 1
        while nesting > 0:
            idx1 = text[offset:].find(start_token)
            idx2 = text[offset:].find(end_token)
            # Check for false positives due to another similar token
            # For example, make sure idx2 points to the second '}' in ${{field: ${name}}}
            if ignore_end_token:
                combined_token = ignore_end_token + end_token
                if text[offset+idx2:offset+idx2+len(combined_token)] == combined_token:
                    idx2 += len(ignore_end_token)
            if idx1 == -1 and idx2 == -1:
                raise MAVSubstituteError("token nesting error")
            if idx1 == -1 or idx1 > idx2:
                offset += idx2 + len(end_token)
                nesting -= 1
            else:
                offset += idx1 + len(start_token)
                nesting += 1
        return offset

    def find_var_end(self, text):
        '''find the of a variable'''
        return self.find_end(text, self.start_var_token, self.end_var_token)

    def substitute(self, text, subvars={},
                   checkmissing=None):
        '''substitute variables in a string'''

        if checkmissing is None:
            checkmissing = self.checkmissing

        while True:
            idx = text.find(self.start_var_token)
            if idx == -1:
                return text
            endidx = text[idx:].find(self.end_var_token)
            if endidx == -1:
                raise MAVSubstituteError('missing end of variable: %s' % text[idx:idx+10])
            varname = text[idx+2:idx+endidx]
            fullvar = varname

            # allow default value after a :
            def_value = None
            colon = varname.find(':')
            if colon != -1:
                def_value = varname[colon+1:]
                varname = varname[:colon]

            if varname in subvars:
                value = subvars[varname]
            elif def_value is not None:
                value = def_value
            elif checkmissing:
                raise MAVSubstituteError("unknown variable in '%s%s%s'" % (
                    self.start_var_token, varname, self.end_var_token))
            else:
                return text[0:idx+endidx] + self.substitute(text[idx+endidx:], subvars, checkmissing=False)
            text = text.replace("%s%s%s" % (self.start_var_token, fullvar, self.end_var_token), str(value))
        return text

if __name__ == "__main__":
    import os
    sub = MAVSubstitute()
    for v in sys.argv[1:]:
        print("'%s' -> '%s'" % (v, sub.substitute(v, os.environ)))
