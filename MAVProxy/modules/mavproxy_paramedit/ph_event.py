#!/usr/bin/env python
'''
Event class and enums for Parameter Editor

'''
# ParamEditorEvents come FROM the GUI (with a few exceptions where the
# Parameter Editor Module sends a message to itself).
# ParamEditorEditorGUIEvents go to the GUI
# enum for ParamEditorEvent types
PEE_READ_KEY = 0
PEE_LOAD_FILE = 1
PEE_SAVE_FILE = 2
PEE_READ_PARAM = 3
PEE_WRITE_PARAM = 4
PEE_TIME_TO_QUIT = 5
PEE_RESET = 6
PEE_FETCH = 7
# enum of ParamEditorGUIEvent types
PEGE_READ_PARAM = 0
PEGE_REFRESH_PARAM = 1
PEGE_WRITE_SUCC = 2
PEGE_RCIN = 3


class ParamEditorEvent:
    def __init__(self, type, **kwargs):
        self.type = type
        self.arg_dict = kwargs

        if self.type not in [PEE_SAVE_FILE, PEE_LOAD_FILE, PEE_READ_KEY,
                             PEGE_REFRESH_PARAM, PEGE_READ_PARAM, PEGE_RCIN,
                             PEE_READ_PARAM, PEE_WRITE_PARAM, PEE_RESET,
                             PEE_TIME_TO_QUIT, PEGE_WRITE_SUCC, PEE_FETCH]:
            raise TypeError("Unrecongized ParamEditorEvent type:" +
                            str(self.type))

    def get_type(self):
        return self.type

    def get_arg(self, key):
        if key not in self.arg_dict:
            print("No key %s in %s" % (key, str(self.type)))
            return None
        return self.arg_dict[key]
