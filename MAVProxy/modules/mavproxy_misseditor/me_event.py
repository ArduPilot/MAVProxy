#!/usr/bin/env python
'''
Event class and enums for Mission Editor
Michael Day
June 2014
'''
#MissionEditorEvents come FROM the GUI (with a few exceptions where the Mission Editor Module sends a message to itself, e.g., MEE_TIME_TO_QUIT)
#MissionEditorGUIEvents go TO the GUI
#enum for MissionEditorEvent types
MEE_READ_WPS = 0
MEE_WRITE_WPS = 1
MEE_TIME_TO_QUIT = 2
MEE_GET_WP_RAD = 3
MEE_GET_LOIT_RAD = 4
MEE_GET_WP_DEFAULT_ALT = 5
MEE_WRITE_WP_NUM = 6
MEE_LOAD_WP_FILE = 7
MEE_SAVE_WP_FILE = 8
MEE_SET_WP_RAD = 9
MEE_SET_LOIT_RAD = 10
MEE_SET_WP_DEFAULT_ALT = 11
#enum of MissionEditorGUIEvent types
MEGE_CLEAR_MISS_TABLE = 0
MEGE_ADD_MISS_TABLE_ROWS = 1
MEGE_SET_MISS_ITEM = 2
MEGE_SET_WP_RAD = 3
MEGE_SET_LOIT_RAD = 4
MEGE_SET_WP_DEFAULT_ALT = 5
MEGE_SET_LAST_MAP_CLICK_POS = 6

class MissionEditorEvent:        
    def __init__(self, type, **kwargs):
        self.type = type
        self.arg_dict = kwargs

        if not self.type in [MEE_READ_WPS, MEE_WRITE_WPS, MEGE_CLEAR_MISS_TABLE,
                             MEGE_ADD_MISS_TABLE_ROWS, MEGE_SET_MISS_ITEM, MEE_TIME_TO_QUIT,
                             MEE_GET_WP_RAD, MEE_GET_LOIT_RAD, MEGE_SET_WP_RAD, MEGE_SET_LOIT_RAD,
                             MEE_GET_WP_DEFAULT_ALT, MEGE_SET_WP_DEFAULT_ALT, MEE_WRITE_WP_NUM,
                             MEE_LOAD_WP_FILE, MEE_SAVE_WP_FILE, MEE_SET_WP_RAD, MEE_SET_LOIT_RAD,
                             MEE_SET_WP_DEFAULT_ALT]:
            raise TypeError("Unrecongized MissionEditorEvent type:" + str(self.type))

    def get_type(self):
        return self.type
    
    def get_arg(self, key):
        if not key in self.arg_dict:
            print("No key %s in %s" % (key, str(self.type)))
            return None
        return self.arg_dict[key]
