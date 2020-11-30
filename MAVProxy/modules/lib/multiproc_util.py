'''Multiprocessing utilities

    This module contains utilties for multiprocessing such as
    custom pickle functions and class wrappers
'''

import copyreg
import mmap
import platform
import struct

def pickle_struct(s):
    '''Custom pickle function for struct.Struct'''

    return struct.Struct, (s.format, )

copyreg.pickle(struct.Struct, pickle_struct)

class WrapFileHandle(object):
    '''Wrap a filehandle object for pickling
    
        This is a pickleable wrapper for a filehandle object
        returned from the builtin function open(file, mode='r', ...)

        The wrapper restores the filehandle seek pointer (offset) and preserves
        the following arguments to open():

            - name (file)
            - mode
            - encoding
            - errors
            - newline
    '''
    
    def __init__(self, filehandle):
        # the filehandle is not pickled
        self._filehandle = filehandle

        # state for pickling - populated in __getstate__
        self._name = None
        self._mode = None
        self._encoding = None
        self._errors = None
        self._newlines = None
        self._offset = None

        # TODO: these are additional args to open() but it is less 
        # clear where to acquire their state when persisting
        # self._buffering = None
        # self._closefd = None
        # self._opener = None

    def unwrap(self):
        '''Returns the encapsulated object'''

        return self._filehandle

    def __getstate__(self):
        # capture the filehandle state
        self._name = self._filehandle.name
        self._mode = self._filehandle.mode
        self._encoding = self._filehandle.encoding
        self._errors = self._filehandle.errors
        self._newlines = self._filehandle.newlines
        self._offset = self._filehandle.tell()
        
        # copy the dict since we change it
        odict = self.__dict__.copy()

        # remove the _filehandle entry
        del odict['_filehandle']

        return odict

    def __setstate__(self, dict):
        # reopen file
        name = dict['_name']
        mode = dict['_mode']
        encoding = dict['_encoding']
        errors = dict['_errors']
        newlines = dict['_newlines']
        filehandle = open(name, mode=mode, encoding=encoding, errors=errors, newline=newlines)
        
        # set the seek pointer
        offset = dict['_offset']
        filehandle.seek(offset)

        # update attributes
        self.__dict__.update(dict)

        # restore the filehandle
        self._filehandle = filehandle  

class WrapMMap(object):
    '''Wrap a memory map object for pickling
    
        This is a pickleable wrapper for a mmap.mmap object
        returned from the builtin function mmap.mmap(fileno, length, ...)

        The wrapper restores the mmap's seek pointer (offset).    
    '''

    # mmap does not retain the filehandle and data_len as attributes
    # when it is constructed, so it is necessary to pass them as args
    # to the wrapper 
    def __init__(self, mm, filehandle, data_len):
        # the mmap is not pickled
        self._mm = mm

        # state for pickling - populated in __getstate__
        self._filehandle = WrapFileHandle(filehandle)
        self._data_len = data_len
        self._offset = None

    def unwrap(self):
        '''Returns the encapsulated object'''

        return self._mm

    def __getstate__(self):
        # capture the mmap state
        self._offset = self._mm.tell()
        
        # copy the dict since we change it
        odict = self.__dict__.copy()

        # remove the _mm entry
        del odict['_mm']

        return odict

    def __setstate__(self, dict):
        # reopen mmap
        filehandle = dict['_filehandle'].unwrap()
        data_len = dict['_data_len']

        mm = None
        if platform.system() == "Windows":
            mm = mmap.mmap(filehandle.fileno(), data_len, None, mmap.ACCESS_READ)
        else:
            mm = mmap.mmap(filehandle.fileno(), data_len, mmap.MAP_PRIVATE, mmap.PROT_READ)

        # set the seek pointer
        offset = dict['_offset']
        mm.seek(offset)

        # update attributes
        self.__dict__.update(dict)

        # restore the mmap
        self._mm = mm  
