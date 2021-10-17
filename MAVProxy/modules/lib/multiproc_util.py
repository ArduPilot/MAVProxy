'''Multiprocessing utilities

    This module contains utilties for multiprocessing such as
    custom pickle functions and class wrappers
'''

from MAVProxy.modules.lib import multiproc

import copyreg
import mmap
import os
import platform
import struct
import threading

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

mutex = multiproc.Lock()

class MPChildTask(object):
    '''Manage a MAVProxy child task
    
        MAVProxy child tasks typically require access to the dataflash
        or telemetry logs. For processes started using the `spawn`
        start method this requires the arguments to the child process
        to be pickleable, which is not the case for file handles and 
        memory mapped files. 

        This class provides two functions `wrap` and `unwrap` that are
        called before and after the parent spawns the child process,
        and in the child process itself before the public child task
        is called. Custom pickle functions and class wrappers / unwrappers
        should be placed in these two functions.
    '''

    def __init__(self, *args, **kwargs):
        self._child_pipe_recv, self._parent_pipe_send = multiproc.Pipe(duplex=False)
        self._close_event = multiproc.Event()
        self._close_event.clear()
        self._child = None

    # @abstractmethod
    def wrap(self):
        '''Apply custom pickle wrappers to non-pickleable attributes'''
        pass

    # @abstractmethod
    def unwrap(self):
        '''Unwrap custom pickle wrappers of non-pickleable attributes'''
        pass

    # @abstractmethod
    def child_task(self):
        '''Launch the child function or application'''
        pass

    def start(self):
        '''Apply custom pickle wrappers then start the child process'''

        # apply lock while the task is started as wrapping may mutate state
        with mutex:
            self.wrap()
            try:
                if os.name == 'nt':
                    # Windows can't pickle the mavlog object, thus can't spin off
                    self._child = threading.Thread(target=self._child_task)
                else:
                    self._child = multiproc.Process(target=self._child_task)
                self._child.start()
            finally:
                if not os.name == 'nt':
                    self.unwrap()

        # deny the parent access to the child end of the pipe
        self.child_pipe_recv.close()

    def _child_task(self):
        '''A non-public child task that calls unwrap before running the public child task'''
        
        # deny the child access to the parent end of the pipe and unwrap
        self.parent_pipe_send.close()
        self.unwrap()

        # call child task (in sub-class)
        self.child_task()

        # set the close event when the task is complete
        self.close_event.set()

    @property
    def child_pipe_recv(self):
        '''The child end of the pipe for receiving data from the parent'''

        return self._child_pipe_recv

    @property
    def parent_pipe_send(self):
        '''The parent end of the pipe for sending data to the child'''

        return self._parent_pipe_send

    @property
    def close_event(self):
        '''A multiprocessing close event'''

        return self._close_event

    @property
    def child(self):
        '''The child process'''

        return self._child

    def close(self):
        '''Set the close event and join the process'''

        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''Returns True if the child process is alive'''

        return self.child.is_alive()

class MPDataLogChildTask(MPChildTask):
    '''Manage a MAVProxy child task that expects a dataflash or telemetry log'''

    def __init__(self, *args, **kwargs):
        '''
        Parameters
        ----------
        mlog : DFReader
            A dataflash or telemetry log
        '''
        super(MPDataLogChildTask, self).__init__(*args, **kwargs)

        # all attributes are implicitly passed to the child process 
        self._mlog = kwargs['mlog']

    # @override
    def wrap(self):
        '''Apply custom pickle wrappers to non-pickleable attributes'''

        # wrap filehandle and mmap in mlog for pickling
        filehandle = self._mlog.filehandle
        data_map = self._mlog.data_map
        data_len = self._mlog.data_len
        self._mlog.filehandle = WrapFileHandle(filehandle)
        self._mlog.data_map = WrapMMap(data_map, filehandle, data_len)

    # @override
    def unwrap(self):
        '''Unwrap custom pickle wrappers of non-pickleable attributes'''

        # restore the state of mlog
        self._mlog.filehandle = self._mlog.filehandle.unwrap()
        self._mlog.data_map = self._mlog.data_map.unwrap()

    @property
    def mlog(self):
        '''The dataflash log (DFReader)'''

        return self._mlog
