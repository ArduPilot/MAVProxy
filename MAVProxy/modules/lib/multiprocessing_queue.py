import multiprocessing
import sys

from multiprocessing.queues import Queue
class osxQueue(Queue):
        """ Wrapper around mp.Queue that works on OSX...

        Queue.qsize() relies on sem_getvalue() to get queue size,
        and sem_getvalue() doesn't exist meaningfully on OSX, so we keep our own counter
        """

        def __init__(self, *args, **kwargs):
                Queue.__init__(self, *args, **kwargs)
                self._counter = multiprocessing.RawValue('i', 0)
                self._lock = multiprocessing.Lock()

        def put(self, *args, **kwargs):
                # If the put fails, the exception will prevent us from incrementing the counter
                Queue.put(self, *args, **kwargs)
                with self._lock:
                        self._counter.value += 1

        def get(self, *args, **kwargs):
                # If the get fails, the exception will prevent us from decrementing the counter
                val = Queue.get(self, *args, **kwargs)
                with self._lock:
                        self._counter.value -= 1
                return val

        def qsize(self):
                with self._lock:
                        return self._counter.value

def makeIPCQueue(*args, **kwargs):
        if sys.platform == 'darwin':
                return osxQueue(*args, **kwargs)
        else:
                return Queue(*args, **kwargs)
