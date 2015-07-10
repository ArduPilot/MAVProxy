import multiprocessing

class mpCounter(object):
        """ Locked/shared counter """

        def __init__(self, default=0):
                self._value = multiprocessing.RawValue('i', default)
                self._lock = multiprocessing.Lock()

        def add(self, delta):
                with self._lock:
                        self._value.value += delta

        @property
        def value(self):
                with self._lock:
                        return self._value.value

from multiprocessing.queues import Queue
class mpQueue(Queue):
        """ Wrapper around mp.Queue that works on OSX...

        Queue.qsize() relies on sem_getvalue() to get queue size,
        and sem_getvalue() doesn't exist meaningfully on OSX, so we keep our own counter
        """

        def __init__(self, *args, **kwargs):
                Queue.__init__(self, *args, **kwargs)
                self.qsizeCounter = mpCounter(0)

        def put(self, *args, **kwargs):
                # If the put fails, the exception will prevent us from incrementing qsizeCounter
                Queue.put(self, *args, **kwargs)
                self.qsizeCounter.add(1)

        def get(self, *args, **kwargs):
                # If the get fails, the exception will prevent us from decrementing qsizeCounter
                val = Queue.get(self, *args, **kwargs)
                self.qsizeCounter.add(-1)
                return val

        def qsize(self):
                return self.qsizeCounter.value
