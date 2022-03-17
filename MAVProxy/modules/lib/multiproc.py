'''
multi-processing abstraction
This wraps the multiprocessing module, using billiard on MacOS
and multiprocessing on Linux and Windows

The key problem on MacOS is that you can't fork in any process that uses
threading, which is almost all of processes as so many libraries use
threads. So instead billiard uses an approach that uses fork+exec and re-runs
the script in the child. It is horrible, but it seems to be the only way to
make things work on MacOS
'''

class PipeQueue(object):
    '''simulate a queue using a pipe. This is used to avoid a problem with
    pipes on MacOS, while still keeping similar syntax'''
    def __init__(self):
        (self.sender,self.receiver) = Pipe()
        self.alive = True
        self.pending = []

    def close(self):
        self.alive = False
        self.sender.close()
        self.receiver.close()

    def put(self, *args):
        if not self.alive:
            return
        try:
            self.sender.send(*args)
        except Exception:
            self.close()

    def fill(self):
        if not self.alive:
            return
        try:
            while self.receiver.poll():
                m = self.receiver.recv()
                self.pending.append(m)
        except Exception:
            self.alive = False
            self.close()

    def get(self):
        if not self.alive:
            return None
        self.fill()
        if len(self.pending) > 0:
            return self.pending.pop(0)
        return None

    def qsize(self):
        self.fill()
        return len(self.pending)

    def empty(self):
        return self.qsize() == 0

import platform, os, sys

# we use billiard (and forking disable) on MacOS, and also if USE_BILLIARD environment
# is set. Using USE_BILLIARD allows for debugging of the crazy forking disable approach on
# a saner platform
# As of Python 3.8 the default start method for macOS is spawn and billiard is not required.
if ((platform.system() == 'Darwin' or os.environ.get('USE_BILLIARD',None) is not None)
    and sys.version_info < (3, 8)):
    from billiard import Process, forking_enable, freeze_support, Pipe, Semaphore, Event, Lock
    forking_enable(False)
    Queue = PipeQueue
else:
    from multiprocessing import Process, freeze_support, Pipe, Semaphore, Event, Lock, Queue
