'''
multi-processing abstraction
This wraps the multiprocessing module, using billiard on MacOS
and multiprocessing on Linux and Windows
'''

import platform, os
if platform.system() == 'Darwin' or os.environ.get('USE_BILLIARD',None) is not None:
    from billiard import Process, forking_enable, freeze_support, Pipe, Semaphore
    forking_enable(False)
else:
    from multiprocessing import Process, freeze_support, Pipe, Semaphore

class Queue(object):
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
        while self.receiver.poll():
            try:
                m = self.receiver.recv()
                self.pending.append(m)
            except Exception:
                self.close()
                break

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
