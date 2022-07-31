#!/usr/bin/env python
'''
a server to serve wx unique IDs between processes. This allows for menus created in child processes
to be passed across pipes to another process
'''

from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import multiproc
import time

idqueue = multiproc.Queue()

def id_server():
    while True:
        # keep the queue stocked with 100 IDs
        while idqueue.qsize() < 100:
            Id = int(wx.NewId())
            idqueue.put(Id)
        time.sleep(0.1)

idserver = multiproc.Process(target=id_server)
idserver.start()

def NewId():
    return idqueue.get()

if __name__ == "__main__":
    def test_child(i):
        print("Starting %u" % i)
        while True:
            Id = NewId()
            print('got: ', Id)

    for i in range(5):
        child = multiproc.Process(target=test_child, args=(i,))
        child.start()
    time.sleep(30)
