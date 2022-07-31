#!/usr/bin/env python
'''
a server to serve wx unique IDs between processes. This allows for menus created in child processes
to be passed across pipes to another process
'''

from MAVProxy.modules.lib.wx_loader import wx
from MAVProxy.modules.lib import multiproc
import time

idqueue = None
idserver = None

def id_server():
    while True:
        # keep the queue stocked with 100 IDs
        while idqueue.qsize() < 100:
            Id = int(wx.NewId())
            print("Added: ", Id)
            idqueue.put(Id)
        time.sleep(0.1)

def start():
    global idserver, idqueue
    if idserver is None:
        idqueue = multiproc.Queue()
        idserver = multiproc.Process(target=id_server)
        idserver.start()

def NewId():
    print("getting ID")
    global idqueue
    if idqueue is None:
        raise Exception("IDserver not started")
    Id = idqueue.get()
    print("got: ", Id)
    return Id

if __name__ == "__main__":
    multiproc.freeze_support()

    def test_child(i):
        print("Starting %u" % i)
        while True:
            Id = NewId()
            print('got: ', Id)

    for i in range(5):
        child = multiproc.Process(target=test_child, args=(i,))
        child.start()
    time.sleep(30)
