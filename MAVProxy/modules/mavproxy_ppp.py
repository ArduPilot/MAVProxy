#!/usr/bin/env python
'''
A PPP over MAVLink module
Andrew Tridgell
May 2012
'''

import time, os, fcntl, pty

mpstate = None

class module_state(object):
    def __init__(self):
        self.command = "noauth nodefaultroute nodetach nodeflate nobsdcomp mtu 128".split()
        self.packet_count = 0
        self.byte_count = 0
        self.ppp_fd = -1
        self.pid = -1

def name():
    '''return module name'''
    return "ppp"

def description():
    '''return module description'''
    return "PPP link"

def ppp_read(ppp_fd):
    '''called from main select loop in mavproxy when the pppd child
    sends us some data'''
    state = mpstate.ppp_state
    buf = os.read(ppp_fd, 100)
    if len(buf) == 0:
        # EOF on the child fd
        stop_ppp_link()
        return
    print("ppp packet len=%u" % len(buf))
    master = mpstate.master()
    master.mav.ppp_send(len(buf), buf)

def start_ppp_link():
    '''startup the link'''
    state = mpstate.ppp_state
    cmd = ['pppd']
    cmd.extend(state.command)
    (state.pid, state.ppp_fd) = pty.fork()
    if state.pid == 0:
        os.execvp("pppd", cmd)
        raise RuntimeError("pppd exited")
    if state.ppp_fd == -1:
        print("Failed to create link fd")
        return

    # ensure fd is non-blocking
    fcntl.fcntl(state.ppp_fd, fcntl.F_SETFL, fcntl.fcntl(state.ppp_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
    state.byte_count = 0
    state.packet_count = 0

    # ask mavproxy to add us to the select loop
    mpstate.select_extra[state.ppp_fd] = (ppp_read, state.ppp_fd)

    
def stop_ppp_link():
    '''stop the link'''
    state = mpstate.ppp_state
    if state.ppp_fd == -1:
        return
    try:
        mpstate.select_extra.pop(state.ppp_fd)
        os.close(state.ppp_fd)
        os.waitpid(state.pid, 0)
    except Exception:
        pass
    state.pid = -1
    state.ppp_fd = -1
    print("stopped ppp link")


def cmd_ppp(args):
    '''set ppp parameters and start link'''
    state = mpstate.ppp_state
    usage = "ppp <command|start|stop>"
    if len(args) == 0:
        print usage
        return
    if args[0] == "command":
        if len(args) == 1:
            print("ppp.command=%s" % " ".join(state.command))
        else:
            state.command = args[1:]
    elif args[0] == "start":
        start_ppp_link()
    elif args[0] == "stop":
        stop_ppp_link()
    elif args[0] == "status":
        mpstate.console.writeln("%u packets %u bytes" % (state.packet_count, state.byte_count))
        
def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.ppp_state = module_state()
    mpstate.command_map['ppp'] = (cmd_ppp, "ppp link control")

def unload():
    '''unload module'''
    stop_ppp_link()

def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.ppp_state
    if m.get_type() == 'PPP' and state.ppp_fd != -1:
        print("got ppp mavlink pkt len=%u" % m.length)
        os.write(state.ppp_fd, m.data[:m.length])
