#!/usr/bin/env python
'''
A PPP over MAVLink module
Andrew Tridgell
May 2012
'''

import time, os, fcntl, pty

from MAVProxy.modules.lib import mp_module

class PPPModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(PPPModule, self).__init__(mpstate, "ppp", "PPP link")
        self.command = "noauth nodefaultroute nodetach nodeflate nobsdcomp mtu 128".split()
        self.packet_count = 0
        self.byte_count = 0
        self.ppp_fd = -1
        self.pid = -1
        self.add_command('ppp', self.cmd_ppp, "ppp link control")


    def ppp_read(self, ppp_fd):
        '''called from main select loop in mavproxy when the pppd child
        sends us some data'''
        buf = os.read(ppp_fd, 100)
        if len(buf) == 0:
            # EOF on the child fd
            self.stop_ppp_link()
            return
        print("ppp packet len=%u" % len(buf))
        master = self.master
        master.mav.ppp_send(len(buf), buf)

    def start_ppp_link(self):
        '''startup the link'''
        cmd = ['pppd']
        cmd.extend(self.command)
        (self.pid, self.ppp_fd) = pty.fork()
        if self.pid == 0:
            os.execvp("pppd", cmd)
            raise RuntimeError("pppd exited")
        if self.ppp_fd == -1:
            print("Failed to create link fd")
            return

        # ensure fd is non-blocking
        fcntl.fcntl(self.ppp_fd, fcntl.F_SETFL, fcntl.fcntl(self.ppp_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        self.byte_count = 0
        self.packet_count = 0

        # ask mavproxy to add us to the select loop
        self.mpself.select_extra[self.ppp_fd] = (self.ppp_read, self.ppp_fd)


    def stop_ppp_link(self):
        '''stop the link'''
        if self.ppp_fd == -1:
            return
        try:
            self.mpself.select_extra.pop(self.ppp_fd)
            os.close(self.ppp_fd)
            os.waitpid(self.pid, 0)
        except Exception:
            pass
        self.pid = -1
        self.ppp_fd = -1
        print("stopped ppp link")


    def cmd_ppp(self, args):
        '''set ppp parameters and start link'''
        usage = "ppp <command|start|stop>"
        if len(args) == 0:
            print(usage)
            return
        if args[0] == "command":
            if len(args) == 1:
                print("ppp.command=%s" % " ".join(self.command))
            else:
                self.command = args[1:]
        elif args[0] == "start":
            self.start_ppp_link()
        elif args[0] == "stop":
            self.stop_ppp_link()
        elif args[0] == "status":
            self.console.writeln("%u packets %u bytes" % (self.packet_count, self.byte_count))

    def unload(self):
        '''unload module'''
        self.stop_ppp_link()

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'PPP' and self.ppp_fd != -1:
            print("got ppp mavlink pkt len=%u" % m.length)
            os.write(self.ppp_fd, m.data[:m.length])

def init(mpstate):
    '''initialise module'''
    return PPPModule(mpstate)
