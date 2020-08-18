#!/usr/bin/env python
'''
support for a GCS attached DGPS system
Set the ip, port and conntype settings to your requiered connection
The module supports TCP and UDP connections (conntype)

For using with Emlid Reach use TCP connection, set the IP to the IP address of the Reach base

Configure ReachView like so:

Base mode:
TCP, Role: Server, address: localhost, Port: 9000
Set the RTCM Messages to 10Hz and enable any that are needed.
'''

import socket, errno, threading
import time
from MAVProxy.modules.lib import rtcm3
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class DGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DGPSModule, self).__init__(mpstate, "DGPS", "DGPS injection support for SBP/RTCP/UBC over tcp/udp")
        print ("Loading DGPS module")
        self.dgps_settings = mp_settings.MPSettings(
            [('ip', str, "127.0.0.1"),
             ('port', int, 13320),
             ('conntype', str, 'UDP'),
             ('silentFail', bool, True),
             ('logfile', str, None),
             ('connectionTimeout', int, 10)
            ])
        self.add_command('dgps', self.cmd_dgps, 'DGPS control',
                         ["<status>",
                          "<start>",
                          "<stop>",
                          "set (DGPSSETTING)"])
        self.add_completion_function('(DGPSSETTING)',
                                     self.dgps_settings.completion)

        self.port = None    
        self.waiting = False  
        self.lastConnAttempt = None
        self.last_pkt = time.time()
        self.inject_seq_nr = 0
        self.pkt_count = 0
        self.last_rate = None
        self.rate_total = 0
        self.rate = 0
        # RTCM3 parser
        self.rtcm3 = rtcm3.RTCM3()
        self.last_id = None
        self.id_counts = {}
        self.last_by_id = {}

    def log_rtcm(self, data):
        '''optionally log rtcm data'''
        if self.dgps_settings.logfile is None:
            return
        if self.logfile is None:
            self.logfile = open(self.dgps_settings.logfile, 'wb')
        if self.logfile is not None:
            self.logfile.write(data)

    def cmd_dgps(self, args):
        '''dgps command handling'''
        if len(args) <= 0:
            print("Usage: dgps <start|stop|status|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.cmd_stop()
        elif args[0] == "status":
            self.dgps_status()
        elif args[0] == "set":
            self.dgps_settings.command(args[1:])

    def cmd_stop(self):
        self.waiting = False
        self.port = None

    def cmd_start(self):
        '''start dgps link'''
        if self.dgps_settings.conntype == "UDP":
            print("Connecting to UDP RTCM Stream")
            self.connect_udp_rtcm_base(self.dgps_settings.ip, self.dgps_settings.port)
        elif self.dgps_settings.conntype == "TCP":
            print("Connecting to TCP RTCM Stream")
            self.connect_tcp_rtcm_base(self.dgps_settings.ip, self.dgps_settings.port)
        else:
            print("Undefined connection type!")
            return    

        self.last_rate = time.time()       
        self.rate_total = 0

    def dgps_status(self):
        '''show dgps status'''
        now = time.time()
        print("\nDGPS Configuration:")
        print("Connection Type: %s" % self.dgps_settings.conntype)
        print("IP Address: %s" % self.dgps_settings.ip)
        print("Port: %s" % self.dgps_settings.port)
        
        if self.waiting is True:
            print("Connection Error attempting to reconnect")
            return

        if self.port is None:
            print("DGPS: Not started")
            return
        elif self.last_pkt is None:
            print("DGPS: no data")
            return
        
        frame_size = 0
        for id in sorted(self.id_counts.keys()):
            print(" %4u: %u (len %u)" % (id, self.id_counts[id], len(self.last_by_id[id])))
            frame_size += len(self.last_by_id[id])
        print("dgps: %u packets, %.2f bytes/sec last %.3fs ago framesize %u" % (self.pkt_count, self.rate, now - self.last_pkt, frame_size))

    def connect_udp_rtcm_base(self, ip, port):
        print ("UDP Connection")
        self.portnum = 13320
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind(("127.0.0.1", self.portnum))
        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        print("DGPS: Listening for RTCM packets on UDP://%s:%s" % ("127.0.0.1", self.portnum))

    def connect_tcp_rtcm_base(self, ip, port):
        if self.waiting is False:
            print ("TCP Connection")
        self.waiting = True
        self.rtcm3.reset()
        try:
            self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.port.settimeout(1)
            self.port.connect((ip, port))
        except:
            if self.dgps_settings.silentFail is False:
                print ("ERROR: Failed to connect to RTCM base over TCP, retrying in 2.5s")
            self.lastConnAttempt = time.time()
            return

        # This prevents the TCP connection from blocking other functions in the system. 
        # If this is set before attempting the connection (.connect) the connection is
        # never established. However if this is not set before that attempt then MAVproxy 
        # is blocked for the timeout period, in this case 1 sec.
        self.port.setblocking(0)

        print ("Connected to base using TCP")
        self.waiting = False
        self.last_pkt = time.time()

    def idle_task(self):
        '''called in idle time'''
        # Dont do anything if no recieve port is set
        if self.port is None:
            return

        # Check to see if we have not recieved packets in a long time and try to reconnect
        now = time.time()
        if self.waiting is False and now - self.last_pkt > self.dgps_settings.connectionTimeout:
            print("Lost Packets attempting reconnection")
            self.cmd_stop()
            self.cmd_start()

        # Try to reconnect in case the connection attempt failed. 
        if self.waiting is True and self.dgps_settings.conntype == "TCP":
            if (time.time() - self.lastConnAttempt) > 2.5:
                if self.dgps_settings.silentFail is False:
                    print("Attempting to connect")
                self.connect_tcp_rtcm_base(self.dgps_settings.ip, self.dgps_settings.port)
            return

        # Read data from socket
        try:
            data = self.port.recv(1024) # Attempt to read up to 1024 bytes.
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise

        # Parse and send RTCM3 data
        for element in data:  # RTCM3 Library expects to read one byte at a time
            if self.rtcm3.read(element):    # Data sent to rtcm library, it returns data once a full packet is recieved
                self.last_id = self.rtcm3.get_packet_ID()
                rtcm3Data = self.rtcm3.get_packet()
                if self.rtcm3.get_packet() is None:
                    print ("No packet")
                    return

                self.log_rtcm(rtcm3Data)    # Log the packet
                
                # Store statistics for each RTCM Packet ID
                if not self.last_id in self.id_counts:
                    self.id_counts[self.last_id] = 0
                    self.last_by_id[self.last_id] = rtcm3Data[:]
                self.id_counts[self.last_id] += 1     

                # Prepare to send the RTCM3 Data through MAVLink
                blen = len(rtcm3Data)
                if blen > 4*180: # can't send this with GPS_RTCM_DATA (wont fit even on fragmented packets?)
                    if self.dgps_settings.silentFail is False:
                        print("Error, cannot send data with GPS_RTCM_DATA")
                    return

                self.rate_total += blen

                # Check if data needs to be fragmented
                if blen > 180:
                    flags = 1 # fragmented
                else:
                    flags = 0

                # add in the sequence number
                flags |= (self.pkt_count & 0x1F) << 3

                # Send data through MAVLink
                fragment = 0
                while blen > 0:
                    send_data = bytearray(rtcm3Data[:180])
                    frag_len = len(send_data)
                    data = rtcm3Data[frag_len:]
                    if frag_len < 180:
                        send_data.extend(bytearray([0]*(180-frag_len)))
                    self.master.mav.gps_rtcm_data_send(flags | (fragment<<1), frag_len, send_data)
                    fragment += 1
                    blen -= frag_len
                self.pkt_count += 1

                # Store time of last packet sent
                now = time.time()
                self.last_pkt = now

        # Calculate the rate at which we are reciving data for stats
        now = time.time()
        if now - self.last_rate > 1:
            dt = now - self.last_rate
            rate_now = self.rate_total / float(dt)
            self.rate = 0.9 * self.rate + 0.1 * rate_now
            self.last_rate = now
            self.rate_total = 0

def init(mpstate):
    '''initialise module'''
    return DGPSModule(mpstate)

