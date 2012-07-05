#!/usr/bin/env python
'''
netconsole  - a network interface for operating MAVProxy
(c) David (Buzz) Bussenschutt and Andrew Tridgell
Released under the same license as MAVProxy ( GNU GPLv3 )

recommended usage to autoload thos module on startup is:
mayproxy.py ZZZZZZZZZZZZZZZZZZZZZZZ --aircraft=XXXXX
 mkdir XXXXX
echo 'module load netconsole' > XXXXX/mavinit.scr
'''

import socket, errno, sys, time, pickle

mpstate = None

def name():
    '''return module name'''
    return "netconsole"

def description():
    '''return module description'''
    return "tcpip based control"
    
# pretend to be a buffer with nothing in it, so as to block real STDIN on console! 
class null_in(object):
    def __init__(self):
        pass    
    def readline(buf):
        time.sleep(100000)
        return " "
        pass

class netconsole_state(object):
    def __init__(self):    
        # "control port" details, as used by optional mavproxy GUI. 
        self.TCP_HOST="" # empty string = localhost
        self.TCP_PORT=45678
        
        # setup input from optional GUI
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.TCP_HOST, self.TCP_PORT))
        self.sock.listen(1)  # server mode
        self.sock.setblocking(0) # non-blocking mode
        self.connection = None;    # single connection at a time allowed, it is stored here:
        self.stdout_saved = None
        self.stdin_saved = None


# server socket callback for listen() - when new client connects.
def ip_listen(sock):
    '''called from main select loop in mavproxy when the child sends us some data'''
    state = mpstate.netconsole_state

    if state.connection is not None:
        state.connection.close()

    state.connection, state.client_address = sock.accept()
    print("new TCP connection from %s" % str(state.client_address))
    state.connection.setblocking(0)
    mpstate.select_extra[state.connection.fileno()] = (ip_data, state.connection)

    # takeover stdin/stdout while we are connected
    state.stdout_saved = sys.stdout
    state.stdin_saved = sys.stdin
    sys.stdout = state.connection.makefile(mode='w', bufsize=0)
    sys.stdin = null_in() #state.connection.makefile(mode='r', bufsize=0)

# server socket callback for accept()  - when an existing client sends data
def ip_data(sock):
    '''read some data from the socket'''
    state = mpstate.netconsole_state

    data = sock.recv(1024)
    if len(data) == 0:
        print("closed connection from %s" % str(state.client_address))
       
        # we have reached EOF on the socket
        sock.close()
        sys.stdout = state.stdout_saved
        sys.stdin = state.stdin_saved
        state.stdout_saved = None
        state.stdin_saved = None
        state.connection = None
        # this exception causes the socket to be removed from
        # the select loop in mavproxy
        raise RuntimeError("got EOF")

    mpstate.functions.process_stdin(data)


def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate # push it to the global var for everyone
    mpstate.netconsole_state = netconsole_state()
    state = mpstate.netconsole_state

    print("Module netconsole loaded")
    
    # ask mavproxy to add us to the select loop
    # pass it function name and its arguments to be called 
    mpstate.select_extra[state.sock.fileno()] = (ip_listen, state.sock)

    
def unload():
    '''unload the module'''
    state = mpstate.netconsole_state
    if state.sock is not None:
        state.sock.close()
    if state.connection is not None:
        state.connection.close()
    if state.stdout_saved is not None:
        sys.stdout = state.stdout_saved
        sys.stdin = state.stdin_saved


def mavlink_packet(msg):
    '''handle an incoming mavlink packet'''
    
    type = msg.get_type()
    
    # since we capture sys.stdout as the outbound channel and pass it to a socket, outbound coms is like htis:
    #if type == 'HEARTBEAT':
       # print pickle.dump(msg,sys.stdout)

    

    pass
