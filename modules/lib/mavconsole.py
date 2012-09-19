
class DelegateMavConsole():
    '''delegate all printing to an existing console'''
    def __init__(self, delegateConsole):
        self._console = delegateConsole
        self._show = False

    def onMessage(self,msg):
        if msg.type == 254 and self._show:
            payload = msg.data[0:msg.len]
            self._console.write(str(payload))
   
    def quiet(self):
        self._show = False

    def active(self) :
        self._show = True

    def write(self, master, payload):
        if len(payload) < 16:
            msg = master.mav.data16_send(254, len(payload), payload)
        elif len(payload) < 32:
            msg = master.mav.data32_send(254, len(payload), payload)
        else:
            print "Error: mavconsole::write placeholder for payloads greater than 32 bytes"


