class Attitude():
    '''The current Attitude Data'''
    def __init__(self, attitudeMsg):
        self.pitch = attitudeMsg.pitch
        self.roll = attitudeMsg.roll
        self.yaw = attitudeMsg.yaw

class VFR_HUD():
    '''HUD Information.'''
    def __init__(self,hudMsg):
        self.airspeed = hudMsg.airspeed
        self.groundspeed = hudMsg.groundspeed
        self.heading = hudMsg.heading
        self.throttle = hudMsg.throttle
        self.climbRate = hudMsg.climb
        
class Global_Position_INT():
    '''Altitude relative to ground (GPS).'''
    def __init__(self,gpsINT):
        self.relAlt = gpsINT.relative_alt/1000.0
        
class BatteryInfo():
    '''Voltage, current and remaning battery.'''
    def __init__(self,batMsg):
        self.voltage = batMsg.voltage_battery/1000.0 # Volts
        self.current = batMsg.current_battery/100.0 # Amps
        self.batRemain = batMsg.battery_remaining # %