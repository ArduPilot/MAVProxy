'''
MAVProxy sailing dashboard utility classes
'''

import enum

class WindReference(enum.Enum):
    '''
    An enumeration for the wind reference (either RELATIVE or TRUE)
    '''

    RELATIVE = 1
    TRUE = 2

class SpeedUnit(enum.Enum):
    '''
    An enumeration for the units of speed: kph, mph, or knots
    '''

    KPH = 1
    MPH = 2
    KNOTS = 3

class WindAngleAndSpeed(object):
    '''
    Wind angle and speed attributes
    '''

    def __init__(self, reference, angle, speed, unit):
        self.wind_reference = reference
        self.wind_angle = angle
        self.wind_speed = speed
        self.wind_speed_unit = unit

class WaterSpeedAndHeading(object):
    '''
    Water speed and heading attributes
    '''

    def __init__(self, water_speed, heading_true):
        self.water_speed = water_speed
        self.heading_true = heading_true
