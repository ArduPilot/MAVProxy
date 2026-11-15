"""
MAVProxy planner module using Shearwater's SmartFLight
"""

from MAVProxy.mavproxy import MPState
from MAVProxy.modules.lib.mp_module import MPModule
from MAVProxy.modules.mavproxy_smartflight.smart_flight_planner import SmartFlightPlannerModule


def init(mpstate: MPState) -> MPModule:
    """
    Initialise module
    """
    return SmartFlightPlannerModule(mpstate)
