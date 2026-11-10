"""
MAVProxy terrain navigation module
"""

from MAVProxy.mavproxy import MPState
from MAVProxy.modules.lib.mp_module import MPModule
from MAVProxy.modules.mavproxy_terrainnav.terrainnav import TerrainNavModule


def init(mpstate: MPState) -> MPModule:
    """
    Initialise module
    """
    return TerrainNavModule(mpstate)
