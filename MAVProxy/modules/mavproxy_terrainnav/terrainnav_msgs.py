"""
Terrain Navigation messages
"""

from dataclasses import dataclass


@dataclass
class SetStart:
    pass


@dataclass
class SetGoal:
    pass


@dataclass
class AddRally:
    pass


@dataclass
class AddWaypoint:
    pass


@dataclass
class RunPlanner:
    pass


@dataclass
class GenWaypoints:
    pass


@dataclass
class Hold:
    pass


@dataclass
class Navigate:
    pass


@dataclass
class Rollout:
    pass


@dataclass
class Abort:
    pass


@dataclass
class Return:
    pass


@dataclass
class ShowContours:
    pass


@dataclass
class HideContours:
    pass


@dataclass
class ShowBoundary:
    pass


@dataclass
class HideBoundary:
    pass


@dataclass
class MoveBoundary:
    pass


@dataclass
class ClearPath:
    pass


@dataclass
class ClearWaypoints:
    pass


@dataclass
class Settings:
    pass
