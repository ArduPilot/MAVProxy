# missions for checking the map drawing

Missions built to put every kind of item the maps draw specially in front
of the drawing code at once, so a SITL flight can be compared against what
was drawn: the live 2D and 3D maps while it flies, and mavflightview or
MAVExplorer's `map3d` over the dataflash log afterwards.

Each item carries a comment saying what it is there for.  They are laid out
around SITL's default CMAC home, `-35.363261 149.165230`, and every one of
them can be flown by the vehicle it is named for.

    wp load tests/missions/plane-mission-geometry.txt

## plane-mission-geometry.txt

Sixteen items covering the loiter drawing:

- a loiter to altitude climbing 400m over a short approach, drawn as a
  spiral of about two turns, and one descending 200m, drawn as about three
- three turns counter-clockwise and two clockwise, close enough together
  that the leg between them is the tangent which crosses between the two
  circles, and a loiter for time after them turning the same way, whose leg
  is the tangent which does not cross
- a loiter with no radius of its own, which takes `WP_LOITER_RAD`
- param4 both ways: 0 leaves the next leg crosstracked from the loiter
  centre, so the drawing pulls the vehicle back onto that track after it
  leaves the circle; 1 crosstracks from the exit, and no pull-back is drawn
- AMSL, home-relative and terrain-frame altitudes, and five changes of
  frame between one item and the next
- a loiter forever at the end.  The mission parks there: switch out of AUTO
  once there is nothing left to watch

ArduPlane has no `NAV_ARC_WAYPOINT`, so the arc waypoints are in the copter
mission instead.  Nothing here flies `MAV_CMD_DO_ORBIT` either: the maps
draw its circle, but no ArduPilot vehicle implements the command, so it
cannot go in a mission which is meant to be flown.

## copter-mission-geometry.txt

Thirteen items over a few hundred metres, covering what a multirotor draws
differently:

- arc waypoints, one sweeping 90 degrees clockwise and one 270
  counter-clockwise.  The second bulges a long way outside its chord, which
  is what the 2D map has to bound it over to keep it on screen
- loiter turns both ways round, near enough that the leg between them is
  the tangent which crosses between the two circles
- the loiters a multirotor holds position for instead of circling --
  unlimited, for time, and to altitude -- which are drawn as the points
  they are even where the item carries a radius
- turns with no radius of its own: a multirotor has no `WP_LOITER_RAD` to
  fall back on, so nothing is drawn (the vehicle flies its
  `CIRCLE_RADIUS_M`, or `CIRCLE_RADIUS` in centimetres on older firmware)
- a takeoff carrying no position, drawn as the climb from where it was
  flown
- a multirotor has no L1 controller, so nothing is drawn pulling it back
  onto a track out of a loiter centre

As with the plane mission, it parks at a loiter forever at the end.
