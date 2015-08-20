============
Flight Modes
============

Flight modes can be directly entered on the command line. This will send
the appropriate command to the APM. Use ``mode n`` to change mode, where
n is the desired mode. To get a list of available modes, enter ``mode``.

mode auto
=========

Enter AUTO mode, starting with the first waypoint.

mode loiter
===========

Enter LOITER mode around the UAV's current position.

mode rtl
========

Enter Return to Launch mode.

mode manual
===========

Enter manual mode. On Arducopter this is equivalent to STABILIZE mode

mode fbwa
=========

Enter Fly By Wire A mode. Applicable on fixed wing APM's only.

land
====

Enter LAND mode. Note this is only applicable on fixed wing APM's.

A specific mode is guided. By entering ``guided alt`` (where alt is the
desired altitude), the user can the select a point on the map window and
the UAV will immediately begin flying towards that point.

Arming/Disarming
----------------

The APM can be armed and disarmed at any time by using the
``arm throttle`` and ``disarm`` commands.

The pre-arming checks can be run (without actually arming the UAV) via
``arm check``. A full list of these checks is given by ``arm list``. To
enable/disable a particular arming check, use ``arm check n`` and
``arm uncheck n`` respectively, where n is the name of the check.

The above commands are contained within the ``mode`` and ``arm`` modules
respectively, which are loaded by default at MAVProxy startup

