==========
Simulation
==========

MAVProxy can be used in conjunction with the APM source code to create a
simulation environment. There are two types of simulation environments:

- Hardware In the Loop (HIL) - The APM code is loaded onto a physical
  board and a simulator on a connected PC provides sensor inputs.
- Software In The Loop (SITL) - The APM code is built and run on a PC.
  A simulator provides sensor inputs.

Linux - SITL
============

Download the `APM source
code <https://github.com/diydrones/ardupilot>`_ and the
`jsbsim <https://github.com/tridge/jsbsim>`_ simulator. Depending on
your system setup, other packages may be required

A full guide for setting up the environment is at the `APM
Wiki <http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/>`_.

Some useful command line options are:

-  -v VEHICLE vehicle type (ArduPlane, ArduCopter or APMrover2)
-  -L select start location from Tools/autotest/locations.txt
-  -w wipe EEPROM and reload parameters
-  -j NUM\_PROC number of processors to use during build (default 1)
-  -c do a make clean before building

Any arguments to be sent to MAVProxy can simply be specified at the end
of the command:

.. code:: bash

    sim_vehicle.sh -v Arduplane -j 4 [mavproxy_options]

Note that the correct parameters for the vehicles in the simulator are
required to be loaded. The parameters are in the Rover.parm,
Arducopter.parm, ArduPlane.parm for the relevant vehicles.

Caveats
=======

The are several important notes in regards to using the simulator:

-  The simulator will (by default) start at a small model aircraft field
   in Canberra, Australia. Any waypoints should be located relative to
   this.
-  The multicopter and rover simulators are fairly basic and should not
   be relied on for accurate simulations.
-  Due to limitations in JSBsim's ground handling model, the aircraft
   may sometimes flip over whilst sitting on the runway.

