**********
Moving Map
**********

.. code:: bash

    module load map
    
A moving map display that shows the UAV's current position, waypoints
and geofence.

Maps are automatically downloaded and cached to the user's hard drive.
The module will use these files if an internet connection is not found.

To display the waypoints and geofence, the ``wp list`` and
``fence list`` commands can be used.

Waypoint editing is allowing by right-clicking to select waypoint and
then right clicking to move it to that point.

To draw a set of waypoints, use the ``wp draw`` command. Right click on
the map for the desired waypoints. When finished, use the ``wp loop`` to
connect the last waypoint to the first one, creating a loop.

Use the "g" key to specify a position to move the map to.

.. image:: map.png



