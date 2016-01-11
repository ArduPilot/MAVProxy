=====================================
Frequently Asked Questions and Issues
=====================================

#. I have random "noise" displaying on the console, showing as corrupted text. How do I remove it?

    MAVProxy will, by default, show corrupted packets. To disable showing them, use ``set shownoise false`` in the MAVProxy console.



#. How do I get MAVProxy to execute specific commands on startup?

    Put the commands in mavinit.scr as per :doc:`./mavinit`


#. Which Flight controllers are compatible with MAVProxy?

    The APM branches (`APM:Plane <http://plane.ardupilot.com/>`_, `APM:Copter <http://copter.ardupilot.com/>`_, `APM:Rover <http://rover.ardupilot.com/>`_) are fully compatible. Other Mavlink-based flight controllers (such as the `PX4 <http://px4.io/>`_ stack) should be compatible, but may show all flight data.



