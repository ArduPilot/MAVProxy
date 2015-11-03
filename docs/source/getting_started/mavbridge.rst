====================
MAVProxy Wifi Bridge
====================

MAVProxy can be used to create a network bridge between a telemetry radio and Wifi or wired network.

.. figure:: bridge.png

This is useful in the circumstances when a telemetry stream is required to be distributed to a group of clients on a network or the GCS client is not able to be directly connected to the telemetry radio (ie. a tablet running the GCS software does not have an available USB port to connect to a USB telemetry radio).


Requirements
============

Raspberry Pi or similar with compatible Wifi dongle (if streaming telemetry to Wifi). Any similar small computer would work. The onyl requirement is that it is able to run the Ubuntu operating system.


Setting up the Pi
=================

The instructions for setting up the Wifi Access Point can be found here: http://dev.ardupilot.com/wiki/making-a-mavlink-wifi-bridge-using-the-raspberry-pi/. Follow steps 1-3.

Setting up MAVProxy
===================

Follow the download and installation instructions :doc:`here <./download_and_installation>`.

Setting up the Daemon
=====================

For this section, a terminal on the Pi is required.

Make a new file for the daemon, based on a skeleton file:

.. code:: bash

    sudo cp /etc/init.d/skeleton /etc/init.d/mavgateway

Edit /etc/init.d/mavgateway:

.. code:: bash
    
    sudo nano /etc/init.d/mavgateway

And edit these lines:

.. code:: bash

    DAEMON_ARGS="--master=/dev/ttyAMA0,57600 --out=udpin:0.0.0.0:14550 --daemon"
    NAME=mavproxy.py
    DESC="Mavproxy based mavlink to wifi gateway"
    Provides: mavgateway
    Short-Description: Mavlink to UDP gateway service
    DAEMON=/usr/local/bin/$NAME

Use CTRL-x to exit the editor

.. note::

    In the DAEMON_ARGS line above, edit the ttyAMA0 port and 57600 baudrate to match the name and baudrate of the telemetry radio connected to the Pi.

Edit the permissions for mavgateway and update the system to include the mavgateway daemon upon startup:

.. code:: bash

    sudo chown root:root /etc/init.d/mavgateway
    sudo update-rc.d mavgateway defaults

Running
=======

The Pi bridge will automatically start up when the Pi is booted.

On the GCS client, first connect to the Pi's network, then connect (via UDP) to 192.168.42.1, port 14550. For example, on a MAVProxy client:

.. code:: bash

    mavproxy.py –master=192.168.42.1:14550


