==========
Quickstart
==========

In it's simplest form, MAVProxy only needs the address of the USB port
or network address to connect to.

Over USB
========

If there is only 1 APM connected, the ``--master`` is not required.
MAVProxy will autodetect the correct port.

Linux:

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0

Windows:

.. code:: bash

    mavproxy.py --master="com14"

Normally MAVProxy will auto-detect the correct baudrate. If required,
the baud rate can instead be manually specified.

Linux:

.. code:: bash

    mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

Windows:

.. code:: bash

    mavproxy.py --master="com14" --baudrate=57600

Over Network
============

Specify the IP address and port containing a mavlink stream. The address
to connect to must be your own IP address.

.. code:: bash

    mavproxy.py --master=192.168.1.1:14550

