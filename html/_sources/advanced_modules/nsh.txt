==========
nsh Module
==========

The nsh module allows direct access to the various serial ports on a
Pixhawk, for both reading and writing data. In addition, the NuttX
serial console can be accessed - negating the requirement for a
specialised debugging cable.

.. note:: 

    The nsh module can only talk to one serial port at a time.

Once loaded with ``module load nsh``, The settings of the module can be
shown and changed by entering ``nsh set port X`` or
``nsh set baudrate Y``, where X is the desired port and Y the desired
baudrate.

Then use ``nsh start`` and ``nsh stop`` to start and stop the serial
port communication.

