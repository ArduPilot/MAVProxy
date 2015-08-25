=========================
Download and Installation
=========================


Windows
=======

A complete windows installer for MAVProxy is available at
http://firmware.diydrones.com/Tools/MAVProxy/.

Please note that the "missionedit" module does not currently run on the
Windows package of MAVProxy. This is being investigated and will
hopefully be fixed soon.


Linux
=====

Linux users can use the PyPi program to get the needed packages:

.. code:: bash

    sudo apt-get install python-pip``

Then download and install MAVProxy. Prerequisites will be
automatically downloaded too. Note a sudo may be required in some
circumstances if the install generates errors.

.. code:: bash

    pip install MAVProxy

The following other packages may also be required:

.. code:: bash

    sudo apt-get install python-opencv python-wxgtk

.. note::

    On some Linux systems, ``python-wxgtk`` may be instead named
    as ``python-wxgtk2.8``.

Mac
===

Some users have reported difficulties with installing wxPython on the
latest version of OSX. These issues have not yet been resolved, so you
may have mixed results trying to get MAVProxy up and running.

OSX should require the same prerequisites as for the Windows
installations.
