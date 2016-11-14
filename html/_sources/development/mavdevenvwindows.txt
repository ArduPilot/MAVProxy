Windows Development Environment
===============================

In Windows, the process is a bit more complicated than for Linux.

You will need to have `Python
2.7 <http://www.python.org/download/releases/2.7/>`_,
`wxPython <http://www.wxpython.org/download.php>`_ and
`pygame <http://pygame.org/download.shtml>`_ installed first.

Next, open up a console in the Python scripts install path
(:file:`C:\\Python27\\Scripts` or similar). Use ``pip install [filepath]`` to install them

- `numPY <http://www.lfd.uci.edu/~gohlke/pythonlibs/#numpy>`_
- `Pillow (replaces
  PIL) <http://www.lfd.uci.edu/~gohlke/pythonlibs/#pillow>`_
- `OpenCV <http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv>`_

.. code:: bash

    cd C:\Python27\Scripts
    pip install C:\Users\Stephen\Desktop\numpy.whl

Use ``pip`` to install the other packages. Note that some packages 
require specific older versions to install:

.. code:: bash

    cd C:\Python27\Scripts
    pip install pip --upgrade
    pip install pyinstaller==2.1 setuptools==19.2 packaging==14.2
    pip install matplotlib pyreadline future
    pip install lxml python-dateutil pytz pyparsing six
    pip install pyserial 
    pip install pymavlink 
    
Download the MAVProxy `source <https://github.com/ArduPilot/MAVProxy>`_.

After making the desired changes, MAVProxy is required to be compiled 
into the Python directory (the modules won't work otherwise).
This needs to happen after any changes to the source code. This can be
done by running the :file:`./MAVProxy/MAVProxyWinUSB.bat` or 
:file:`./MAVProxy/MAVProxyWinLAN.bat` file. This will
perform the necessary build actions and then run MAVProxy. Some of the 
details in the batch files (port numbers, etc) may need to be altered to 
match the user's system configuration.

To create a one-click windows installer for MAVProxy, run ``MAVProxyWinBuild.bat```, 
which is in the ``./windows`` directory. The installer will be created in the 
``./windows/output`` directory. The `Inno Setup <http://www.jrsoftware.org/isdl.php#stable>`_ 
program will be required for this process and is assumed to be installed in the 
``C:\Program Files (x86)\Inno Setup 5\`` folder

Visual Studio Setup
-------------------

Visual Studio can be used to develop MAVProxy on Windows platforms.

Within the ``windows`` subfolder in MAVProxy, there is a Visual Studio Project file. It does require the `Python Tools for Visual Studio <http://microsoft.github.io/PTVS/>`_ to be installed.

.. figure:: VS1.png

Use the build or debug commands in Visual Studio to install MAVProxy to the local Python folder. This is required in order for MAVProxy to be run correctly.

MAVProxy can then be run by right-clicking on mavproxy.py and selecting ``Start`` (with or without debugging). The mavexplorer.py can be run in the same way.

.. figure:: VS2.png


