=======================
Development Environment
=======================

For developing MAVProxy, a slightly more complex setup is required. The
following section will show how this setup is achieved.

This section assumes basic knowledge of setting up development
environments, Python and git.

Linux
=====

Install the Python and MAVlink libraries as per the
topic :doc:`../getting_started/download_and_installation`. Additionally, install:

.. code:: bash

    pip install setuptools

Use git to download the MAVProxy source:

.. code:: bash

    git clone https://github.com/Dronecode/MAVProxy.git

After making the desired changes, MAVProxy is required to be installed
(the modules won't work otherwise). This needs to happen after any
changes to the source code. This can be done by:

.. code:: bash

    python setup.py build install --user

MAVProxy can then be run as per normal.

.. note::

    Those using `Dronekit <http://python.dronekit.io/>`_ may also require the following packages: python-dev python-serial python-pyparsing via:
    
    .. code:: bash
    
        sudo apt-get install python-serial python-pyparsing

Windows
=======

In Windows, the process is a bit more complicated.

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

Use ``pip`` to install the other packages:

.. code:: bash

    cd C:\Python27\Scripts
    pip install pyinstaller 
    pip install matplotlib 
    pip install pyreadline 
    pip install pyserial 
    pip install pymavlink  
    pip install lxml python-dateutil pytz pyparsing six

Download the MAVProxy `source <https://github.com/Dronecode/MAVProxy>`_.

After making the desired changes, MAVProxy is required to be compiled
and copied into the Python directory (the modules won't work otherwise).
This needs to happen after any changes to the source code. This can be
done by running the :file:`./MAVProxy/MAVProxyWinUSB.bat` or 
:file:`./MAVProxy/MAVProxyWinLAN.bat` file. This will
perform the necessary build actions and then run MAVProxy. Some of the 
details in the batch files (port numbers, etc) may need to be altered to 
match the user's system configuration.

Visual Studio Setup
-------------------

In the ``windows`` folder, there is a Visual Studio Project file. It does require the `Python Tools for Visual Studio <http://microsoft.github.io/PTVS/>`_ to be installed.

.. figure:: VS1.png

Use the build or debug commands to run the ``setup.py`` to build MAVProxy. Then right-click on mavproxy.py and select ``Start`` (with or without debugging). The mavexplorer.py can be run in the same way.

.. figure:: VS2.png


