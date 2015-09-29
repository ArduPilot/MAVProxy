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
topic :doc:`../getting_started/download_and_installation`. Additionally, install the
``setuptools`` package via ``pip``

Use git to download the MAVProxy source:

.. code:: bash

    git clone https://github.com/tridge/MAVProxy.git

After making the desired changes, MAVProxy is required to be installed
(the modules won't work otherwise). This needs to happen after any
changes to the source code. This can be done by:

.. code:: bash

    python setup.py build install --user

MAVProxy can then be run as per normal.

Windows
=======

In Windows, the process is a bit more complicated.

You will need to have `Python
2.7 <http://www.python.org/download/releases/2.7/>`_,
`wxPython <http://www.wxpython.org/download.php>`_ and
`pygame <http://pygame.org/download.shtml>`_ installed first.

Next, open up a console in the Python scripts install path
(C:\\Python27\\Scripts or similar). Use ``pip install [filepath]`` to install them

- `numPY <http://www.lfd.uci.edu/~gohlke/pythonlibs/#numpy>`_
- `Pillow (replaces
  PIL) <http://www.lfd.uci.edu/~gohlke/pythonlibs/#pillow>`_
- `OpenCV <http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv>`_

Use ``pip`` to install the other packages:

.. code:: bash

    pip install pyinstaller 
    pip install matplotlib 
    pip install pyreadline 
    pip install pyserial 
    pip install pymavlink  
    pip install lxml python-dateutil pytz pyparsing six

Download the MAVProxy `source <https://github.com/tridge/MAVProxy>`_.

After making the desired changes, MAVProxy is required to be compiled
and copied into the Python directory (the modules won't work otherwise).
This needs to happen after any changes to the source code. This can be
done by running the :file:`./MAVProxy/MAVProxyWinUSB.bat` or 
:file:`./MAVProxy/MAVProxyWinLAN.bat` file. This will
perform the necessary build actions and then run MAVProxy. Some of the 
details in the batch files (port numbers, etc) may need to be altered to 
match the user's system configuration.

