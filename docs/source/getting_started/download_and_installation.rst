=========================
Download and Installation
=========================

The following instructions are for user that just want to run MAVProxy. Developers should look at the :doc:`Developer's Guide <../development/mavdevenv>` for setting up the development environment.

Windows
=======

A complete windows installer for MAVProxy is available at
http://firmware.diydrones.com/Tools/MAVProxy/.

.. note::

    Please note that the "missionedit" module does not currently run on the
    Windows package of MAVProxy. This is being investigated and will
    hopefully be fixed soon. See https://github.com/Dronecode/MAVProxy/issues/129 for more details.


Linux
=====

First, a few pre-requisite packages need to be installed:

.. code:: bash

    sudo apt-get install python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml

.. note::

    On some older Linux systems, ``python-wxgtk3.0`` may be instead named
    as ``python-wxgtk2.8``.

.. note::

    MAVProxy runs on the Python 2.7.x environment. Ensure your system is running
    the correct version of Python.
        
Then download and install MAVProxy via Pypi. Prerequisites will be
automatically downloaded too. Note a sudo may be required in some
circumstances if the install generates errors:

.. code:: bash

    pip install MAVProxy
    
Depending on user and system settings, there may be some extra configuration required.

If not already set, MAVProxy needs to be on the system path:

.. code:: bash

    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

The user permissions may also need to be changed to allow access to serial devices:
   
.. code:: bash

    sudo adduser <username> dialout    

The system will need to be logged out and logged back in again to apply the above two changes.

Mac
===

If you’re on Mac OSX, you can use Homebrew to install WXMac.

.. code:: bash

    brew tap homebrew/science
    brew install wxmac wxpython opencv

Uninstall python-dateutil (OSX and Windows come bundled with a version that is not supported for some dependencies):

.. code:: bash

    sudo pip uninstall python-dateutil

Install MAVProxy and its remaining dependencies from the public PyPi repository:

.. code:: bash

    sudo pip install numpy pyparsing
    sudo pip install MAVProxy


