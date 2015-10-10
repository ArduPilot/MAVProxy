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

First, a few pre-requisite packages need to be installed :

.. code:: bash

    sudo apt-get install python-opencv python-wxgtk python-pip python-dev

.. note::

    On some Linux systems, ``python-wxgtk`` may be instead named
    as ``python-wxgtk2.8``.
    
Then download and install MAVProxy via Pypi. Prerequisites will be
automatically downloaded too. Note a sudo may be required in some
circumstances if the install generates errors.

.. code:: bash

    pip install MAVProxy

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


