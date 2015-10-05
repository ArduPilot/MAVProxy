*******
Modules
*******

MAVProxy can be extended with modules. These run in parallel threads to
the main MAVProxy program in order to maintain overall stability.

Modules can include such things as GUI elements and diagnostic and
monitoring applications.

.. toctree::
    :maxdepth: 1
    :hidden:
    
    antenna
    auxopt
    cameraview
    console
    dgps
    gimbals
    graph
    hil
    joystick
    log
    map
    misseditor
    nsh
    relay
    sensors
    serial
    speech
    terrain
    tracker
    tuneopt
    
Module Management
=================

Modules need to be loaded before they can be used. The following command
can be used:

.. code:: bash

    module load modulename

Other management commands for unloading, reloading and listing currently
loaded modules include:

.. code:: bash

    module unload modulename
    module reload modulename
    module list

Default Modules
---------------

MAVProxy starts with several modules by default. They are:

.. code:: bash

    log
    wp
    rally
    fence
    param
    relay
    tuneopt
    arm
    mode
    calibration
    rc
    auxopt
    misc
    cmdlong
    battery
    terrain
    output
    


