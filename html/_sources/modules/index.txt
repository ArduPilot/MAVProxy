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
    
    adsb
    antenna
    auxopt
    battery
    cameraview
    cmdlong
    console
    dataflash_logger
    devop
    DGPS
    firmware
    followtest
    gasheli
    gimbal
    GPSInput
    graph
    hil
    horizon
    joystick
    kmlread
    link
    log
    magical
    map
    misseditor
    nsh
    ppp
    rcsetup
    relay
    sensors
    serial
    signing
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
    signing
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
    adsb



