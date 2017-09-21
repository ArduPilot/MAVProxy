==========================
Image Processing with cuav
==========================

This section details how to use the cuav module.

The cuav module is a specialised module for performing on-the-fly or offline image analysis of imagery collected by a UAV (and other vehicle). It is tightly integrated with the Ardupilot Flight Controller, which it uses for giving precise coordinates of detected items in images.

Source code is available at https://github.com/CanberraUAV/cuav

----------
Installing
----------

The cuav module requires the following pre-requisites (this assumes the MAVproxy is already installed as per the instructions at :doc:`../getting_started/download_and_installation`):

.. code:: bash

    sudo apt-get install libusb-1.0.0-dev libdc1394-22-dev libopencv-dev
    pip install cuav
    
.. note::

    Some users may also require the following packages: python-pyexiv2 python-httplib2 libjpeg-turbo-progs
 
------------------------------
Compatibility and System Setup
------------------------------

The cauv module is compatible with any IEEE 1394 (Firewire) based cameras that conform to the 1394-based Digital Camera Specifications (also known as the IIDC or DCAM Specifications). Note this does include some USB 2.0 cameras from Point Grey.

A modified `dc1394 <http://damien.douxchamps.net/ieee1394/libdc1394/>`_ library is used for communication with the camera.

This module has only been tested with the `Point Grey Chameleon <http://www.ptgrey.com/chameleon-usb2-cameras>`_ camera.

-----
Usage
-----

Within MAVProxy, use the following to activate cuav:

.. code:: bash

    module load cuav.modules.camera
    
The following commands are available:

.. code:: bash

    camera start
    camera stop
    
These commands start and stop the image capturing and analysis processes. It is useful to command the camera start just after takeoff and stop just before landing.
    
    
.. code:: bash

    camera status
    
This displays the current status: Number of images captured, Error count, scanned images count, framerate, region count, image size, images in transmit queue (primary and secondary radio links), frames lost, scan queue size and efficieny.

It is useful to look at the scan queue size. If it is continually increasing, there is not enough computing power available to process the images in real-time.
    
.. code:: bash

    camera view
    
Displays the GUI for the captured images. This consists of two windows. The first window is a live view of captured images. The second window displays thumbnails of any images that pass the image detection threshold. These images can be right-clicked to download a full size image.

Additionally, the position of the interesting part of the images will be displayed on the map.

.. code:: bash

    camera set X Y
    
Change cuav setting X to value Y. Available settings are:

==================   ================================   ===============================
Setting              Description                        Value Range
==================   ================================   ===============================
depth                Image Depth                        8 or 16
save_pgm             Save Raw Images                    0 or 1
capture_brightness   Capture Brightness                 10 to 300
gamma                Capture Gamma                      0 to 1000
roll_stabilised      Roll Stabilised                    0 or 1 
roll_limit           Roll Stabilisation limit           0 to 90 degrees
altitude             UAV Altitude                       0 to 10000m
minalt               Minimum Altitude                   0 to 10000m
mpp100               MPPat100m                          0 to 10000
rotate180            Rotate the image 180 degrees       0 or 1
camparms             Camera parameters file             '\path\to\file'
filter_type          Filter Type                        'simple' or 'compactness'
blue_emphasis        Emphasise Blue colour              0 or 1
framerate            Frame (Capture) rate               1, 3, 7, 15
process_divider      Process Divider                    1 to 50
send2_divider        Send2 Divider                      1 to 50
use_capture_time     Use Capture Time                   0 or 1
gcs_address          GCS IP Address                     IP address 'XXX.XXX.XXX.XXX'
gcs_view_port        GCS IP Port                        0 to 30000
gcs_slave            GCS Slave IP Address               IP address 'XXX.XXX.XXX.XXX'
aircraft_address     UAV IP Address                     IP address 'XXX.XXX.XXX.XXX'
aircraft_port        UAV IP Port                        0 to 30000
bandwidth            Link1 Bandwidth                    bytes/sec
bandwidth2           Link2 Bandwidth                    bytes/sec
quality              Compression Quality                0 to 100
transmit             Transmit Enable                    0 or 1
send1                Link1 Transmit Enable              0 or 1
send2                Link2 Transmit Enable              0 or 1
maxqueue1            Maximum queue Link1                0 or higher
maxqueue2            Maxqueue queue Link2               0 or higher
thumbsize            Thumbnail Size                     10 to 200
mosaic_thumbsize     Mosaic Thumbnail Size              10 to 200
use_bsend2           Enable Link2                       0 or 1
minspeed             Min vehicle speed to save images   0 or higher
minscore             Min Score Link1                    0 to 5000
minscore2            Min Score Link2                    0 to 5000
packet_loss          Packet Loss Link1                  0 to 100
packet_loss2         Packet Loss Link2                  0 to 100
clock_sync           GPS Clock Sync                     0 or 1
brightness           Display Brightness                 0.1 to 10
debug                Debug enable                       0 or 1
==================   ================================   ===============================

---------------
Usage (offline)
---------------

For offline imagery analysis, there is a playback script. This requires the source code of cuav 
(https://github.com/CanberraUAV/cuav).

Within the ``cuav/tests/`` folder, the ``playback.py`` script can be used for playback. It streams 
an existing directory of image files and Mavlink telemetry over a network link to simulate a live 
feed from a UAV. It can be used as such:

.. code:: bash

    export FAKE_CHAMELEON=1
    cuav/tests/playback.py $FDIR/flight.tlog --imagedir=$IDIR --speedup=1 & rm -rf OBC2016/logs
    mavproxy.py --aircraft OBC2016 --master 127.0.0.1:14550
    
Where ``$FDIR`` is the folder containing the flight log and ``$IDIR`` contains the images. The 
images should be named according to ``raw<timestamp>.pgm``. An example is this is ``raw2016090700200251Z.pgm```. The file extension be be either pgm or jpg. In the case of jpg files, 
the ``--jpeg`` command line option must be used.

The ``export FAKE_CHAMELEON=1`` line tells cuav to use a folder as the imagery source, rather than
looking for a camera.

.. note::

    By default, MAVProxy and cuav will save the telemetry and images to file (it doesn't know that
    the imagery is coming from file rather than a UAV). For large datasets this may lead to large 
    amounts of disk space usage. Thus the ``& rm -rf OBC2016/logs`` command in the above example 
    will delete any previous telemetry and images before replaying it again.
    
.. note::

    Users may have issues trying to stop the playback process. Use ``pkill -9 -f playback.py`` to
    stop the playback process.

The commandline options are: 

==================   ================================   ===============================
Option               Description                        Default
==================   ================================   ===============================
--out                MAVLink output port (IP:port)      127.0.0.1:14550
--baudrate           baud rate                          57600
--imagedir           raw image directory                <none>
--condition          condition on mavlink log           <none>
--speedup            playback speedup                   1.0
--loop               playback in a loop                 False
--jpeg               use jpegs instead of PGMs          False
--HIL                send HIL_STATE messages            False
==================   ================================   ===============================


