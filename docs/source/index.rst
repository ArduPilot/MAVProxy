.. MAVProxy documentation master file, created by
   sphinx-quickstart on Wed Aug 19 05:17:36 2015.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

========
MAVProxy
========

**A UAV ground station software package for MAVLink based systems**

MAVProxy is a fully-functioning GCS for UAV's. The intent is for a
minimalist, portable and extendable GCS for any UAV supporting the
MAVLink protocol (such as the APM).

Features
=========

-  It is a command-line, console based app. There are plugins included
   in MAVProxy to provide a basic GUI.
-  Can be networked and run over any number of computers.
-  It's portable; it should run on any POSIX OS with python, pyserial,
   and ``select()`` function calls, which means Linux, OS X, Windows, and
   others.
-  The light-weight design means it can run on small netbooks with ease.
-  It supports loadable modules, and has modules to support console/s,
   moving maps, joysticks, antenna trackers, etc
-  Tab-completion of commands.

A list of commits to MAVProxy (closest thing to a changelog that is
available) can be seen `here <https://github.com/Dronecode/MAVProxy/commits/master>`_.

Screenshots
============

.. figure:: getting_started/mavproxy_linux.jpg

   MAVProxy running under Ubuntu

.. figure:: getting_started/mavproxy_windows.jpg

   MAVProxy running under Windows 7

License
=======

MAVProxy is released under the GNU General Public License v3 or later

Core Team
=========

MAVProxy is maintained by `Andrew Tridgell <https://github.com/tridge>`_ and documentation is by 
`Stephen Dade <https://github.com/stephendade>`_

This page was generated using Sphinx SSG and is hosted using `GitHub Pages <http://pages.github.com>`_.




.. toctree::
    :hidden:

    getting_started/index
    uav_configuration/index
    modules/index
    analysis_and_simulation/index
    development/index
    cuav_module/index
    license


