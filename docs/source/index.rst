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

MAVProxy was first developed by `CanberraUAV <http://www.canberrauav.org.au>`_,
to enable the use of companion computing and multiple datalinks with 
ArduPilot. It has grown to be one of the most versatile tools in the ArduPilot 
ecosystem, and many of the features users now see in other GCS tools 
can trace their origins to MAVProxy.

The team at CanberraUAV still lead development of MAVProxy: 
they are a volunteer organisation, but would appreciate your support.

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


Screenshots
============

.. figure:: getting_started/mavproxy_linux.jpg

   MAVProxy running under Ubuntu

.. figure:: getting_started/mavproxy_windows.jpg

   MAVProxy running under Windows 7

Printable Documentation
=======================

A 1 page printable "cheatsheet" of commonly used MAVProxy commands is available below.

`Libreoffice odt <./_static/files/MAVProxyCheetsheet.odt>`_.

`Pdf format <./_static/files/MAVProxyCheetsheet.pdf>`_.


License
=======

MAVProxy is released under the GNU General Public License v3 or later.

Except where otherwise noted, *this documentation* is licensed under
`CC Attribution-Share Alike 3.0 Unported <https://creativecommons.org/licenses/by-sa/3.0/>`_.


Core Team
=========

MAVProxy is maintained by `Andrew Tridgell <https://github.com/tridge>`_, `Peter Barker <https://github.com/peterbarker>`_ and `Stephen Dade <https://github.com/stephendade>`_

This page was generated using Sphinx SSG and is hosted using `GitHub Pages <http://pages.github.com>`_.


.. toctree::
    :hidden:

    getting_started/index
    uav_configuration/index
    modules/index
    analysis/index
    development/index






