==================
Module Development
==================

This section is a quick tutorial on how to create custom modules in
MAVProxy.

Location
========

If developing from the MAVProxy source code, modules are placed in the
``MAVProxy/modules`` folder. Then the ``setup.py build install --user``
routine needs to be run in order to (re)install MAVProxy and the modules
in this folder which will be automatically picked up and become part of
the installation.

If developing from the Windows installation, simply copy the modules
into the MAVProxy installation folder (Usually C:\\Program Files
(x86)\\MAVProxy).

In either case, the module can then be loaded then using the
``module load myModule`` in MAVProxy.

Module File Structure
=====================

At the base level, a MAVProxy module is an instance of the MPModule
class, with hooks for initialisation and reading Mavlink packets:

.. code:: python

    #!/usr/bin/env python
    '''module template'''
    import time, math
    from pymavlink import mavutil
    from MAVProxy.modules.lib import mp_module
    from MAVProxy.modules.lib.mp_settings import MPSetting
    
    class TestModule(mp_module.MPModule):
      def __init__(self, mpstate):
        super(TestModule, self).__init__(mpstate, "test", "test module")
        '''initialisation code'''

      def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        
      def init(mpstate):
        '''initialise module'''
        return TestModule(mpstate)           ``

The initialisation section defines commands and module-wide variables.
To add a command:

.. code:: python

    self.add_command('bat', self.cmd_bat, "show some information")

The above will add the "bat" command, which the MAVProxy help will show
"show some information" and the self.cmd\_bat function will be run.

Within the packet handling, the
`Mavlink <https://pixhawk.ethz.ch/mavlink/>`_ reference is useful for
filtering the incoming packets for the relevant data:

.. code:: python

    if m.get_type() == 'VFR_HUD':
      curspeed =  m.airspeed

There are a great number of MAVProxy modules, so the best reference
would be to look through the these
`modules <https://github.com/tridge/MAVProxy/tree/master/MAVProxy/modules>`_
for examples of what you want to do.