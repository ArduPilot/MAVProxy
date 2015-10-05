**************
Joystick Input
**************

.. code:: bash

    module load joystick
    
The joystick module passes control signals (roll/pitch/yaw/throttle)
from a joystick (or similar) as RC commands to the APM. Thus it can be
used for control of a UAV without an RC transmitter.

When the module is loaded and if there is a compatible joystick detected,
control signals will start automatically be passed through. In nearly all 
cases only the 4 flight axes are passed through. Usage of the other RC channels  should be done on the command line.

The following joysticks and RC transmitter dongles are compatible:

- `Carolbox
  USB <http://www.hobbyking.com/hobbyking/store/__13597__USB_Simulator_Cable_XTR_AeroFly_FMS.html>`_
- `Saili Simulator
  USB <http://www.hobbyking.com/hobbyking/store/__13597__USB_Simulator_Cable_XTR_AeroFly_FMS.html>`_
- Sony Playstation 3 Controller
- GREAT PLANES InterLink Elite
- Great Planes GP Controller
- `WAILLY PPM TO USB
  Adapter <http://www.amazon.com/gp/product/B000RO7JAI/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1>`_
- Cheap Ebay 'FMS Simulator'


