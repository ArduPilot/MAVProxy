****************
Relay Management
****************

.. code:: bash

    module load relay
    
Control the operation of the output servos. Note this should only be
used for the non-flight-control servos. An example of usage would be
camera shutter controls. Use ``servo set [SERVO_NUM] [PWM]`` to set an
output servo to a particular PWM value. Use
``servo repeat [SERVO_NUM] [PWM] [COUNT] [PERIOD]`` to make the servo
repeat between it's current and specified PWM values.

If relay controls are available, the ``relay set [RELAY_NUM] [0|1]`` to
set a particular relay to 0 or 1. Similar to the servos, use
``relay repeat [RELAY_NUM] [COUNT] [PERIOD]`` to setup a repeating
relay.

