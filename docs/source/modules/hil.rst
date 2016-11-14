***
HIL
***

.. code:: bash

    module load hil
    
Provides a Hardware-In-the-Loop (HIL) interface to an APM running HIL code. It
is to be used in conjunction with the Tools/autotest/jsbsim/runsim.py
file in the APM source code, which provides a flight dynamics simulator
(JSBSim).

It is used for testing modifictions to the APM code in a simulated
environment.

See the :doc:`../analysis_and_simulation/simulation` page  
for more details on running HIL mode.
