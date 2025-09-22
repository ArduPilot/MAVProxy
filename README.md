Installation:

sudo apt install -y python3-wxgtk4.0 python3-matplotlib
python3 -m venv --system-site-packages venv
pip install -e .[recommended]


####

First run:

run ardupilot simulation in another console, eg:

```sim_vehicle.py -v ArduCopter --no-mavproxy```

check on which port the SITL is runnning, and then run mavproxy, eg:

```mavproxy.py --master=tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --map```

For take off:
mode GUIDED
arm throttle
takeoff 10

wp clear
wp add -35.3632636 149.1662277 20
wp add -35.3632636 149.1666691 20
wp add -35.3629518 149.1664484 20
wp list
mode AUTO


In order to run graph, do eg.:

```
module load graph
graph ATTITUDE.roll ATTITUDE.pitch ATTITUDE.yaw
graph VFR_HUD.climb # Vertical speed (positive up)
```

In order to make changes visible in the code:
```
module unload graph
module load graph
```

to get list of possible values type:
```status```

to change tickresolution / refresh rate of the graph do:

```module tickresolution <value>```

where value is refresh period in seconds


![GitHub Actions](https://github.com/ardupilot/MAVProxy/actions/workflows/windows_build.yml/badge.svg)

MAVProxy

This is a MAVLink ground station written in python. 

Please see https://ardupilot.org/mavproxy/index.html for more information

This ground station was developed as part of the CanberraUAV OBC team
entry

License
-------

MAVProxy is released under the GNU General Public License v3 or later


Maintainers
-----------

The best way to discuss MAVProxy with the maintainers is to join the
mavproxy channel on ArduPilot discord at https://ardupilot.org/discord

Lead Developers: Andrew Tridgell and Peter Barker

Windows Maintainer: Stephen Dade

MacOS Maintainer: Rhys Mainwaring
