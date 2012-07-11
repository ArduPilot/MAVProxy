#!/bin/bash
echo "As the first parameter, please pass in the device name where your APM is connected ( ie the USB or Telemetry device )"
mkdir ./GUI
echo "module load netconsole" > ./GUI/mavinit.scr
python2.7 mavproxy.py --master=$1 --out=127.0.0.1:6678 --aircraft=GUI & 
cd mavgui
python2.7 mavgui.py


