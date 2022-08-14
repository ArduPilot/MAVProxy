## install MAVProxy on Android ?

Yes, it's possible to install MAVProxy on Android, totally unsupported, no guarantees, good luck, but here goes.

### install/load Termux Android app from .apk on github
see https://github.com/termux/termux-app for more.

eg: https://github.com/termux/termux-app/releases/download/v0.118.0/termux-app_v0.118.0+github-debug_armeabi-v7a.apk 

tip: do NOT install from play store, its a really old version. 

Once installed, in the termux console run these commands:
```
pkg upgrade 
```
[ answer 'y' to all questions.]

```
pkg install python
```
[3.10.x at time of testing]

```
pkg install libxml2* 
```
[ for -dev pkg ]

```
pkg install libxslt*
```
[ for -dev pkg ]

```
pkg install python-*   
```
[ for python-dev , be sure to include the '-' here.]

```
pip install --upgrade pip
```
pip is needed for the next few packages

```
pip install wheel
pip install pymavlink
```
wheel isn't installed automatically in some cases, so we install it manually first

#https://superuser.com/questions/1724176/why-pip-throwing-an-error-when-installing-numpy-in-termux
```
MATHLIB="m" pip3 install numpy
```
numpy doesnt install without this extra prefix on android , so we do it first as well.

```
pip install mavproxy
```
finally we install mavproxy!

```
pkg install mlocate
updatedb
```
optionally other things that might be helpful

```
locate mavproxy.py
```
where is it?

```
mavproxy.py

mavproxy.py --master=udp:1.2.3.4:14550
```
a few different ways to run it!
eg: if u have udp mavlink data being forward to your phone over wifi, then 1.2.3.4 should be your PHONE's ip address, and it will receive the data.


known issues:
 *  --console and all the GUI parts of mavproxy don't work, as the 'WX' package isn't available for android.
 *  no idea if using a usb/serial connecton works, but UDP-over-wifi definitely does! 
 *  totally unsupported, may break at any time and stop working.  please do a pull-request to edit this file if you find these instructions no longer work.


<img src="https://raw.githubusercontent.com/davidbuzz/MAVProxy/master/docs/MavProxy_Screenshot_Termux.jpg" width="200">

