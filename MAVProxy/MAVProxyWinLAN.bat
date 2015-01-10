setlocal

SET PATH=%PATH%;..\mavlink;..\mavlink\pymavlink\examples
SET PYTHONPATH=%PYTHONPATH%;C:\Python27\Lib;C:\Python27\Lib\site-packages;.\MAVProxy\modules;.\MAVProxy\modules\lib
cd ..\
.\MAVProxy\mavproxy.py --master=192.168.1.6:14550

pause
