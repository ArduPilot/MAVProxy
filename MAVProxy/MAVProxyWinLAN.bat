cd ..\
python setup.py build install --user
python .\MAVProxy\mavproxy.py --master=192.168.1.6:14550 --console
pause
