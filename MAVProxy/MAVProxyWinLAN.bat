cd ..\
python setup.py build install --user
python .\MAVProxy\mavproxy.py --master=0.0.0.0:14550 --console
pause
