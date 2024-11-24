cd ..\
python.exe -m pip install --upgrade build . --user
python.exe .\MAVProxy\mavproxy.py --master=0.0.0.0:14550 --console
pause
