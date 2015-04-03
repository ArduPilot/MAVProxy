rem build the standalone MAVProxy.exe for Windows.
rem This assumes Python is installed in C:\Python27
cd ..\
python setup.py build install --user
cd .\MAVProxy
C:\Python27\Scripts\pyinstaller --clean ..\windows\mavproxy.spec
pause
