rem build the standalone MAVProxy.exe for Windows.
rem This assumes Python is installed in C:\Python27
cd ..\MAVProxy
C:\Python27\Scripts\pyinstaller ..\windows\mavproxy.spec
pause