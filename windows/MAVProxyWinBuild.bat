rem build the standalone MAVProxy.exe for Windows.
rem This assumes Python is installed in C:\Python27
SETLOCAL enableextensions

rem change this path if Python is installed somewhere non-standard
set PYTHON_LOCATION=C:\Python27

rem get the version
for /f "tokens=*" %%a in (
 '%PYTHON_LOCATION%\python returnVersion.py'
 ) do (
 set VERSION=%%a
 )


rem -----Upgrade pymavlink if needed-----
%PYTHON_LOCATION%\Scripts\pip install pymavlink -U

rem -----Build MAVProxy-----
cd ..\
%PYTHON_LOCATION%\python setup.py clean build install
cd .\MAVProxy
%PYTHON_LOCATION%\Scripts\pyinstaller --clean ..\windows\mavproxy.spec

rem -----Create version Info-----
@echo off
@echo %VERSION%> ..\windows\version.txt
@echo on

rem -----Build the Installer-----
cd  ..\windows\
"C:\Program Files (x86)\Inno Setup 5\ISCC.exe" /dMyAppVersion=%VERSION% -compile mavproxy.iss

pause
