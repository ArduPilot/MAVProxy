rem build the standalone MAVProxy.exe for Windows.
rem This assumes Python is installed in C:\Python27
SETLOCAL enableextensions

rem get the version
for /f "tokens=*" %%a in (
 'python returnVersion.py'
 ) do (
 set VERSION=%%a
 )


rem -----Upgrade pymavlink if needed-----
C:\Python27\Scripts\pip install pymavlink -U

rem -----Build MAVProxy-----
cd ..\
python setup.py clean build install --user
cd .\MAVProxy
C:\Python27\Scripts\pyinstaller --clean ..\windows\mavproxy.spec

rem -----Create version Info-----
@echo off
@echo %VERSION%> ..\windows\version.txt
@echo on

rem -----Build the Installer-----
cd  ..\windows\
"C:\Program Files (x86)\Inno Setup 5\ISCC.exe" /dMyAppVersion=%VERSION% -compile mavproxy.iss

pause
