rem build the standalone MAVProxy.exe for Windows.
rem This assumes Python and pip are on the system path
rem This assumes InnoSetup is installed in C:\Program Files (x86)\Inno Setup 6
rem   If it is not, change the INNOSETUP environment variable accordingly
SETLOCAL enableextensions

if "%INNOSETUP%" == "" (set "INNOSETUP=C:\Program Files (x86)\Inno Setup 6")

rem get the version
for /f "tokens=*" %%a in (
 'python.exe returnVersion.py'
 ) do (
 set VERSION=%%a
 )
 
rem -----Upgrade pymavlink if needed-----
if exist "..\..\pymavlink" (
 rem Rebuild and use pymavlink from pymavlink sources if available
 pushd ..\..\pymavlink
 python.exe setup.py build install --user
 popd
) else (
 if exist "..\..\mavlink\pymavlink" (
  rem Rebuild and use pymavlink from mavlink\pymavlink sources if available
  pushd ..\..\mavlink\pymavlink
  python.exe setup.py build install --user
  popd
 ) else (
  pip.exe install pymavlink -U --user
 )
)

rem -----Build MAVProxy-----
cd ..\
python.exe -m pip install . --user
cd .\MAVProxy
copy ..\windows\mavproxy.spec
pyinstaller -y --clean mavproxy.spec
del mavproxy.spec

rem -----Create version Info-----
@echo off
@echo %VERSION%> ..\windows\version.txt
@echo on

rem -----Download parameter files-----
cd  ..\
mkdir Parameters
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/APMrover2/apm.pdef.xml' -Destination 'Parameters\Rover.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml' -Destination 'Parameters\ArduCopter.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml' -Destination 'Parameters\ArduPlane.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/ArduSub/apm.pdef.xml' -Destination 'Parameters\ArduSub.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.xml' -Destination 'Parameters\AntennaTracker.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Rover-defaults.parm' -Destination 'Parameters\Rover-defaults.parm'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Copter-defaults.parm' -Destination 'Parameters\ArduCopter-defaults.parm'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Plane-defaults.parm' -Destination 'Parameters\ArduPlane-defaults.parm'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Sub-defaults.parm' -Destination 'Parameters\ArduSub-defaults.parm'"

rem -----Build the Installer-----
cd .\windows
rem Newer Inno Setup versions do not require a -compile flag, please add it if you have an old version
"%INNOSETUP%\ISCC.exe" /dMyAppVersion=%VERSION% mavproxy.iss

pause
