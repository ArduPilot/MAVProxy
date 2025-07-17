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

rem -----Install additional Python packages-----
python.exe -m pip install -U wheel setuptools pip
python.exe -m pip install pywin32 lxml pymavlink numpy matplotlib pyserial opencv-python PyYAML Pygame Pillow wxpython prompt-toolkit scipy future
python.exe -m pip install -U openai pyaudio
python.exe -m pip install -U pyinstaller==6.7.0 packaging 
python.exe -m pip install -U requests

rem -----Build MAVProxy-----
cd ..\
python.exe -m pip install .[recommended] --user
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
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Rover/apm.pdef.xml' -Destination 'Parameters\Rover.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Copter/apm.pdef.xml' -Destination 'Parameters\Copter.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Plane/apm.pdef.xml' -Destination 'Parameters\Plane.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Sub/apm.pdef.xml' -Destination 'Parameters\Sub.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.xml' -Destination 'Parameters\AntennaTracker.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Heli/apm.pdef.xml' -Destination 'Parameters\Heli.xml'"
powershell.exe "Start-BitsTransfer -Source 'http://autotest.ardupilot.org/Parameters/Blimp/apm.pdef.xml' -Destination 'Parameters\Blimp.xml'"

rem -----Build the Installer-----
cd .\windows
rem Newer Inno Setup versions do not require a -compile flag, please add it if you have an old version
"%INNOSETUP%\ISCC.exe" /dMyAppVersion=%VERSION% mavproxy.iss

pause
