rem This demonstrates multiple TCP and UDP outputs from MAVProxy
rem It will send a mavlink stream to the first connection to this computer's
rem IP address and UDP port 15000 and TCP port 16000
mavproxy.exe --master=0.0.0.0:14550 --out=udpin:0.0.0.0:15000 --out=tcpin:0.0.0.0:16000
pause
