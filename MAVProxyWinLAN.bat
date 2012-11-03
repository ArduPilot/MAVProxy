setlocal
SET PATH=%PATH%;..\mavlink;..\mavlink\pymavlink\examples

mavproxy.py --master=192.168.1.6:14550

pause