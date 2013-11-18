setlocal
SET PATH=%PATH%;..\mavlink;..\mavlink\pymavlink\examples

mavproxy.py --master=COM10 --baudrate=115200

pause