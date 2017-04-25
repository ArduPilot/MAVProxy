rem This demonstrates saving the logfile (tlog) to a specific directory
rem In this case all logs will be saved to the user's Documents/Test1 folder
mavproxy.exe --master=0.0.0.0:14550 --state-basedir=%USERPROFILE%\Documents --aircraft=Test1
pause