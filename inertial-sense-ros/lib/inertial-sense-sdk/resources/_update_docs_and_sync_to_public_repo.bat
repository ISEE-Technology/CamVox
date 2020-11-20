@echo off

set SDKDIR=..\..\..\InertialSenseSDK
rem set COMDIR=..\..\..\inertialsense_serial_protocol

echo.
echo Prepare SDK for release.
::echo.

cd ..

echo.
echo Cleaning SDK
call :remove_sub_directories build
call :remove_sub_directories Debug
call :remove_sub_directories Release
call :remove_sub_directories .vs
call :remove_sub_directories .vs
call :remove_sub_directories IS_logs

cd resources

echo.
echo Update doxygen docs
call update_doxygen_docs.bat

cd ..

echo.
echo Sync SDK into %SDKDIR% repo
robocopy %SDKDIR%\.git %SDKDIR%tmp\.git /MIR /XA:H
robocopy . %SDKDIR% /MIR /XA:H
robocopy %SDKDIR%tmp\.git %SDKDIR%\.git /MIR /XA:H
::rmdir /S /Q %SDKDIR%tmp

cd src 

rem echo.
rem echo Sync SDK into %COMDIR% repo
rem robocopy . ..\%COMDIR% data_sets.c data_sets.h ISComm.c ISComm.h ISConstants.h

echo.
echo Sync logInspector into SDK
cd ..\..\..\python\src\logInspector
robocopy . %SDKDIR%\logInspector assets /MIR
robocopy . %SDKDIR%\logInspector docs /MIR
robocopy . %SDKDIR%\logInspector include /MIR
robocopy . %SDKDIR%\logInspector src /MIR
robocopy . %SDKDIR%\logInspector assets /MIR
robocopy . %SDKDIR%\logInspector _build_log_inspector.bat _clean_log_inspector.bat CMakeLists.txt __init__.py logInspector.py logPlotter.py logReader.py README.md setup.py

echo.
echo Sync logInspector into SDK
cd ..\..\..\python\src\logInspector
robocopy . %SDKDIR%\logInspector assets /MIR
robocopy . %SDKDIR%\logInspector docs /MIR
robocopy . %SDKDIR%\logInspector include /MIR
robocopy . %SDKDIR%\logInspector src /MIR
robocopy . %SDKDIR%\logInspector assets /MIR
robocopy . %SDKDIR%\logInspector _build_log_inspector.bat _clean_log_inspector.bat CMakeLists.txt __init__.py logInspector.py logPlotter.py logReader.py README.md setup.py

REM echo.
REM echo Sync inertial_sense_ros repo
REM cd ..\..\..\catkin_ws\src\inertial_sense_ros
REM update_and_sync_ros_node.bat

echo.
echo Done.
echo.

:: Sleep using ping for invalid ip and timeout
::ping 123.45.67.89 -n 1 -w 5000 > nul

:: Wait for user input
pause
:: Success
exit 0

:remove_sub_directories
echo - removing "%~1" directories
for /d /r "." %%a in (%~1\) do (
    if exist "%%a" rmdir /s /q "%%a"
)

