@echo off
setlocal enabledelayedexpansion

:: Parameters
::set sdkDir=docs
::set inputfilename=release-notes-template.txt

:: Read Release version, date, and time
::set /p releaseversion=<version\releaseVersion.txt
::set /p releasedate=<version\releaseDate_form.txt
::set /p releasetime=<version\releaseTime_form.txt

cd ..


:: Remove Old Files
rmdir /q /s docs/html

:: Create Directory
::mkdir docs/html



echo Generating Doxygen Documentation

:: Update Doxygen documentation
::doxygen resources\DoxyfileSDK
( type resources\DoxyfileSDK & echo PROJECT_NUMBER="Release 1.8.2" ) | doxygen -
::( type resources\DoxyfileSDK & echo PROJECT_NUMBER="Release %releaseversion%,  %releasedate%,  %releasetime%" ) | doxygen -



:: Zip files
::powershell.exe -nologo -noprofile -command "& { Add-Type -A 'System.IO.Compression.FileSystem'; [IO.Compression.ZipFile]::CreateFromDirectory('%sdkDir%','%sdkDir%.zip'); }"
::set cmdstr=CScript zip_folder.vbs %START_PATH%Firmware %START_PATH%%releasedir%\Firmware.zip
::echo %cmdstr%


:: Automatically Open Doxygen Docs
start index.html



exit /b
:: Script End


:GetRelease <filename>
set rtn=-1
for /F "tokens=2 delims=: " %%b in ('findstr /b "Release " %1') do (
   set rtn=%%b
   exit /b
)
set rtn=%rtn:"=%
exit /b
