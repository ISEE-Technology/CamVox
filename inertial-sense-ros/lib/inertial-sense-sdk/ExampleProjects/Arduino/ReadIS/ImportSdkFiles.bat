@echo off

REM # This script copies needed SDK files into the Arduino project.

robocopy ..\..\..\src src\ISsdk data_sets.c data_sets.h ISComm.h ISComm.c ISConstants.h
robocopy ..\..\..\hw-libs\printf-master src\hw-libs\printf-master printf.h printf.c
