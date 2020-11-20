#!/bin/bash

# This script copies needed SDK files into the Arduino project.

mkdir -p src/ISsdk
mkdir -p src/hw-libs/printf-master
cp ../../../src/{data_sets.c,data_sets.h,ISComm.h,ISComm.c,ISConstants.h} src/ISsdk/.
cp ../../../hw-libs/printf-master/{printf.c,printf.h} src/hw-libs/printf-master/.
