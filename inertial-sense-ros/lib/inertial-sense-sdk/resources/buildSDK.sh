#!/bin/bash

# Needed to setup cross compile
# sudo apt-get install libc6-dev-i386
# sudo apt-get install gcc-multilib g++-multilib

echo Building x32 and x64 static libraries

rm -f linux.zip
rm -rf bin
mkdir bin
mkdir bin/Linux

BIN_DIR="bin/Linux"
RES_DIR="../../resources"

cd ..
cd InertialSenseCLTool/

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32
make -j 7
echo bin/cltool $RES_DIR/$BIN_DIR/cltool-x32
cp bin/cltool $RES_DIR/$BIN_DIR/cltool-x32
cp lib/libInertialSense.a $RES_DIR/$BIN_DIR/libInertialSense-x32.a

#exit

cd ..
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS=-m64 -DCMAKE_C_FLAGS=-m64
make -j 7
cp bin/cltool $RES_DIR/$BIN_DIR/cltool-x64
cp lib/libInertialSense.a $RES_DIR/$BIN_DIR/libInertialSense-x64.a

cd $RES_DIR

zip -r bin.zip bin

#rm -rf build
