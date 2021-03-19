echo "Building ROS nodes"

cd camvox/online
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release 
make -j10


