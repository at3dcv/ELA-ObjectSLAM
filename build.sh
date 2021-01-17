#!/bin/sh

cd /mnt/cube_slam/src/objectslam/object_slam/Thirdparty/g2o
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release
make -j2 

cd /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/g2o
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release
make -j2 

cd /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/DBoW2
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/DBoW2 -DCMAKE_BUILD_TYPE=Release
make -j2

cd /mnt/cube_slam
catkin_make -j1