#!/bin/sh

echo $PATH
echo ######################################################
echo ########################11111#########################
echo ######################################################


cd /mnt/cube_slam/src/objectslam/object_slam/Thirdparty/g2o
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release
make -j2 

echo ######################################################
echo ########################22222#########################
echo ######################################################

cd /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/g2o
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release
make -j2 
echo ######################################################
echo ########################33333#########################
echo ######################################################


cd /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/DBoW2
mkdir build
cd build
cmake /mnt/cube_slam/src/objectslam/orb_object_slam/Thirdparty/DBoW2 -DCMAKE_BUILD_TYPE=Release
make -j2
echo ######################################################
echo #######################444444##########################
echo ######################################################


cd /mnt/cube_slam
catkin_make -j1
