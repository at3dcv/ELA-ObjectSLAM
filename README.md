
## AT3DCV: ObjectSLAM

This repository is an extension of [Shichao Yang's](https://shichaoy.github.io./) CubeSLAM implementation which in turn is based on **CubeSLAM: Monocular 3D Object SLAM**, IEEE Transactions on Robotics 2019, S. Yang, S. Scherer  [**PDF**](https://arxiv.org/abs/1806.00557).
It is the result of a collective effort by Ezgi Cakir, Andy Chen, and Leander Lauenburg during the cause of the practical course Advanced Topics of 3D computer vision.

 ## Installation

 The project is written in C++ and based on ROS nodes. For a quick and easy installation we provide a Docker based setup

 ### Quick Start

1. Clone this[https://gitlab.lrz.de/ge39gol/objectslam] repo to \</your_path\>/cubeslam/src/objectslam/.
2. Clone the [Docker Ubuntu VNC repo](https://github.com/ct2034/docker-ubuntu-vnc-desktop).
3. Replace the Dockerfile in the Docker Ubuntu VNC repo with the docker file provided in this repo.
4. Open a terminal, navigate to the Docker Ubuntu VNC repo and run `sudo docker build -t at3dcv_2020`.
5. After the image is build, run `sudo docker run -it --rm -p 6080:80 -v </your_path>/cubeslam/src/objectslam/:/mnt/ at3dcv_2020:latest`.
6. Now open your browser and go to `http://127.0.0.1:6080/`.
7. Accessing the Ubuntu instance via your browser open a terminal and run:
	1. soure /opt/ros/kinetic/setup.bash
	2. cd /mnt/cubeslam/src
	3. run bash build.sh
	4. catkin_init_workspace
	5. source /mnt/cubeslam/devel/setup.bash
	6. echo "source /mnt/cubeslam/devel/setup.bash" >> ~/.bashrc

## Running the examples

1. Make sure that the `ORBvoc.txt` file is located in the `Vocabulary` folder of the `orb_object_slam` folder, with in your mounted `objectslam` folder. Should it not be present download the file from the orginal [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2/tree/master/Vocabulary) repo and place it into the mentioned folder.
2. Download [this](https://drive.google.com/file/d/15_6ugaKt5t7rSIzpFhDgqXrkb48qv3Mt/view?usp=sharing) data folder containing the bag-file, bounding boxes, edge detections and convex hulls and place it, on the same level as the folder `cubeslam`, in to a folder `Datasets`.
3. Open three terminals
4. In each terminal run
	1. `soure /opt/ros/kinetic/setup.bash`
	2. `source /mnt/cubeslam/devel/setup.bash``
5. Terminal one run: `roscore`
6. Terminal two run: `rosbag play - 0.5 <path-to-datafolder>/freiburg3_walking_xyz.bag /camera/rgb/image_color:=/kitti/left/image_raw`
7. Terminal tree run
	1. For mono: roslaunch `orb_object_slam tum_mono.launch`
	2. For RGB-D: roslaunch `orb_object_slam tum_rgbd.launch`
