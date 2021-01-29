
## AT3DCV

## Quick Start
1. Download this repo: https://github.com/ct2034/docker-ubuntu-vnc-desktop
2. Change the Dockerfile with the Dockerfile uploaded here
3. Open terminal and go to the folder where the Dockerfile is located
4. sudo docker build -t at3dcv_2020 .
5. `docker run -it --rm -p 6080:80 -v ~/at3dcv/:/mnt/ at3dcv_2020:latest`
6. Browse to http://127.0.0.1:6080/
7. In the docker image open Terminal
8. cd cube_slam_ws
9.  `source /opt/ros/kinetic/setup.bash`
10. `catkin_make -j1`
11. `source devel/setup.bash`
12. `roslaunch orb_object_slam object_slam_example.launch > roslaunch.log`

## Start ROS
1. download the `ORBvoc.txt` file from the original ORB Slam 2 Vocabulary folder or from our now updated master branch.
2. copy the file to your Vocabulary folder with in the orb_object_slam folder with in your mounted objectslam folder.
3. download the whole `seq7` data folder from here: https://drive.google.com/drive/folders/1FrBdmYxrrM6XeBe_vIXCuBTfZeCMgApL
4. copy the `seq7` folder to a folder with in your mounted volume.
5. Edit the path of the parameter `base_data_folder` with in the mono.launch file to point to the `seq7` folder
6. Change `ORBvoc.bin` to `ORBvoc.txt` with in the mono.launch file (fifth line).
7. Open two terminals and in all three source the setup fill: `source <path_to_cu  be_slam>/devel/setup.bash`
8. Terminal one run: `roslaunch orb_object_slam xxxx.launch` and wait for two windows to pop up
9. Terminal two run: `rosbag play /mnt/datasets/freiburg3_walking_xyz/rgbd_dataset_freiburg3_walking_xyz.bag --clock -r 0.5 `

#### Architecture for 3D dense map reconstruction - brainstorming session 05.11.2020

![3D dense map architecture](./resources/architecture/architecture_dense_map.png)

# Cube SLAM #
This code contains two mode:
1)  object SLAM integrated with ORB SLAM. See ```orb_object_slam```  Online SLAM with ros bag input. It reads the offline detected 3D object.
2) Basic implementation for Cube only SLAM. See ```object_slam``` Given RGB and 2D object detection, the algorithm detects 3D cuboids from each frame then formulate an object SLAM to optimize both camera pose and cuboid poses.  is main package. ```detect_3d_cuboid``` is the C++ version of single image cuboid detection, corresponding to a [matlab version](https://github.com/shichaoy/matlab_cuboid_detect).

**Authors:** [Shichao Yang](https://shichaoy.github.io./)

**Related Paper:**

* **CubeSLAM: Monocular 3D Object SLAM**, IEEE Transactions on Robotics 2019, S. Yang, S. Scherer  [**PDF**](https://arxiv.org/abs/1806.00557)

If you use the code in your research work, please cite the above paper. Feel free to contact the authors if you have any further questions.



## Installation

### Prerequisites
This code contains several ros packages. We test it in **ROS indigo/kinetic, Ubuntu 14.04/16.04, Opencv 2/3**. Create or use existing a ros workspace.
```bash
mkdir -p ~/cubeslam_ws/src
cd ~/cubeslam_ws/src
catkin_init_workspace
git clone git@github.com:shichaoy/cube_slam.git
cd cube_slam
```

### Compile dependency g2o
```bash
sh install_dependenices.sh
```


### Compile
```bash
cd ~/cubeslam_ws
catkin_make -j4
```


## Running #
```bash
source devel/setup.bash
roslaunch object_slam object_slam_example.launch
```
You will see results in Rviz. Default rviz file is for ros indigo. A kinetic version is also provided.

To run orb-object SLAM in folder ```orb_object_slam```, download [data](https://drive.google.com/open?id=1FrBdmYxrrM6XeBe_vIXCuBTfZeCMgApL). See correct path in ```mono.launch```, then run following in two terminal:
``` bash
roslaunch orb_object_slam mono.launch
rosbag play mono.bag --clock -r 0.5
```

To run dynamic orb-object SLAM mentioned in the paper, download [data](https://drive.google.com/drive/folders/1T2PmK3Xt5Bq9Z7UhV8FythvramqhOo0a?usp=sharing). Similar to above, set correct path in ```mono_dynamic.launch```, then run the launch file with bag file.


If compiling problems met, please refer to ORB_SLAM.


### Notes

1. For the online orb object SLAM, we simply read the offline detected 3D object txt in each image. Many other deep learning based 3D detection can also be used similarly especially in KITTI data.

2. In the launch file (```object_slam_example.launch```), if ```online_detect_mode=false```, it requires the matlab saved cuboid images, cuboid pose txts and camera pose txts.  if ```true```, it reads the 2D object bounding box txt then online detects 3D cuboids poses using C++.

3. ```object_slam/data/``` contains all the preprocessing data. ```depth_imgs/``` is just for visualization. ```pred_3d_obj_overview/``` is the offline matlab cuboid detection images. ```detect_cuboids_saved.txt``` is the offline cuboid poses in local ground frame, in the format "3D position, 1D yaw, 3D scale, score". ```pop_cam_poses_saved.txt``` is the camera poses to generate offline cuboids (camera x/y/yaw = 0, truth camera roll/pitch/height) ```truth_cam_poses.txt``` is mainly used for visulization and comparison.

	```filter_2d_obj_txts/``` is the 2D object bounding box txt. We use Yolo to detect 2D objects. Other similar methods can also be used. ```preprocessing/2D_object_detect``` is our prediction code to save images and txts. Sometimes there might be overlapping box of the same object instance. We need to filter and clean some detections. See the [```filter_match_2d_boxes.m```](https://github.com/shichaoy/matlab_cuboid_detect/blob/master/filter_match_2d_boxes.m) in our matlab detection package.