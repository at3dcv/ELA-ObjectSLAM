#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

# built-in packages
sudo apt-get update
sudo apt-get -y install -y --no-install-recommends software-properties-common curl
sudo sh -c "echo 'deb http://download.opensuse.org/repositories/home:/Horst3180/xUbuntu_16.04/ /' >> /etc/apt/sources.list.d/arc-theme.list"
sudo curl -SL http://download.opensuse.org/repositories/home:Horst3180/xUbuntu_16.04/Release.key | apt-key add -
sudo add-apt-repository ppa:fcwu-tw/ppa
sudo apt-get update
sudo apt-get -y install psmisc git
sudo apt-get -y install -y --no-install-recommends --allow-unauthenticated
sudo apt-get -y install supervisor
sudo apt-get -y install openssh-server pwgen sudo vim-tiny
sudo apt-get -y install net-tools
sudo apt-get -y install lxde
sudo apt-get -y install x11vnc
sudo apt-get -y install xvfb
sudo apt-get -y install gtk2-engines-murrine ttf-ubuntu-font-family
sudo apt-get -y install firefox
sudo apt-get -y install nginx
sudo apt-get -y install python-pip python-dev build-essential
sudo apt-get -y install mesa-utils libgl1-mesa-dri
sudo apt-get -y install gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine pinta arc-theme
sudo apt-get -y install dbus-x11 x11-utils
sudo apt-get -y install terminator
sudo apt-get autoclean
sudo apt-get autoremove
sudo rm -rf /var/lib/apt/lists/*

# AC: add /usr/lib/web to VM
git clone https://github.com/fcwu/docker-ubuntu-vnc-desktop.git
cd docker-ubuntu-vnc-desktop
git checkout trusty
sudo mkdir /usr/lib/noVNC
sudo mkdir /usr/lib/web
sudo /bin/cp -rf image/usr/lib/web /usr/lib/web
sudo /bin/cp -rf image/usr/lib/noVNC /usr/lib/noVNC
sudo /bin/cp -f image/etc/nginx/sites-enabled/default /etc/nginx/sites-enabled/default
sudo /bin/cp -f image/etc/supervisor/conf.d/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
cd ~

# =================================
# install ros (source: https://github.com/osrf/docker_images/blob/5399f380af0a7735405a4b6a07c6c40b867563bd/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile)
# install packages
sudo apt-get update
sudo apt-get -y install dirmngr
sudo apt-get -y install gnupg2
sudo rm -rf /var/lib/apt/lists/*

# setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
sudo apt-get update
sudo apt-get -y install python-rosdep
sudo apt-get -y install python-rosinstall
sudo apt-get -y install python-vcstools
sudo rm -rf /var/lib/apt/lists/*

sudo apt-get update
sudo apt-get -y install cmake build-essential pkg-config
sudo apt-get -y install libglew-dev libeigen3-dev libomp-dev
sudo apt-get -y install libboost-dev libboost-thread-dev libboost-filesystem-dev
sudo apt-get -y install libboost-dev libboost-serialization-dev
sudo apt-get -y install qt5-default
sudo apt-get -y install ca-certificates
sudo apt-get -y install curl htop nano wget git unzip
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

echo "###################"
echo "##### line 63 #####"
echo "###################"

# Pangolin
cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build && cmake ..
make -j$(nproc) && make install
cd / && rm -rf /tmp/Pangolin

sudo add-apt-repository multiverse
sudo add-apt-repository universe
sudo apt-get update
sudo apt-cache search openblas
sudo apt-get -y install libblas-dev liblapack-dev -y
sudo apt-get -y install python-opencv -y
sudo apt-get -y install libopencv-dev -y
sudo apt-get -y install -f ros-kinetic-pcl-ros


# setup environment
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# bootstrap rosdep
rosdep init
rosdep update

# AC: remove not required packages
sudo apt autoremove

echo "###################"
echo "##### line 94 #####"
echo "###################"

# install ros packages
export ROS_DISTRO=kinetic
sudo apt-get update && apt-get install -y ros-kinetic-desktop-full
sudo rm -rf /var/lib/apt/lists/*
#    ros-kinetic-ros-core=1.3.1-0* \
#              A
#              +--- full desktop \

# setup entrypoint
# COPY ./ros_entrypoint.sh /


# =================================

echo "###################"
echo "##### line 113 #####"
echo "###################"

# user tools
sudo apt-get update
sudo apt-get -y install terminator
sudo apt-get -y install gedit
sudo apt-get -y install okular
sudo apt-get -y install vim
sudo rm -rf /var/lib/apt/lists/*

# AC: Don't need tini as we are not using Docker...
# tini for subreap
# export TINI_VERSION=v0.9.0
# # replace ADD
# sudo wget -O /bin/tini https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini 
# sudo chmod +x /bin/tini

sudo mkdir /image

pip3 install setuptools wheel && pip3 install -r /usr/lib/web/requirements.txt

sudo cp /usr/share/applications/terminator.desktop /root/Desktop
echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

# AC: Will not use shichaoy's cube slam...
# echo "###################"
# echo "##### line 136 #####"
# echo "###################"

# sudo mkdir -p ~/cube_slam_ws/src
# cd ~/cube_slam_ws/src
# sudo git clone https://github.com/shichaoy/cube_slam.git ~/cube_slam_ws/src/cube_slam
# pwd

# cd ~/cube_slam_ws/src/cube_slam/object_slam/Thirdparty/g2o \
# 	&& mkdir build \
# 	&& cd build \
# 	&& cmake ~/cube_slam_ws/src/cube_slam/object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release \
# 	&& make -j2 

# cd ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/g2o \
# 	&& mkdir build \
# 	&& cd build \
# 	&& cmake ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release \
# 	&& make -j2 

# cd ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/DBoW2 \
# 	&& mkdir build \
# 	&& cd build \
# 	&& cmake ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/DBoW2 -DCMAKE_BUILD_TYPE=Release \
# 	&& make -j2 

# Install ROS dependencies
rosdep update

echo "###################"
echo "##### line 167 #####"
echo "###################"

sudo add-apt-repository multiverse
sudo add-apt-repository universe
sudo apt-get update

sudo apt-get -y install python-catkin-tools -y
# build ros package source
sudo catkin config --extend /opt/ros/kinetic

#sudo cd ~/cube_slam_ws
#RUN catkin config -DCMAKE_CXX_STANDARD=11
#RUN pwd
#RUN ls
#RUN source /opt/ros/kinetic/setup.bash
#RUN catkin build -j4

echo "###################"
echo "##### line 186 #####"
echo "###################"

pip3 install --ignore-installed enum34
pip3 install --ignore-installed pyasn1-modules
pip3 install six==1.10.0
pip3 install tensorflow==1.4.1
pip3 install keras==2.1.2

pip3 install h5py==2.7.0
pip3 install numpy==1.13.3
#pip3 install rviz
pip3 install scikit-image==0.13.0
pip3 install scikit-learn==0.19.1
pip3 install scipy==0.19.1
pip3 install ipython

echo "###################"
echo "##### line 205 #####"
echo "###################"

pip3 install --upgrade pip
pip3 install h5py==2.7.0
pip3 install Keras==2.1.2
pip3 install numpy==1.13.3
#pip3 install rviz==1.12.13
pip3 install opencv-python==3.4.0.12
pip3 install scikit-image==0.13.0
pip3 install scikit-learn==0.19.1
pip3 install scipy==0.19.1
pip3 install tensorflow

echo "###################"
echo "##### line 221 #####"
echo "###################"

# AC: I don't know what to do here...
# EXPOSE 80
cd /root
export HOME=/home/ubuntu
export SHELL=/bin/bash
sudo bash startup.sh

echo "###################"
echo "###### END ########"
echo "###################"