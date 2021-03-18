#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

# AC: add /usr/lib/web to VM
git clone https://github.com/fcwu/docker-ubuntu-vnc-desktop.git
cd docker-ubuntu-vnc-desktop
git checkout trusty
cp -r image /
cd ~

# built-in packages
sudo apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common curl \
    && sh -c "echo 'deb http://download.opensuse.org/repositories/home:/Horst3180/xUbuntu_16.04/ /' >> /etc/apt/sources.list.d/arc-theme.list" \
    && curl -SL http://download.opensuse.org/repositories/home:Horst3180/xUbuntu_16.04/Release.key | apt-key add - \
    && add-apt-repository ppa:fcwu-tw/ppa \
    && apt-get update \
    && apt-get install -y --no-install-recommends --allow-unauthenticated \
        supervisor \
        openssh-server pwgen sudo vim-tiny \
        net-tools \
        lxde \
        x11vnc \
        xvfb \
        gtk2-engines-murrine ttf-ubuntu-font-family \
        firefox \
        nginx \
        python-pip python-dev build-essential \
        mesa-utils libgl1-mesa-dri \
        gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine pinta arc-theme \
        dbus-x11 x11-utils \
        terminator \
    && apt-get autoclean \
    && apt-get autoremove \
    && rm -rf /var/lib/apt/lists/*

# =================================
# install ros (source: https://github.com/osrf/docker_images/blob/5399f380af0a7735405a4b6a07c6c40b867563bd/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile)
# install packages
sudo apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
sudo apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

sudo apt-get update && apt-get install -y --no-install-recommends \
		cmake build-essential pkg-config \
		libglew-dev libeigen3-dev libomp-dev \
		libboost-dev libboost-thread-dev libboost-filesystem-dev \
		libboost-dev libboost-serialization-dev \
		qt5-default \
		ca-certificates \
		curl htop nano wget git unzip \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

echo "###################"
echo "##### line 63 #####"
echo "###################"

# Pangolin
cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin.git \
	&& cd Pangolin && mkdir build && cd build && cmake .. \
	&& make -j$(nproc) && make install \
	&& cd / && rm -rf /tmp/Pangolin

sudo add-apt-repository multiverse
sudo add-apt-repository universe
sudo apt-get update
sudo apt-cache search openblas
sudo apt-get install libblas-dev liblapack-dev -y
sudo apt-get install python-opencv -y
sudo apt-get install libopencv-dev -y
sudo apt-get install ros-kinetic-pcl-ros -y


# setup environment
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# bootstrap rosdep
rosdep init \
    && rosdep update

# AC: remove not required packages
sudo apt autoremove

echo "###################"
echo "##### line 94 #####"
echo "###################"

# install ros packages
export ROS_DISTRO=kinetic
sudo apt-get update && apt-get install -y ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
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
sudo apt-get update && apt-get install -y \
    terminator \
    gedit \
    okular \
    vim \
    && rm -rf /var/lib/apt/lists/*

# AC: Don't need tini as we are not using Docker...
# tini for subreap
# export TINI_VERSION=v0.9.0
# # replace ADD
# sudo wget -O /bin/tini https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini 
# sudo chmod +x /bin/tini

sudo mkdir /image

# AC: install pip as it is apparently broken...
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
python get-pip.py

pip install setuptools wheel && pip install -r /usr/lib/web/requirements.txt

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

sudo apt-get install python-catkin-tools -y
# build ros package source
sudo catkin config \
      --extend /opt/ros/kinetic

#sudo cd ~/cube_slam_ws
#RUN catkin config -DCMAKE_CXX_STANDARD=11
#RUN pwd
#RUN ls
#RUN source /opt/ros/kinetic/setup.bash
#RUN catkin build -j4

echo "###################"
echo "##### line 186 #####"
echo "###################"

sudo apt-get install python-pip
pip install -U pip
pip install --ignore-installed enum34
pip install --ignore-installed pyasn1-modules
pip install six==1.10.0
pip install tensorflow==1.4.1
pip install keras==2.1.2

pip install h5py==2.7.0
pip install numpy==1.13.3
#pip install rviz
pip install scikit-image==0.13.0
pip install scikit-learn==0.19.1
pip install scipy==0.19.1
pip install ipython

echo "###################"
echo "##### line 205 #####"
echo "###################"

# AC: pip3 support for python3.5 is deprecated...
curl -fsSL -o- https://bootstrap.pypa.io/pip/3.5/get-pip.py | python3.5
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
export HOME=/home/ubuntu \
    SHELL=/bin/bash
sudo bash startup.sh

echo "###################"
echo "###### END ########"
echo "###################"
