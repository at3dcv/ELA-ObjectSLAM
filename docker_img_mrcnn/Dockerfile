FROM ubuntu:16.04

ENV DEBIAN_FRONTEND noninteractive

# built-in packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common curl \
    && sh -c "echo 'deb http://download.opensuse.org/repositories/home:/Horst3180/xUbuntu_16.04/ /' >> /etc/apt/sources.list.d/arc-theme.list" \
    && curl -SL http://download.opensuse.org/repositories/home:Horst3180/xUbuntu_16.04/Release.key | apt-key add - \
    && add-apt-repository ppa:fcwu-tw/ppa \
    && apt-get update \
    && apt-get install -y --no-install-recommends --allow-unauthenticated \
        supervisor \
        openssh-server pwgen sudo vim-tiny \
        net-tools \
        lxde x11vnc xvfb \
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
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
		cmake build-essential pkg-config \
		libglew-dev libeigen3-dev libomp-dev \
		libboost-dev libboost-thread-dev libboost-filesystem-dev \
		libboost-dev libboost-serialization-dev \
		qt5-default \
		ca-certificates \
		curl htop nano wget git unzip \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin.git \
	&& cd Pangolin && mkdir build && cd build && cmake .. \
	&& make -j$(nproc) && make install \
	&& cd / && rm -rf /tmp/Pangolin

RUN sudo add-apt-repository multiverse
RUN sudo add-apt-repository universe
RUN apt-get update
RUN apt-cache search openblas
RUN apt-get install libblas-dev liblapack-dev -y
RUN apt-get install python-opencv -y
RUN apt-get install libopencv-dev -y
RUN apt-get install ros-kinetic-pcl-ros -y


# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
#    ros-kinetic-ros-core=1.3.1-0* \
    ros-kinetic-desktop-full \
    #              A
    #              +--- full desktop \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
# COPY ./ros_entrypoint.sh /


# =================================

# user tools
RUN apt-get update && apt-get install -y \
    terminator \
    gedit \
    okular \
    vim \
    && rm -rf /var/lib/apt/lists/*

# tini for subreap
ENV TINI_VERSION v0.9.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /bin/tini
RUN chmod +x /bin/tini

ADD image /
RUN pip install setuptools wheel && pip install -r /usr/lib/web/requirements.txt

RUN cp /usr/share/applications/terminator.desktop /root/Desktop
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc


RUN mkdir -p ~/cube_slam_ws/src
RUN cd ~/cube_slam_ws/src
RUN git clone https://github.com/shichaoy/cube_slam.git ~/cube_slam_ws/src/cube_slam
RUN pwd

RUN cd ~/cube_slam_ws/src/cube_slam/object_slam/Thirdparty/g2o \
	&& mkdir build \
	&& cd build \
	&& cmake ~/cube_slam_ws/src/cube_slam/object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release \
	&& make -j2 

RUN cd ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/g2o \
	&& mkdir build \
	&& cd build \
	&& cmake ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/g2o -DCMAKE_BUILD_TYPE=Release \
	&& make -j2 

RUN cd ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/DBoW2 \
	&& mkdir build \
	&& cd build \
	&& cmake ~/cube_slam_ws/src/cube_slam/orb_object_slam/Thirdparty/DBoW2 -DCMAKE_BUILD_TYPE=Release \
	&& make -j2 

# Install ROS dependencies
RUN rosdep update

RUN sudo add-apt-repository multiverse
RUN sudo add-apt-repository universe
RUN apt-get update

RUN apt-get install python-catkin-tools -y
# build ros package source
RUN catkin config \
      --extend /opt/ros/kinetic

#RUN cd ~/cube_slam_ws
#RUN catkin config -DCMAKE_CXX_STANDARD=11
#RUN pwd
#RUN ls
#RUN source /opt/ros/kinetic/setup.bash
#RUN catkin build -j4

RUN apt-get install python-pip
RUN pip install -U pip
RUN pip install --ignore-installed enum34
RUN pip install --ignore-installed pyasn1-modules
RUN pip install tensorflow==1.4.1
RUN pip install keras==2.1.2

RUN pip install h5py==2.7.0
RUN pip install numpy==1.13.3
#RUN pip install rviz
RUN pip install scikit-image==0.13.0
RUN pip install scikit-learn==0.19.1
RUN pip install scipy==0.19.1
RUN pip install ipython



RUN apt-get install -y python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install h5py==2.7.0
RUN pip3 install Keras==2.1.2
RUN pip3 install numpy==1.13.3
#RUN pip3 install rviz==1.12.13
RUN pip3 install opencv-python==3.4.0.12
RUN pip3 install scikit-image==0.13.0
RUN pip3 install scikit-learn==0.19.1
RUN pip3 install scipy==0.19.1
RUN pip3 install tensorflow


EXPOSE 80
WORKDIR /root
ENV HOME=/home/ubuntu \
    SHELL=/bin/bash
ENTRYPOINT ["/startup.sh"]
