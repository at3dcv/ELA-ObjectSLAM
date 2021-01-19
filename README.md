docker-ubuntu-vnc-desktop
=========================

[![Docker Pulls](https://img.shields.io/docker/pulls/ct2034/vnc-ros-kinetic-full.svg)](https://hub.docker.com/r/ct2034/vnc-ros-kinetic-full/)
[![Docker Stars](https://img.shields.io/docker/stars/ct2034/vnc-ros-kinetic-full.svg)](https://hub.docker.com/r/ct2034/vnc-ros-kinetic-full/)

Docker image to provide HTML5 VNC interface to access ROS kinetic on Ubuntu 16.04 with the LXDE desktop environment.

Quick Start
-------------------------

Run the docker image and open port `6080`

```
docker run -it --rm -p 6080:80 ct2034/vnc-ros-kinetic-full
```

Browse http://127.0.0.1:6080/

![screenshot](https://raw.github.com/ct2034/docker-ubuntu-vnc-desktop/master/screenshots/ros-kinetic.png)


Connect with VNC Viewer and protect by VNC Password
------------------

Forward VNC service port 5900 to host by

```
docker run -it --rm -p 6080:80 -p 5900:5900 ct2034/vnc-ros-kinetic-full
```

Now, open the vnc viewer and connect to port 5900. If you would like to protect vnc service by password, set environment variable `VNC_PASSWORD`, for example

```
docker run -it --rm -p 6080:80 -p 5900:5900 -e VNC_PASSWORD=mypassword ct2034/vnc-ros-kinetic-full
```

A prompt will ask password either in the browser or vnc viewer.


Mount directory
---------------

Mount host directory to docker using

```
docker run -v <your directory>:/mnt/ -it --rm -p 6080:80 ct2034/vnc-ros-kinetic-full

```


Troubleshooting and FAQ
==================

1. boot2docker connection issue, https://github.com/fcwu/docker-ubuntu-vnc-desktop/issues/2
2. Screen resolution is fitted to browser's window size when first connecting to the desktop. If you would like to change resolution, you have to re-create the container


License
==================

See the LICENSE file for details.
