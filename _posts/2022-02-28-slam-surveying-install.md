---
layout: post
cover: assets/images/survey-slam-install/2723025827_a96dcce288_4k.jpg
title: Surveying SLAM algorithms with ROS I - Installation
date: 2022-03-01 12:00:00 +0545
categories: ROS SLAM survey
author: olaya
featured: true
summary: installing slam packages.
---

In this post we will install different state-of-art SLAM algorithms and test that they work in our computer.


# 1. ORB-SLAM3

**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 


## 1.1 Installation

- Download [this fork](https://github.com/olayasturias/ORB_SLAM3) of ORB-SLAM3 in your preferred folder. Install the dependencies indicated in the README. Some notes in the dependencies:
    - If OpenCV gives you a version error, modify the CMakeLists to look for the opencv version that you have installed. For example, if you are using ROS Melodic you will have OpenCV 3.2: `find_package(OpenCV 3.2)`
    - ROS packages for ORB-SLAM3 need rospkg: `sudo apt install python-rospkg`
    - Install Eigen3 (I am using 3.4.0)
        - Some dependencies for eigen (optional):
            - `sudo apt install qt5-default libsparsehash-dev libadolc2 libmpfr-dev fftw-dev`
        - [Download](https://gitlab.com/libeigen/eigen/-/releases/3.4.0) the source code in your preferred folder:
            - `wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz`
        - unzip the file:
            - `tar -zxvf eigen-3.4.0.tar.gz`
        - prepare the directory for building
            ```
            cd eigen-3.4.0
            mkdir build
            cd build
            ```
        - Build it
            ```
            cmake ..
            make
            sudo make install
            ```
        - check which eigen version you have installed with: `pkg-config --modversion eigen3`
    - The rest worked just fine for me following the readme :hugs:


## 1.2 Testing

We will test that the ORB-SLAM scripts work for us with our webcam. 
- Install the required package to publish the webcam image as ROS messages:
    `sudo apt install ros-melodic-usb-cam`
- Start a roscore session:
    - `roscore`
- Run the webcam publisher. Note that we need to remap the default topic from usb_cam because ORB-SLAM3 reads from `/camera/image_raw`, but usb_cam is publishing in `/usb_cam/image_raw`.
    - `rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw`
- Run the monocular version of ORB-SLAM3 as:
    - `rosrun ORB_SLAM3 Mono ORB_SLAM3_PATH/Vocabulary/ORBvoc.txt ORB_SLAM3_PATH/Examples/Monocular/EuRoC.yaml`

If everything is working correctly, you should be seeing something like this:

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/survey-slam-install/orbrun.PNG)

<!---# 2. VINS-Mono

- First, install some ROS dependencies:
    ```
        sudo apt install ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport
    ```
- Build VINS-Mono by running `catkin build vins`. If it gives you an error, try removing your build, devel and logs folders and rebuild it again.-->
[under construction]





