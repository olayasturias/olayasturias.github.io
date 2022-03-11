---
layout: post
cover: assets/images/survey-slam-install/2723025827_a96dcce288_4k.jpg
title: Surveying SLAM algorithms with ROS (I) - Installation
date: 2022-03-01 12:00:00 +0545
categories: ROS SLAM survey
author: olaya
featured: true
summary: installing slam packages.
---

In this post we will install different state-of-art SLAM algorithms and test that they work in our computer.


# 1. ORB-SLAM3

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 


## 1.1 Installation

- Download [this fork](https://github.com/olayasturias/ORB_SLAM3) of ORB-SLAM3 in your preferred folder. Install the dependencies indicated in the README. Some notes in the dependencies:
    - If OpenCV gives you a version error, modify the CMakeLists to look for the opencv version that you have installed. For example, if you are using ROS Melodic you will have OpenCV 3.2: `find_package(OpenCV 3.2)`
    - ROS packages for ORB-SLAM3 need rospkg: 
        ```
        sudo apt install python-rospkg
        ```
    - Install Eigen3 (I am using 3.4.0)
        - Some dependencies for eigen (optional):
            ```
            sudo apt install qt5-default libsparsehash-dev libadolc2 libmpfr-dev fftw-dev
            ```
        - [Download](https://gitlab.com/libeigen/eigen/-/releases/3.4.0) the source code in your preferred folder:
            ```
            wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
            ```
        - unzip the file:
            ```
            tar -zxvf eigen-3.4.0.tar.gz
            ```
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
        - check which eigen version you have installed with: 
            ```
            pkg-config --modversion eigen3
            ```
    - The rest worked just fine for me by following the readme :hugs:


## 1.2 Testing

We will test that the ORB-SLAM scripts work for us with our webcam. 
- Install the required package to publish the webcam image as ROS messages:
    ```
    sudo apt install ros-melodic-usb-cam
    ```
- Start a roscore session:
    ```
    roscore
    ```
- Run the webcam publisher. Note that we need to remap the default topic from usb_cam because ORB-SLAM3 reads from `/camera/image_raw`, but usb_cam is publishing in `/usb_cam/image_raw`.
    ```
    rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw _pixel_format:="yuyv"
    ```
- Run the monocular version of ORB-SLAM3 as:
    ```
    rosrun ORB_SLAM3 Mono ORB_SLAM3_PATH/Vocabulary/ORBvoc.txt ORB_SLAM3_PATH/Examples/Monocular/EuRoC.yaml
    ```

If everything is working correctly, you should be seeing something like this:

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/survey-slam-install/orbrun.png)

<!---# 2. VINS-Mono

- First, install some ROS dependencies:
    ```
        sudo apt install ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport
    ```
- Build VINS-Mono by running `catkin build vins`. If it gives you an error, try removing your build, devel and logs folders and rebuild it again.-->

# 2. DSO

## 2.1 Installation

DSO has a ROS wrapper, but it first requires that you install DSO:

- Install DSO. Follow the instructions from [the readme in DSO's repository](https://github.com/JakobEngel/dso).
    - If you get an OpenCV error, modify the CMakeLists to point to your version (mine is 3.2) `find_package(OpenCV 3 QUIET)`
    - This will compile a library libdso.a, which can be linked from external projects. It will also build a binary dso_dataset, to run DSO on datasets. It is stored in /build/lib.
- Install ROS wrapper. Download [the source code](https://github.com/olayasturias/dso_ros).
    - It will look for your lib file in ${DSO_PATH}/build/lib. We need to specify our DSO_PATH, which is the path to your dso repository folder:
        ```
            export DSO_PATH=YOUR_PATH/dso
        ```    
    you chan check if it was correctly set with
        ```
            echo $DSO_PATH
        ``` 
    - build it with:
        ```
            catkin build dso_ros
        ``` 


## 2.2 Testing

We will test that the dso_live scripts work for us with our webcam. 
- If you didn't do it before, install the required package to publish the webcam image as ROS messages:
    ```
    sudo apt install ros-melodic-usb-cam
    ```
- Start a roscore session:
    ```
    roscore
    ```
- Run the webcam publisher. Note that we need to remap the default topic from usb_cam because ORB-SLAM3 reads from `/camera/image_raw`, but usb_cam is publishing in `/usb_cam/image_raw`.
    ```
    rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw _pixel_format:="yuyv"
    ```
- Run dso:
    ```
    rosrun dso_ros dso_live image:=image_raw \
		calib=XXXXX/camera.txt \
		gamma=XXXXX/pcalib.txt \
		vignette=XXXXX/vignette.png \

    ```

    - Note that you need to provide **at least** a camera.txt file. It has the format:

            fx fy cx cy k1 k2 r1 r2
            in_width in_height
            "crop"/"full"/"fx fy cx cy 0"
            out_width out_height
    
    - An example of a camera.txt could be like this:

            Pinhole 458.654 457.296 367.215 248.375 0
            640 480
            crop
            640 480
            0.110074


Now if everything works as it should, you should be seeing something like this:

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/survey-slam-install/dso_ros.png)

# 3. RDS-SLAM
## 3.1 Installation
- You will need to install some requirements:
    - Caffe. Follow the instructions [here](https://atinesh.medium.com/caffe-installation-on-ubuntu-18-04-lts-python-2-7-8e8c388ce51f)
    
        - If you get errors with hdf5
            ```
            cd /usr/lib/x86_64-linux-gnu
            sudo ln -s libhdf5_serial.so.100.0.1 libhdf5.so # or whichever version you have
            sudo ln -s libhdf5_serial_hl.so.100.0.0 libhdf5_hl.so # same applies here
            ```
        - If you get errors with Opencv
- Clone the RDS-SLAM repository in your `/catkin_ws/src`
    ```
    git clone https://github.com/yubaoliu/RDS-SLAM

    ```
- cd into the SLAM folder and build with the following script:
    ```
    cd SLAM/
    ./build_thridparty.sh
    ```
- go back to your `/catkin_ws/src` and build rds_slam:
    ```
    cd ~/catkin_ws
    catkin build rds_slam
    ```
    
## 3.2 Testing

If everything is working correctly, you should be seeing something like this:

[under construction]





