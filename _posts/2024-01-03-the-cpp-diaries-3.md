---
layout: post
cover: assets/images/cpp/cpp2.png
title: Setting a Jetson AGX Orin for robotics development (III) - event-based SLAM
date: 2024-01-03 12:00:00 +0545
categories: phdstuff cpp
author: olaya
featured: true
summary: notebook for cpp project III.
---

Welcome to the second post on setting up a Jetson AGX Orin for robotics development! Today, we will focus on setting up a set of state-of-the-art methods for event-based localization. The structure of the repo goes as follows:

- [1. ESVO-Extension](#1-esvo-extension)
  - [1.1 Prerequisites](#11-prerequisites)
  - [1.2 Installation](#12-installation)


> **Reminder:** For this project I will be using the **JETSON AGX ORIN DEVELOPER KIT**. Some things to take into account about this board:
> - It is an ARM64 architecture.
> - It has Ubuntu 20.04 installed
> - It comes with the Nvidia Jetpack 5.1.2
> - It has ROS Noetic

# 1. ESVO-Extension

## 1.1 Prerequisites

```
sudo apt install libflann-dev
```

- **PCL library** == 1.10. Follow the installation instructions [here](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html).
- **ROS Noetic dependencies**. I will install this in my `ros_noetic_ws`, therefore, all the comands for each package below are preceded by a `cd ros_noetic_ws/src`.
    - **vision_opencv** 
        ```
        git clone https://github.com/ros-perception/vision_opencv.git
        cd vision_opencv && git checkout noetic
        cd ../..
        ```
    - **pcl_msgs**
        ```
        git clone https://github.com/ros-perception/pcl_msgs.git
        cd pcl_msgs && git checkout noetic-devel
        ```
    - **geometry2**
        ```
        git clone https://github.com/ros/geometry2.git
        cd geometry2 && git checkout noetic-devel
        ```
    - **perception_pcl**
        ```
        git clone https://github.com/ros-perception/perception_pcl.git 
        cd perception_pcl && git checkout melodic-devel
        ```
- **Other ROS dependencies**. These are the dependencies that I will install in my `catkin_ws`. Similarly, all the comands for each package below are preceded by a `cd catkin_ws/src`.
    - **cnpy_catkin**
        ```
        git clone https://github.com/uzh-rpg/cnpy_catkin.git
        ```

## 1.2 Installation

`ESVO_extension` is a ROS package that I will intall in my `catkin_ws` workspace.

```
cd catkin_ws/src
git clone https://github.com/olayasturias/ESVO_extension.git
```
For installing it and other dependencies, follow the installation instructions [in the repo](https://github.com/olayasturias/ESVO_extension).