---
layout: post
cover: assets/images/cpp/cpp1.png
title: Setting a Jetson AGX Orin for robotics development (I) - building the basis
date: 2023-06-05 12:00:00 +0545
categories: phdstuff cpp
author: olaya
featured: true
summary: notebook for cpp project I.
---

<!-- # Mucho texto: Table of contents
- [1. Setting up Ubuntu](#day-1-workshops-and-tutorials)  
    - [W.1. Distributed graphs workshop](#w1-distributed-graphs-workshop)
    - [W.2. ICRA 2023 Workshop on Unconventional spatial representations: Opportunities for robotics](#w2-icra-2023-workshop-on-unconventional-spatial-representations-opportunities-for-robotics)
- [Day 3. Orals and Posters](#day-3-orals-and-posters) -->

For this project I will be using the **JETSON AGX ORIN DEVELOPER KIT**. Some things to take into account about this board:
- It is an ARM64 architecture.

# 1. Setting up Ubuntu
Setting up Ubuntu in the Jetson is quite straightforward - or at least, similar to what you do in a normal PC. Here are some of the steps that showed up in the process:

### APP Partition size
Just leave the default, which is the maximum accepted size: 59342 MB
### Select Nvpmodel Mode
What amount of power you want to use
MAXN - (Default) which corresponds to maximum
### Install Chromium Browser
I selected yes but it gave the error "Cannot install chromium snap", so let's skip it for now.

Another thing we will want to do is to ssh into our board. Good news: it's already set up and enabled in the board!

# 2. Libraries
Let's start by checking which library version the board has by default:
- **Compilers**
    - **GCC compiler**: GCC is the GNU compiler. It includes a compilation of compilers, including...
    - **G++ compiler**, which is specifically designed for compiling C++ source code. It is not installed by default, can be installed as:
    ```
    sudo apt update
    sudo apt install g++
    ```
    The version installed should be the same as for gcc:
    ```
    gcc -v
    g++ -v
    ```
    Which for me is:
    ```
    gcc (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
    g++ (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
    ```
- **C++ version**. The compilers are compatible with several C++ versions, which are usually pointed during compilation using the flag `-std=c++14`, `-std=c++17`, or `-std=c++20`. The c++ version used will be conditioned by the ROS version is compatible with NVidia's setup.
- **Debugger**: to install the gdb debugger run:
    ```
    sudo apt install build-essential gdb
    ```
- **Visual Studio Code**: I will install whichever last version they have (not so important), but taking into account that the binary should be for a arm64 architecture. I followed the setup instructions [here](https://code.visualstudio.com/docs/cpp/config-linux#_prerequisites).
- **Follow NVIDIA's quickstart** under [this link](https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit).
- **ROS version**: it is not installed, but it is important to note that only ROS Foxy is supported, which is compatible with C++14. You can follow the official installation instructions [here](https://docs.ros.org/en/foxy/Installation.html).
- **CUDA**. The latest CUDA version compatible with the Jetson Orin must be 11.4, as pointed out in [this forum](https://forums.developer.nvidia.com/t/cuda-is-not-installed-on-jetson-orin/220661). If you installed the nvidia-jetpack it should be as simple as:
    ```
    sudo apt update
    sudo apt install cuda-toolkit-11-4
    ```
    And then open your .bashrc file and add the CUDA installation directory to your path as:
    ```
    export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64{LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    ```
- **OpenCV (GPU accelerated)**. Actually, the nvidia-jetpack already installs openCV, although not with CUDA. You can check so with the following command:
    ```
    dpkg -l | grep libopencv
    ```
    I followed the instructions introduced in [this video](https://www.youtube.com/watch?v=art0-99fFa8) pointing to [this installation file](https://github.com/mdegans/nano_build_opencv/blob/master/build_opencv.sh). Some things to note:

    - `-D CUDA_ARCH_BIN=8.7` for Orin.
    - `-D CUDNN_VERSION='8.0'` for the CUDA version used here. Funnily enough, OpenCV would not find my CuDNN, which should have been automatically installed with the nvidia-jetpack. I ran again `sudo apt install nvidia-jetpack`, and now it installed some remaining packages (including CuDNN). 
    - `-D BUILD_opencv_python2=OFF` because I'm not interested in using python 2.
    
    Once you've set up everything you can install OpenCV automatically as `./build_opencv.sh 4.5.4`.


**Next:** setting up the driver for the event-camera DVXplorer Mini!