---
layout: post
# cover: assets/images/icra23/meaticra.jpeg
title: The CPP diares I
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
- **ROS version**: it is not installed, but it is important to note that only ROS Foxy is supported, which is compatible with C++14.



