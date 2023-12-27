---
layout: post
cover: assets/images/cpp/cpp2.png
title: Setting a Jetson AGX Orin for robotics development (II) - Inivation's event camera
date: 2023-12-11 12:00:00 +0545
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
- It has Ubuntu 20.04 installed
- It comes with the Nvidia Jetpack 5.1.2

# 1. Prerequisites
```
sudo apt install liblz4-dev libzstd-dev libgoogle-perftools-dev meson ninja-build python3-pip
```
- **Compilers**
    - **GCC and G++ compiler** should be version 10.0 as required by the inivation driver. Check which version you have set up by default as:
        ```
        gcc -v
        g++ -v
        ```
        For me it is:
        ```
        gcc (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
        g++ (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
        ```
        Therefore, we first need to install gcc and g++ 10.0:
        ```
        sudo apt install gcc-10 g++-10
        ```
        And then set it as the default compiler:
        ```
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10
        sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
        ```
- **Boost library** >= 1.76. Check which version you have installed as:
    ```
    apt list --installed | grep libboost
    ```
    If your version is less than 1.76, you can install a newer version as follows:
    ```
    sudo add-apt-repository ppa:mhier/libboost-latest
    sudo apt update
    ```
    And then (e.g. for version 1.83):
    ```
    sudo apt install libboost1.83-all-dev
    ```
- **OpenSSL**
    ```
    sudo apt install libssl-dev
    ```
- **Eigen3**. In Ubuntu 20.4, the eigen3 version installed by the apt is 3.3.4. To install the required version 3.4, follow the instructions depicted [here](https://olayasturias.github.io/ros/slam/survey/2022/03/01/slam-surveying-install.html).

- **fmt**. Similarly, the apt manager installs an older version of the fmt library. To install the lastest release:
    ```
    wget https://github.com/fmtlib/fmt/releases/download/10.1.0/fmt-10.1.0.zip
    unzip fmt-10.1.0.zip
    cd fmt-10.1.0
    mkdir build && cd build
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
    make
    sudo make install
    ```
- **(Optional) SDL**, with installation instructions [here](https://wiki.libsdl.org/SDL2/Installation).
- **(Optional) Aravis**, with installation instructions [here](https://aravisproject.github.io/aravis/building.html).
    - In Ubuntu 20.04, the meson version installed by the apt is too old for this build. Therefore, you need to install it manually:
        ```
        git clone https://github.com/mesonbuild/meson
        cd meson
        python3 -m pip install -I meson
        python3 -m pip install ninja
        ```

- **OpenCV additional modules**. The default installation is missing some OpenCV modules. You can follow the instructions from [my previous post](https://olayasturias.github.io/phdstuff/cpp/2023/06/05/the-cpp-diaries.html) to install all required modules.


# 2. Setting up the driver
Inivation provides instructions for the Ubuntu packages (for Ubuntu 20.04 in my case) [in their official website](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html). However, the apt installation did not work: The Orin comes with CuDNN 8.0, which requires OpenCV >= 4.4, and the apt dv-runtime-dev installation is looking for OpenCV 4.2. Therefore, we've installed the libraries from source. For that, we will need to download the DV repos in our favourite folder (for me, `$HOME/dev`). Let's go one by one:
- [libcaer](https://gitlab.com/inivation/dv/libcaer):
    ```
    git clone https://gitlab.com/inivation/dv/libcaer
    cd libcaer
    cmake -DCMAKE_INSTALL_PREFIX=/usr .
    make
    sudo make install
    ```
- [dv-processing](https://gitlab.com/inivation/dv/dv-processing):
    ```
    git clone https://gitlab.com/inivation/dv/dv-processing
    cd dv-processing
    cmake -DCMAKE_INSTALL_PREFIX=/usr .
    make
    sudo make install
    ```
- [dv-runtime](https://gitlab.com/inivation/dv/dv-runtime):
    ```
    git clone https://gitlab.com/inivation/dv/dv-runtime
    cd dv-runtime
    cmake -DCMAKE_INSTALL_PREFIX=/usr DVR_ENABLE_PROFILER=ON .
    make
    sudo make install
    ```