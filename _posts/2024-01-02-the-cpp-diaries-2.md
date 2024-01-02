---
layout: post
cover: assets/images/cpp/cpp2.png
title: Setting a Jetson AGX Orin for robotics development (II) - Inivation's event camera
date: 2024-01-02 12:00:00 +0545
categories: phdstuff cpp
author: olaya
featured: true
summary: notebook for cpp project I.
---

Welcome to the second post on setting up a Jetson AGX Orin for robotics development! Today, we will focus on setting up all the required drivers and dependencies for Inivation's event camera, including the ROS installation and ROS packages. The structure for this post goes as follows:

- [1. Prerequisites](#1-prerequisites)
- [2. Setting up the camera driver](#2-setting-up-the-camera-driver)
- [3. Setting up ROS Noetic](#3-setting-up-ros-noetic)
  - [3.1 Installing bootstrap dependencies](#31-installing-bootstrap-dependencies)
  - [3.2 Initializing rosdep](#32-initializing-rosdep)
  - [3.3 Installation](#33-installation)
    - [3.3.1 Create a catkin workspace](#331-create-a-catkin-workspace)
    - [3.3.2 Resolve dependencies](#332-resolve-dependencies)
    - [3.3.3 Building the catkin workspace](#333-building-the-catkin-workspace)
- [4. Setting up the ROS camera driver](#4-setting-up-the-ros-camera-driver)
  - [4.1 Some extra dependencies](#41-some-extra-dependencies)
  - [4.2 Setting up the workspace](#42-setting-up-the-workspace)
  - [4.3 Cloning and building dv-ros](#43-cloning-and-building-dv-ros)

Once you've gone through all these steps, you will be ready to run the iniVation's event camera on your board.

> **Reminder:** For this project I will be using the **JETSON AGX ORIN DEVELOPER KIT**. Some things to take into account about this board:
> - It is an ARM64 architecture.
> - It has Ubuntu 20.04 installed
> - It comes with the Nvidia Jetpack 5.1.2

# 1. Prerequisites
```
sudo apt install liblz4-dev libzstd-dev libgoogle-perftools-dev ninja-build python3-pip
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
    However, before deciding which version to install, I recommend you to read section [3. Setting up ROS Noetic](#3-setting-up-ros-noetic) in this post.
- **OpenSSL**
    ```
    sudo apt install libssl-dev
    ```
- **Eigen3**. In Ubuntu 20.4, the eigen3 version installed by the apt is 3.3.4. To install the required version 3.4, follow the instructions depicted [here](https://olayasturias.github.io/ros/slam/survey/2022/03/01/slam-surveying-install.html).

- **fmt**. Similarly, the apt manager installs an older version of the fmt library. To install the latest release:
    ```
    wget https://github.com/fmtlib/fmt/releases/download/10.1.0/fmt-10.1.0.zip
    unzip fmt-10.1.0.zip
    cd fmt-10.1.0
    mkdir build && cd build
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
    make
    sudo make install
    ```
- **OpenCV additional modules**. The default installation is missing some OpenCV modules. You can follow the instructions from [my previous post](https://olayasturias.github.io/phdstuff/cpp/2023/06/05/the-cpp-diaries.html) to install all required modules.


- **(Optional) SDL**, with installation instructions [here](https://wiki.libsdl.org/SDL2/Installation).
- **(Optional) Aravis**, with installation instructions [here](https://aravisproject.github.io/aravis/building.html).
    - In Ubuntu 20.04, the meson version installed by the apt is too old for this build. Therefore, you need to install it manually:
        ```
        git clone https://github.com/mesonbuild/meson
        cd meson
        python3 -m pip install -I meson
        python3 -m pip install ninja
        ```
    - When running `ninja install` for installing Aravis, you might get the error `ModuleNotFoundError: no module named 'mesonbuild'`. This is due to the installation path for the meson library. You can fix it by adding it to the `PYTHONPATH`:
        ```
        export PYTHONPATH=$(python3 -m site --user-site)
        ```
        and then, running the ninja install command as:
        ```
        sudo -E ninja install
        ```

# 2. Setting up the camera driver
Inivation provides instructions for the Ubuntu packages (for Ubuntu 20.04 in my case) [on their official website](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html). However, the apt installation did not work: The Orin comes with CuDNN 8.0, which requires OpenCV >= 4.4, and the apt dv-runtime-dev installation is looking for OpenCV 4.2. Therefore, we've installed the libraries from source. For that, we must download the DV repos in our favorite folder (for me, `$HOME/dev`). Let's go one by one:
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

# 3. Setting up ROS Noetic
Fighting demons or being the demon? Yes, we're setting up ROS1 here. It is still the primary OS for event cameras. This section needs a bit of storytelling:
*This is not my first iteration of installing all these libraries in the Orin.* In a previous iteration, I installed all the libraries (OpenCV, ROS, etc.), and the installation of the iniVation drivers would fail. I blamed OpenCV for these issues (don't look at me like that, OpenCV; we've been through this before). But I think now that the problems actually came from ROS and its dependencies!

It turns out that ROS Noetic uses a version of Boost older than the one that the inivation driver needs. Because I installed the iniVation driver first, the compilation went down smoothly. However, now I can't install ROS Noetic with `apt`. We will need to install it from source. Damm! Seems like we're being the demons *and* fighting demons after all.

> Note for future self: **maybe** if instead of Boost 1.83 I would have installed the minimum required version by inivation, that is, Boost 1.76, then **maybe** `sudo apt install ros-noetic-desktop-full` would have worked? It does not seem likely because I tried running that command with `aptitude`, and the only option it gave for Boost was 1.71. But this could be something worth a try.

Now, I will put here the instructions for installing ROS Noetic from source. They're slightly modified with respect to [the original instructions](http://wiki.ros.org/noetic/Installation/Source). For convenience, I will follow the same structure.

## 3.1 Installing bootstrap dependencies

```
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool build-essential
```

## 3.2 Initializing rosdep
```
sudo rosdep init
rosdep update
```
## 3.3 Installation
### 3.3.1 Create a catkin workspace
I will call my workspace `ros_noetic_ws`. You can give it whichever name you want.
```
mkdir ~/ros_noetic_ws
cd ~/ros_noetic_ws
```
Download *all* the ROS packages into the source:
```
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src
```
### 3.3.2 Resolve dependencies
I'm putting this command here, but it gave me an error. It gave me the same error as for the `sudo apt install ros-noetic-desktop-full`: unresolved dependencies. Basically, I had to manually install all the dependencies myself. A bit annoying but not too bad.
```
 rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
```
The dependencies I had to install are:
```
sudo apt install -y libassimp-dev liburdfdom-dev liblog4cxx-dev libgpgme-dev libbz2-dev libgtest-dev sip-dev pyqt5-dev python-sip-dev pyqt5-dev-tools python3-pyqt5 libtinyxml2-dev libtinyxml-dev libpoco-dev  libconsole-bridge-dev python3-empy
```
Another dependency available in `apt` but that needs to be installed manually due to the unresolved dependencies with Boost is `libogre`, with building instructions indicated [here](https://ogrecave.github.io/ogre/api/13/building-ogre.html).
- **pugixml** (required by Ogre):
    ```
    git clone https://github.com/zeux/pugixml.git
    cd pugixml
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr DVR_ENABLE_PROFILER=ON ..
    make && sudo make install
    ```

- **Ogre v1.12.2**, as it is the latest version supported by ROS Noetic. First, install the dependencies:
    ```
    sudo apt install libcppunit-dev
    ```
    Then proceed to the installation itself:
    ```
    git clone https://github.com/OGRECave/ogre.git
    cd ogre
    git checkout v1.12.2  # Replace with the version you need
    ```
    > **NOTE:** If you get compilation issues with Ogre, you can proceed without installing the ROS packages that depend on it. Those are `rviz` and all the packages depending on `rviz`. This means that you will have to run the catkin_make_isolated command with the flag:   `--ignore-pkg rviz rviz_plugin_tutorials rviz_python_tutorial librviz_tutorial rqt_rviz`
- **image_common ROS stack**: for some reason, the image_common stack downloaded by the previous steps does not include all the packages that we will need. You can manually download it into the workspace as:
    ```
    cd ~/ros_noetic_ws/src
    git rm -rf image_common && git clone https://github.com/ros-perception/image_common.git
    cd image_common && git checkout noetic-devel
    ```

### 3.3.3 Building the catkin workspace
This command is slightly different from the original instructions. The difference lies in the flag `-DCMAKE_CXX_FLAGS=-DBOOST_TIMER_ENABLE_DEPRECATED.` Without it, the compilation gives an error due to a deprecated header:
> /usr/include/boost/progress.hpp:23:3: error: #error This header is deprecated and will be removed. (You can define BOOST_TIMER_ENABLE_DEPRECATED to suppress this error.)

What the flag does is ignore the error and resume the compilation:


```
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_FLAGS=-DBOOST_TIMER_ENABLE_DEPRECATED
```

Now, you should have successfully compiled all the ROS Noetic packages! You can source the workspace by adding the following line to the end of the `~/.bashrc`:
```
source ~/ros_catkin_ws/install_isolated/setup.bash
```

# 4. Setting up the ROS camera driver
We finally have all the external libraries ready to run the ROS driver for the iniVation camera. Let's install it!
## 4.1 Some extra dependencies
After following the previous instructions, you might still miss the following dependencies:
```
pip3 install defusedxml netifaces
```
## 4.2 Setting up the workspace
Let's set up a separate workspace for ROS development. Doing so is as simple as creating a folder containing a source workspace and building it with catkin:
```
mkdir -p catkin_ws/src 
cd catkin_ws
catkin build
```

## 4.3 Cloning and building dv-ros
The ROS package that contains the code to run the iniVation camera is dv-ros. The original code is in [this repo](https://gitlab.com/inivation/dv/dv-ros). However, we will use [this fork](https://github.com/olayasturias/dv-ros) of the repo. This is because, as of today (Jan 24), the original code is not compatible with `fmt10` (although that's not the case for the camera drivers).

```
cd catkin_ws/src 
git clone https://github.com/olayasturias/dv-ros
cd ..
catkin build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10
```

If the compilation is successful, test that the driver is working:

```
source devel/setup.bash
roslaunch dv_ros_visualization event_visualization.launch
```