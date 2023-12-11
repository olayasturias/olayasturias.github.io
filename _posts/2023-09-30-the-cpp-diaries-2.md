---
layout: post
# cover: assets/images/icra23/meaticra.jpeg
title: Setting a Jetson AGX Orin for robotics development (I) - Inivation's event camera
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

# 1. Setting up the driver
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