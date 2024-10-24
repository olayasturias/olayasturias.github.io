---
layout: post
cover: assets/images/workshop-roscon.png
title: My ROSCON 2024 digest - Day 1
date: 2024-10-21 12:00:00 +0545
categories: cpp lidar ROS
author: olaya
featured: true
summary: My roscon digest on the first day
---

# Workshop 3: Open source, open hardware hand-held mobile mapping system for large scale surveys 

This workshop consisted on deploying an open-hardware hand-held device for recording large scale datasets and perform mapping tasks on them.
The device consistis of a low-cost Lidar sensor with a GPS unit and an IMU sensor. The data is collected by a Raspberry Pi 4 board.
The data processing is performed offline, after transferring the collecting data to our personal PCs.

## The open hardware

The bill of materials for the device can be found [under the Mandeye repository.](https://github.com/JanuszBedkowski/mandeye_controller)
The bill of materials comprises:

- A Rasperry 4 (or 5).
- The Lidar sensor Livox Mid-360. It is a low-cost lidar intended for low-speed robotics. It provides a field of view of 360*59 degrees, with a minimum range of 0.1m and a 40-line point cloud density.
- 3D printed cases for the Lidar and the raspberry Pi.
- An Ulanzi tripod with a built-in battery to power up the unit (and hand-hold the device).
- ... among other materials that you can check out [here](https://github.com/JanuszBedkowski/mandeye_controller/blob/main/doc/BIM.md).

The main selection criteria for the provided materials is to keep the device (relatively) low cost, being low cost in this case less than 1000 euros.

## The open software

The open softwareware is all collected under the [HDMapping project](https://github.com/MapsHD/HDMapping).
Some (quick) interesting notes about this project:
- In Windows, the deployment is quite off-the-shell. You just need to download and execute the binaries, without any installation process.
- It has been developed with the pure objective of collecting ground-truth datasets. To that aim, it allows both manual drift correction, but also algorithmic-based loop closing for data optimization.


# BoF: Accelerating roboyt learning at scale in simulation :hatched_chick:
![alt text](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/bof_nvidia.png)

Aside from the workshops, day 1 also featured a series of "Bird of feather" sessions. Birds of a Feather (BoF) sessions are informal gatherings at a pre-arranged time and place by individuals with a common interest in a topic. 
This session was hosted by Markus Wuensch, head of Robotics Ecosystem in NVIDIA, who proposed the following bullet points:
- Intro
- State of your development
- Products/use cases
- ROS in robot learning
- Tools
- Challenges
- Wishlist
- Future

This, all things said, clearly oriented to get ideas and feedback from the attendants around the usage of NVIDIA hardware and NVIDIA's simulator Isaac Sim. 

![alt text]([image.png](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/bof_nvidia_1.png))

The BoF session was joined by people with very different backgrounds (manipulation, planning, navigation...) with a vast majority of us applying computer vision to these problems. Therefore, most of use use simulators with realistic renderings: Unreal Engine, Unity, or Open3D (more on this one later), were the most mentioned. Gazebo is the primary ROS simulator, which, despite its outstanding capabilities for robot simulation, falls short when it comes to visual renderings. Surprisingly, very few people in the room were using Isaac Sim.
Moreover, most of 