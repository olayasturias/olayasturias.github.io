---
layout: post
cover: assets/images/workshop-roscon.png
title: My ROSCON 2024 digest - Day 1
date: 2024-10-21 12:00:00 +0545
categories: cpp lidar ROS conference roscon
author: olaya
featured: true
summary: My roscon digest on the first day
---

# Workshop 3: Open source, open hardware hand-held mobile mapping system for large scale surveys 

This workshop consisted of deploying an open-hardware hand-held device to record large-scale datasets and perform mapping tasks on them.
The device comprises a low-cost Lidar sensor with a GPS unit and an IMU sensor. A Raspberry Pi 4 board collects the data.
Data processing is performed offline after the collected data is transferred to our PCs.

## The open hardware

The bill of materials for the device can be found [under the Mandeye repository.](https://github.com/JanuszBedkowski/mandeye_controller)
The bill of materials comprises:

- A Raspberry 4 (or 5).
- The Lidar sensor Livox Mid-360. It is a low-cost lidar intended for low-speed robotics. It provides a field of view of 360*59 degrees, with a minimum range of 0.1m and a 40-line point cloud density.
- 3D printed cases for the Lidar and the Raspberry Pi.
- A Ulanzi tripod with a built-in battery is used to power up the unit (and hand-hold the device).
- ... among other materials you can check out [here](https://github.com/JanuszBedkowski/mandeye_controller/blob/main/doc/BIM.md).

The main selection criteria for the provided materials is to keep the device (relatively) low cost, in this case less than 1000 euros.

## The open software

The open softwareware is all collected under the [HDMapping project](https://github.com/MapsHD/HDMapping).
Some (quick) interesting notes about this project:
- In Windows, the deployment is quite off-the-shell. You just need to download and execute the binaries without any installation process.
- It has been developed with the pure objective of collecting ground-truth datasets. To that aim, it allows manual drift correction and algorithmic-based loop closing for data optimization.


# BoF: Accelerating robot learning at scale in simulation :hatched_chick:
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/bof_nvidia.png?raw=true)

Aside from the workshops, day 1 also featured a series of "Bird of Feather" sessions. Birds of a Feather (BoF) sessions are informal gatherings at a pre-arranged time and place by individuals with a common interest in a topic. 
Markus Wuensch, head of the Robotics Ecosystem at NVIDIA, hosted this session and proposed the following bullet points:
- Intro
- State of your development
- Products/use cases
- ROS in robot learning
- Tools
- Challenges
- Wishlist
- Future

All things said, this was oriented to get ideas and feedback from the attendants about using NVIDIA hardware for simulators in general, and NVIDIA's simulator Isaac Sim in particular.

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/bof_nvidia_1.png?raw=true)

The BoF session was joined by people with very different backgrounds (manipulation, planning, navigation...) with a vast majority of us applying computer vision to these problems. Therefore, most use simulators with realistic renderings: Unreal Engine, Unity, or Open3D (more on this one later), were the most mentioned. Gazebo is the primary ROS simulator, which, despite its outstanding capabilities for robot simulation, falls short when it comes to visual renderings. Surprisingly, very few people in the room were using Isaac Sim. People commented on the high computational requirements and the difficulty of deploying it. People in underwater robotics commented on the need for underwater simulation :wink:

Most people utilize simulators for training models that will be deployed in real robots later. Markus concurred on this and mentioned that NVIDIA aims for what they call the "three body computation", divided into train, simulation, and run. Their aimed solution consists of training in simulation and deploying the trained models at runtime, which is the aim for most of the attendees. 

We discussed **limitations in simulators**. Beyond the limitations that I mentions earlier, when it comes to computer vision, **the repetition of texture patterns** can be troublesome. If, for example, you're using those texture renderings in a visual localization algorithms, the detection of those repeated patterns can mislead the localization estimate. One of the attendees mentioned the use of **procedural generation of textures** to address this problem.

My main take-home message from this discussion is a confirmation of the lack of consensus regarding simulation. Most of us take whichever simulation in the literature looks more convenient and adapt it to our specific needs. For this purpose, the preferred simulators are open-source and have a relatively simple API (which is not the case for Isaac Sim).

This raises a few questions: does having a simulator that fulfills everyone's needs make sense? If so, how could that be possible? Is there any billionaire in the room willing to fund such an open-source initiative?