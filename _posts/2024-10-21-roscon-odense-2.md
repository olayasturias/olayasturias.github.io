---
layout: post
cover: assets/images/roscon/rosconattendees.png
title: My ROSCON 2024 digest - Days 2 and 3
date: 2024-10-25 12:00:00 +0545
categories: ROS conference roscon
author: olaya
featured: true
summary: My roscon digest on the second and third day
---
- [Simulation and AI Tools](#simulation-and-ai-tools)
  - [Robotec.ai: Simulation and AI tools](#robotecai-simulation-and-ai-tools)
  - [USBL Simulator](#usbl-simulator)
- [Data Management](#data-management)
  - [Segments.ai](#segmentsai)
  - [Roboto.ai](#robotoai)
  - [Foxglove](#foxglove)
- [Communication Protocols and Middleware](#communication-protocols-and-middleware)
  - [Zenoh](#zenoh)
  - [eProsima](#eprosima)
  - [Pixi package manager](#pixi-package-manager)
- [Diagnosis and Performance Monitoring](#diagnosis-and-performance-monitoring)
  - [How is my robot? - On the state of ROS diagnostics](#how-is-my-robot---on-the-state-of-ros-diagnostics)
  - [Scenic: a probabilistic language for world modelling](#scenic-a-probabilistic-language-for-world-modelling)
- [Localization, Navigation, Motion Planning and Collision Avoidance](#localization-navigation-motion-planning-and-collision-avoidance)
  - [Learn Probabilistic Robotics with ROS 2](#learn-probabilistic-robotics-with-ros-2)
  - [Beluga AMCL: a modern Monte Carlo Localization implementation for ROS](#beluga-amcl-a-modern-monte-carlo-localization-implementation-for-ros)
  - [A ROS2 Package for Dynamic Collision Avoidance Based on On-Board Proximity Sensors for Human-Robot Close Interaction](#a-ros2-package-for-dynamic-collision-avoidance-based-on-on-board-proximity-sensors-for-human-robot-close-interaction)
  - [GSplines: Generalized Splines for Motion Optimization and Smoot Collision Avoidance](#gsplines-generalized-splines-for-motion-optimization-and-smoot-collision-avoidance)
  - [Nav2 Docking](#nav2-docking)
  - [Radar Tracks for Path Planning in the presence of Dynamic Obstacles](#radar-tracks-for-path-planning-in-the-presence-of-dynamic-obstacles)
- [Software and Tool Integration](#software-and-tool-integration)
  - [Main Street Autonomy](#main-street-autonomy)
  - [CROSS: FreeCAD and ROS](#cross-freecad-and-ros)
- [Robotics Education and Community Resources](#robotics-education-and-community-resources)
  - [The defenitive guide to ROS Mobile Robotics](#the-defenitive-guide-to-ros-mobile-robotics)
  - [Foss Books](#foss-books)
  - [IEEE Robotics and Automation Practice (RA-P)](#ieee-robotics-and-automation-practice-ra-p)
  - [Awesome conferences and schools list](#awesome-conferences-and-schools-list)
- [Other Interesting tools](#other-interesting-tools)
  - [Happypose](#happypose)


# Simulation and AI Tools

## Robotec.ai: Simulation and AI tools
<!-- Michal Pelka -->
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/robotec_ai.png?raw=true)

[Robotec.ai](https://www.robotec.ai/) introduced us to their simulator. It integrates seamlessly with ROS (though it can function independently as well), utilizing NVIDIA's [Physx engine](https://github.com/NVIDIA-Omniverse/PhysX), and the renderings from [Open 3D Engine](https://o3de.org/). Essentially, Robotec.ai has migrated the ROS stack from Gazebo to O3DE, enhancing both the visual and functional capabilities of simulations.
 
The simulator is **modular**, allowing components like the physics engine to be swapped out based on specific project needs. While optimized for ROS, the simulator’s modular nature allows it to adapt to **various robotic frameworks**, offering flexibility for different use cases.

Robotec.ai provides tailored services to set up simulators and simulation environments suited to the customer's specific requirements.

Later in the conference, a talk from Michal Pelka showed us how easy it is to set up O3DE for simulation:
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/o3desetup.png?raw=true)


## USBL Simulator

![USBL sim](https://github.com/rwth-irt/USBL-Simulator/raw/main/data/icon.svg)

One of my quick notes includes [this repo](https://github.com/rwth-irt/USBL-Simulator) to an USBL simulator. The simulator is written in C++, with a ROS 2 node acting as wrapper for it.

The simulator promisis a high-fidelity simulation, with features including (but not limited to) Round-trip-time (RTT) and Time-Difference-of-Arrival (TDOA) noise and quantization simulation.

The parameters are configured in a YAML file, and the repository includes an example config file for an [OEM USBL by Evologics](https://www.evologics.com/usbl-oem).
![](https://www.evologics.com/web/image/15614/EvoLogics-USBL-OEM-2-600.jpg)


# Data Management


## Segments.ai

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/segmentsai.png?raw=true)

[Segments.ai](https://segments.ai/) is a company located in Belgium. They provide tools for data labelling in 2D and 3D, and an API to export it to whichever framework you use (e.g. Pytorch).
They also provide data labelling service.
Their labels seem to be focused on object detection and segmentation in 2D and 3D (as you might have guessed by the name). Currently, it doesn’t appear that they support localization directly, though I plan to explore this further through their demo offerings.
Segments.ai offers [free academic licenses and demo datasets](https://segments.ai/conferenceinfo).

## Roboto.ai
<!-- Yves Albers -->
[Roboto.ai](https://www.roboto.ai/) provides a platform for curating and analyze robotics data.
![https://docs.roboto.ai/_images/overview.png](https://docs.roboto.ai/_images/overview.png)

With Roboto.ai, you can extract specific data from large datasets, making it especially useful for working with rosbags. Instead of handling one massive file, you can query only the data you need, optimizing memory usage. The platform also allows you to curate datasets based on events or anomalies detected within the data. Roboto's API includes built-in functions for automatic anomaly detection, streamlining data analysis and cleanup. More info on the concepts and functionality can be found [here](https://docs.roboto.ai/learn/concepts.html), or in the [Github repo](https://github.com/roboto-ai/roboto-python-sdk).

## Foxglove
[Foxglove](https://foxglove.dev/) is a well-known tool for visuallizing and debugging data. The data can be streamed from the cloud, local files, or in real time from the robot.
Foxglove also allows replaying the data, [(read here)](https://docs.foxglove.dev/docs/visualization/panels/publish/?_gl=1*7mf2yn*_gcl_au*MTI2MTYzNzUxNi4xNzI5ODQ3ODUz) [(or here)](https://docs.foxglove.dev/api/#tag/Stream-data/paths/~1data~stream/post).
One of Foxglove’s standout features is its ability to display multiple data sources simultaneously, offering flexible, customizable visualizations. For instance, it can look something like this: 

![](https://cdn.prod.website-files.com/66a37d395dfadcdb65dcdf45/66d9c41c7fcb77d973f41888_autonomous_underwater_vehicle-animation.webp)

...but it doesn’t have to—Foxglove is fully customizable! By the way, the sample above showcases [our SubPipe dataset](https://github.com/remaro-network/SubPipe-dataset)! :smiley:
Foxglove has a free plan, and provides free access to students and academics.

# Communication Protocols and Middleware

## Zenoh
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/zenoh.png?raw=true)

[Zenoh](https://zenoh.io/) made a bold entrance at this year's ROSCon with a strong marketing push to position itself as the next standard in robotics middleware, potentially overtaking DDS. 
While choosing between these options requires more than a quick discussion or blog post, if you're undecisive, you might like to know that [Zenoh offers a bridge to ROS 2 DDS](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds). This bridge allows you to integrate Zenoh's capabilities alongside DDS without needing to replace the existing setup, giving you the flexibility to leverage Zenoh’s features while maintaining DDS compatibility.


## eProsima
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/eProsima.png?raw=true)

[eProsima](https://www.eprosima.com/) is well-known for providing the most adopted DDS solution as of today.Although their ROS 2 implementation of Fast DDS has faced criticism for some networking issues, it’s worth noting (as their CEO emphasized to us) that these issues have been addressed in recent updates.

In addition to their popular open-source libraries, eProsima offers commercial solutions, including specialized plugins for low-bandwidth connections, catering to diverse and demanding network requirements.


## Pixi package manager
![](https://github.com/prefix-dev/pixi/assets/4995967/e42739c4-4cd9-49bb-9d0a-45f8088494b5)
Pixi is a package manager that works seamlessly with Ubuntu, MacOS, and Windows. There are examples on setting [Python](https://pixi.sh/latest/tutorials/python/#lets-get-started) (as alternative to conda) and [C++](https://pixi.sh/latest/examples/cpp-sdl/) packages.
Here's [an example package on how to use pixi for a ROS2 workspace](https://github.com/ruben-arts/turtlesim-pixi).

# Diagnosis and Performance Monitoring


## How is my robot? - On the state of ROS diagnostics
<!-- Christian Henkel -->

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/diagnosis.jpeg?raw=true)

Christian Henkel introduced to as a series of ROS packages for diagnosis from the [ROS diagnostics](https://github.com/ros/diagnostics) stack. Aside from that, he presented us a series of good practices that his team at Bosch applies for diagnosis:
- **Diagnosis phylosophy**
  - Main Purpose: Observe the **current state** of the robot.
  - Think of it as a control panel where operators **has all the information** they need.
  - Try to **limit** the metrics to <10, ideally 2-3 per component.
  - **Warnings** are states that are unusual but allow continued operation.
  - **Errors** indicate states that do not allow the robot to operate further and shall be immediately addressed.
  - Think about a logging and diagnostics concept in your team and **document it**.
- **Comparison to other concepts**
  - Diagnostics vs:
    - Logging:
      - Logging is (a lot) more verbose.
      - Captures the inner state of a SW component.
      - Are (usually) for later consumption and analysis.
    - Bagfiles:
      - Are useful to record diagnostics in bagfiles.
      - Will also contain non-critical state info.
    - Testing:
      - Diagnostics help to find more quickly causes for failing tests, but don't replace testing.
      - Crucial diagnostics may be tested themselves.
- **Antipatterns**
  - In general, diagnosis are not meant to be used functionally:
    - The error handling that a robotic system does by itself should not depend on diagnosis.
    - Diagnostics should help a human observer or technician to understand a problem that was not recovered from.
  - The "right" amount of red:
    - Diagnostics must be tuned in a way such that they really mean a problem.
    - Otherwise, human observers get used to seeing error messages and don't recognize critical ones.
    - In a similar theme, warnings should not be too frequent to not become meaningless.
  - Diagnosis must be received:
    - Diagnostics are meant as a communication method from robot to human.
    - So, in fully autonomous systems, they must be logged correctly and evaluated retroactively.
    - It is also worth to difference between roles:
      - For example, if an end user will see and/or understand diagnostics content or
      - if it must only be consumed by the trained technician.

![](https://i0.wp.com/warandpeas.com/wp-content/uploads/2017/03/war-and-peas-insult-machine.jpg?resize=580%2C649&ssl=1)

## Scenic: a probabilistic language for world modelling

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/scenic1.png?raw=true)

[Scenic](https://scenic-lang.org/) is a stochastic scenario generator and a powerful language for defining spatio-temporal relationships in scenes. Whether you're creating environments for CARLA, Gazebo, Unity, or Webots, Scenic can model scenarios as a combination of scene distributions and agent behaviors.

![scenic](https://docs.scenic-lang.org/en/latest/_images/scenic-sim.png)
Why is it useful? Scenic helps test cyber-physical systems against rare edge cases, design environments for exploration, and generate synthetic data for robust machine learning models. While it’s not available for Unreal Engine yet, its versatility makes it a game-changer for simulation and testing.


# Localization, Navigation, Motion Planning and Collision Avoidance

## Learn Probabilistic Robotics with ROS 2

<div align="center">
  <img src="https://github.com/user-attachments/assets/9884b471-ac75-4f76-b1a5-6aea9f8d5945" width="400"/>
</div>

If you work in robotics, it is extremely likely that you've heard about the Probabilistic Robotics book. There are plenty of repositories in the literature implementing such algorithms, however, as a developer you might want to code your own implementation that is adapted to your needs.

However, as addressed by Carlos Argueta, learning these algorithms is challenging, specially when it comes to translating theory into actual code (and making it work in the real world).

Under [this repo](https://github.com/carlos-argueta/rse_prob_robotics), you can find a compilation of open-source implementations of the algorithms under the probabilistic robotics book, accompanied by [an article explaining them](https://soulhackerslabs.com/the-unreasonable-power-of-the-unscented-kalman-filter-with-ros-2-d4c97d4b4bb9).

## Beluga AMCL: a modern Monte Carlo Localization implementation for ROS
<!-- Franco Cipollone -->

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/beluga.png?raw=true)

In this talk, Franco Cipollone first introduced [Beluga](https://ekumen-os.github.io/beluga/index.html), a toolkit for Monte Carlo Localization (MCL) which operates in 2D over laser scan measurements. He mentioned, however, how the ACML algorithms are aging:

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/beluga2.png?raw=true)

Under that premise, he introduced a new library EKUMEN is working on: [the Localization And Mapping BenchmarKINg Toolkit (LAMBKIN)](https://github.com/Ekumen-OS/lambkin)

Lambkin introduces a benchmarking application for localization and mapping systems, in which each component runs in a separate container.
It relies on a series of open-source libraries and benchmarks, including [evo-tools](https://michaelgrupp.github.io/evo/) and [timem](https://timemory.readthedocs.io/en/develop/features.html#command-line-tools).

 


## A ROS2 Package for Dynamic Collision Avoidance Based on On-Board Proximity Sensors for Human-Robot Close Interaction

![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/dynamic_collision_avoidance.png?raw=true)

## GSplines: Generalized Splines for Motion Optimization and Smoot Collision Avoidance
![logo](https://github.com/rafaelrojasmiliani/gsplines_cpp/raw/master/img/logo.svg)

Rafael A. Rojas presented his open-source library to represent and formulate motion and trajectory planning problems with *generalized splines* and *piece-wise polynomials*.
Considering that motion planners have waypoints as output, these points must be joined to form a trajectory. This work presents a library to obtain these trajectories with GSplines.

> "A **generalized spline** is a piece-wise defined curve such that in **each interval** it is the **linear combination** of certain linearly independent functions".

In contrast to ROS classes, the introducied GSplines class contains all the necessary elements to derive polynomial trajectories for a given set of waypoints.
ROS classes define points and trajectories as nested structures. An example can be seen with the  JointTrajectory and JointTrajectoryPoint classess (or messages):

**JointTrajectory:**
``` PYTHON
# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header

# The names of the active joints in each trajectory point. 
string[] joint_names

# Array of trajectory points.
JointTrajectoryPoint[] points
```

**JointTrajectoryPoint:**
``` PYTHON

# Single DOF joint positions for each joint relative to their "0" position.
float64[] positions

# The rate of change in position of each joint. 
float64[] velocities

# Rate of change in velocity of each joint.
float64[] accelerations

# The torque or the force to be applied at each joint.
float64[] effort

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start
```

This array leads to inefficient memory layout and complicates the translation into the flat vector format that optimizers require. **There is no clear interface for evaluation at arbitrary time instances, computation of derivatives, or time scaling.** 

To efficiently derive the trajectories for the given waypoints, the GSplines class represents:
- **Solutions** of the minimum-X trajectory passing through the given waypoints.
- **Representing Transformations:**
  - Derivatives
  - Linear operations
  - Linear scaling
  
That is, the class itself includes the derivative linear operations and linear scaling. The class'goal is to achieve the [monoid property](https://en.wikipedia.org/wiki/Monoid) on the derivative, scaling and addition/scalar multiplication operations. These operations will become a monoid under the composition. The monoid gives nice properties to program.
By implementing the polynomial basis solution of the trajectory, the GSpline class allows implementing optimal control.

For more info visit [the repositorie's Github,](https://github.com/rafaelrojasmiliani/gsplines_cpp?tab=readme-ov-file) which includes the code, theoretical explanations, and link to the associated publications.


## Nav2 Docking
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/nav2.png?raw=true)

Nav2 Docking Server is part of the [Nav2 library](https://github.com/ros-navigation/navigation2), and it has its own [dedicated repo](https://github.com/open-navigation/opennav_docking).
The Nav2 Docking Server can be used with arbitrary robots and docks for auto-docking. The plugins *ChargingDock* and *NonChargingDock* allows to implement parameters specific to your docking station on how to detect if the docking was succesful. The docking procedure seemed to be wrapped around a behavior tree. It can be set up to work within your own behavior tree. The docking steps are:
1. Retrieve dock pose.
2. Navigate to dock (if not within range).
3. Use the docking pluging to detect the dock.
4. Enter a vision-control loop to reach the docking pose.
5. Exit the vision-control loop once the contact has been detected.
6. Wait until chargin starts (if applicable) and return success.

The dock database is an external yaml in server config or in action request. The controller is [nav2_graceful_controller](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html).
Further details on the docking API and dock configurations can be found on the [server's documentation](https://docs.nav2.org/tutorials/docs/using_docking.html).

## Radar Tracks for Path Planning in the presence of Dynamic Obstacles
<!-- Alexander Yuan -->
![radar](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/radarobstacle.png?raw=true)

Obstacles in ROS 2 are typically represented by a stationary point in an occupancy grid, usually inflated by a user-defined radius and cost scaling factor. These points are updated frame-to-frame via ray tracing. A limitation on this approach is the difficulty on using obstacle dynamics.


# Software and Tool Integration

## Main Street Autonomy
<!-- Isaac Dykeman -->
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/mainstreetautonomy.png?raw=true)

The main product advertised by [Main Street Autonomy](https://mainstreetautonomy.com/) was their sensor calibration *"Calibration Anywhere"*. This tool calibrates all lidar, radar, camera, IMU, and GPS/GNSS sensors in a single pass, providing both intrinsic and extrinsic parameters, along with time offsets between sensors—all without the need for checkerboards or targets. It is, definitely, a very interesting product.
What impressed me most, however, was their vSLAM solution, which demonstrated impressively stable features even in challenging scenarios with highly uniform textures, as shown in the photo here.


## CROSS: FreeCAD and ROS
[CROSS](https://github.com/galou/freecad.cross) is a FreeCAD workbench to generate robot description packages (xacro or URDF) for ROS.



# Robotics Education and Community Resources

## The defenitive guide to ROS Mobile Robotics
Steve Macenski introduced to us [one of his last papers](https://www.sciencedirect.com/science/article/abs/pii/S092188902300132X), a survey that answers to the following questions: 
- What are his algorithm choices
- How do they work
- When should I use what and why
- What features exist?
- How do they compare computationally?

## Foss Books

VM (Vicky) Brasseur presented to us her books:
- [Forge Your Future with Open Source](https://pragprog.com/titles/vbopens/forge-your-future-with-open-source/)
- [Business Success with Open Source](https://pragprog.com/titles/vbfoss/business-success-with-open-source/). Some of the sections from this books are available, such as:
  - [Avoid Common FOSS Business Risks](https://media.pragprog.com/titles/vbfoss/risks.pdf?_gl=1*1jiysm4*_ga*MTk5MTQ0NjE0OS4xNzMwNTcxNjM4*_ga_MJ4659LJZC*MTczMDU3MTYzOC4xLjEuMTczMDU3MTg2Mi4wLjAuMA..)
  - [Role of Inbound FOSS in Digital Transformation](https://media.pragprog.com/titles/vbfoss/strategy.pdf?_gl=1*ru2jaq*_ga*MTk5MTQ0NjE0OS4xNzMwNTcxNjM4*_ga_MJ4659LJZC*MTczMDU3MTYzOC4xLjEuMTczMDU3MTg2Mi4wLjAuMA..)
  - [Basic License Compliance](https://media.pragprog.com/titles/vbfoss/compliance.pdf?_gl=1*4765k*_ga*MTk5MTQ0NjE0OS4xNzMwNTcxNjM4*_ga_MJ4659LJZC*MTczMDU3MTYzOC4xLjEuMTczMDU3MTg2Mi4wLjAuMA..)

## IEEE Robotics and Automation Practice (RA-P)
![ieee](https://www.ieee-ras.org/images/publications/RA-P/24-TA-2-001-FP_IEEE_RAP_web_image_996x479_no_button.jpg)

A new IEEE journal with less focus on theoretical contributions and more on applied research. Link to the journal [here](http://www.ieee-ras.org/publications/ra-p)


## Awesome conferences and schools list

A very much needed repository with links to interesting robotics conferences and schools, with a list going as far as 2028! Follow the link [here](https://torydebra.github.io/AwesomeRoboticsConferencesAndSchoolsList/).

# Other Interesting tools

## Happypose
![happypose](https://agimus-project.github.io/happypose/cosypose/images/example_predictions.png)

Krzysztof Wojciechowski presented [happypose_ros](https://github.com/agimus-project/happypose_ros), a ROS 2 wrapper of [Happypose](https://agimus-project.github.io/happypose/) for 6D object pose estimation. Given an RGB image and a 2D bounding box of an object with known 3D model, the 6D pose estimator predicts the full 6D pose of the object with respect to the camera. As a sum up, Happypose introduces a library for:
- Single camera pose estimation.
- Multi camera pose estimation.
- 6D pose estimation of objects in the scene (for the pretrained objects). Easy Fine-tuning with different objects is currently a future work.
- ROS API.
The slides on how the method works can be found [here](https://docs.google.com/presentation/d/1jZDu4mw-uNcwzr5jMFlqEddZsb7SjQozXVG3dT6-1M0/edit#slide=id.g9145acbbc5_0_0)


