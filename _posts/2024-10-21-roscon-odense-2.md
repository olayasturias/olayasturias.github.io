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
- [Stands](#stands)
  - [Robotec.ai: Simulation and AI tools](#robotecai-simulation-and-ai-tools)
  - [Zenoh](#zenoh)
  - [eProsima](#eprosima)
  - [Main Street Autonomy](#main-street-autonomy)
  - [Segments.ai](#segmentsai)
  - [Roboto.ai](#robotoai)
  - [Foxglove](#foxglove)
- [Talks](#talks)
    - [A ROS2 Package for Dynamic Collision Avoidance Based on On-Board Proximity Sensors for Human-Robot Close Interaction](#a-ros2-package-for-dynamic-collision-avoidance-based-on-on-board-proximity-sensors-for-human-robot-close-interaction)
  - [GSplines: Generalized Splines for Motion Optimization and Smoot Collision Avoidance](#gsplines-generalized-splines-for-motion-optimization-and-smoot-collision-avoidance)
- [Other stuff](#other-stuff)


# Stands

## Robotec.ai: Simulation and AI tools
<!-- Michal Pelka -->
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/robotec_ai.png?raw=true)

[Robotec.ai](https://www.robotec.ai/) introduced us to their simulator. It integrates seamlessly with ROS (though it can function independently as well), utilizing NVIDIA's [Physx engine](https://github.com/NVIDIA-Omniverse/PhysX), and the renderings from [Open 3D Engine](https://o3de.org/). Essentially, Robotec.ai has migrated the ROS stack from Gazebo to O3DE, enhancing both the visual and functional capabilities of simulations.
 
The simulator is **modular**, allowing components like the physics engine to be swapped out based on specific project needs. While optimized for ROS, the simulator’s modular nature allows it to adapt to **various robotic frameworks**, offering flexibility for different use cases.

Robotec.ai provides tailored services to set up simulators and simulation environments suited to the customer's specific requirements.


## Zenoh
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/zenoh.png?raw=true)

[Zenoh](https://zenoh.io/) made a bold entrance at this year's ROSCon with a strong marketing push to position itself as the next standard in robotics middleware, potentially overtaking DDS. 
While choosing between these options requires more than a quick discussion or blog post, if you're undecisive, you might like to know that [Zenoh offers a bridge to ROS 2 DDS](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds). This bridge allows you to integrate Zenoh's capabilities alongside DDS without needing to replace the existing setup, giving you the flexibility to leverage Zenoh’s features while maintaining DDS compatibility.


## eProsima
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/eProsima.png?raw=true)

[eProsima](https://www.eprosima.com/) is well-known for providing the most adopted DDS solution as of today.Although their ROS 2 implementation of Fast DDS has faced criticism for some networking issues, it’s worth noting (as their CEO emphasized to us) that these issues have been addressed in recent updates.

In addition to their popular open-source libraries, eProsima offers commercial solutions, including specialized plugins for low-bandwidth connections, catering to diverse and demanding network requirements.

## Main Street Autonomy
<!-- Isaac Dykeman -->
![alt text](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/roscon/mainstreetautonomy.png?raw=true)

The main product advertised by [Main Street Autonomy](https://mainstreetautonomy.com/) was their sensor calibration *"Calibration Anywhere"*. This tool calibrates all lidar, radar, camera, IMU, and GPS/GNSS sensors in a single pass, providing both intrinsic and extrinsic parameters, along with time offsets between sensors—all without the need for checkerboards or targets. It is, definitely, a very interesting product.
What impressed me most, however, was their vSLAM solution, which demonstrated impressively stable features even in challenging scenarios with highly uniform textures, as shown in the photo here.

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

# Talks

### A ROS2 Package for Dynamic Collision Avoidance Based on On-Board Proximity Sensors for Human-Robot Close Interaction

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


# Other stuff

http://www.ieee-ras.org/publications/ra-p

https://github.com/rwth-irt/USBL-Simulator
