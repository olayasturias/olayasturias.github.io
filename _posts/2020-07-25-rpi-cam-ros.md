---
layout: post
cover: assets/images/rpi-cam-ros/rpicam.jpg
title: Streaming video from a Raspberry Pi Cam using ROS
date: 2020-07-25 12:00:00 +0545
categories: ros raspberry picam phdstuff
author: olaya
featured: true


---
In this post I'm streaming the video from a Raspberry Pi (model 3B) cam using ROS
(Robot Operating System). We will have ROS configured as a distributed system: that is,
it will be running in both PC (master) and Raspberry Pi (slave).

I'm working under ROS melodic, and I'm supposing that you have it already installed
in your PC ([installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu))
and your Raspberry Pi ([installation instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi))

And now you might be asking yourself:

> But Olaya, what's the point of having ROS running in the RPi? Couldn't you just
> send the stream and have ROS running only on your PC?

Well, that's a very good question, dear reader. Yes, I could just do that.
The point is to be able to easily expand the system!
Maybe in the future I want to have some ROS node running locally in the
Raspberry pi to, let's say, add a motor and make a gimbal. Or turn the Raspberry Pi into
a robot somehow, without depending on the PC.

So, without further ado, let's get into it!


# Setting up master and slave
First of all, both devices need to be on the same local network.
The master device (the PC) is the one that will have the roscore running.
Note that the roscore will run on the PC, working as master, and the camera on the Raspberry, the slave.

## On the PC
Let's start by configuring your PC to act as master.

Running `ifconfig` in your terminal, find the IP of your PC. That IP will be the
value of the `ROS_IP` parameter.

```ini
export ROS_IP="10.42.0.1"
```

And for the `ROS_MASTER_URI`:

```ini
export ROS_MASTER_URI="http://10.42.0.1:11311"
```

Alternatively, you can add them to the .bashrc and keep the values for future sessions:

```ini
echo'export ROS_IP="10.42.0.1"' >> ~/.bashrc
echo'export ROS_MASTER_URI="http://10.42.0.1:11311"' >> ~/.bashrc
```

## On the raspberry

open a terminal and ssh into your raspberry pi as follows:
```ini
ssh pi@{raspberry_ip}
```

Be sure to have ssh (and the camera) enabled on Preferences > Raspberry Pi configuration > Interfaces.

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/rpi-cam-ros/interfaces.PNG)

If you and the terminal are not great friends, you can alternatively use some
VNC viewer (with VNC enabled in your raspberry) to log into
your raspberry and see its screen from your computer.

Now, the `ROS_IP` of the RPi is the raspberry's IP, and the `ROS_MASTER_URI` is the
same PC IP that you specified before:
```ini
export ROS_IP="10.42.0.65"
export ROS_MASTER_URI="http://10.42.0.1:11311"
```
or write it to the .bashrc, just as before:
```ini
echo'export ROS_IP="10.42.0.65"' >> ~/.bashrc
echo'export ROS_MASTER_URI="http://10.42.0.1:11311"' >> ~/.bashrc
```

Now we're ready to put the camera to work.

# Configuring the camera

Again in Preferences > Raspberry Pi configuration > Interfaces,
check that you have the camera enabled. If not, enable it and reboot
your Raspberry Pi.

Now, we will install the required ROS packages to put the camera to work.
I'm using the raspicam node by [Ubiquity Robotics](https://github.com/UbiquityRobotics/raspicam_node).

First, clone it to your `src` directory:

```ini
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```
An additional step this package needs to install the dependencies not recognized by ros is
to create the file `/etc/ros/rosdep/sources.list.d/30-ubiquity.list`:

```ini
sudo nano /etc/ros/rosdep/sources.list.d/30-ubiquity.list
```
And write the following in it:

```ini
yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
```

And run `rosdep update`. Install the dependencies with:

```ini
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```

Now it might have given you an error like this:

> ERROR: The following packages/stacks could not have their rosdep keys resolved to system dependencies:
> raspicam_node: No definition of [camera_info_manager] for OS version [buster]

I solved it by manually cloning the package `image_common` into `src`, and then
compiling everything:

```ini
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
```








# Software setup
