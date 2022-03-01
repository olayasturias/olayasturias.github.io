---
layout: post
cover: assets/images/rostrioska.png
title: creating ROS metapackages
date: 2022-02-28 12:00:00 +0545
categories: ROS
author: olaya
featured: true
summary: concepts and creation of ROS metapackages.
---


This post will make sense to you if you have already familiarized yourself with ROS packages.

As you may know, one of the main strengths of ROS is the possibility of making modular software. By creating specialized packages, you can easily reuse them for other projects. However, as your project gets bigger, so will get your source folder, making it messy.
What's more, if you have a set of packages you are currently working on, compiling them separately might get painful.

![](https://media.giphy.com/media/uWzDsAsRm2X9qULHLs/giphy.gif)

ROS has a solution for both problems: **metapackages** (referred to as **stacks** in older versions of ROS).
A metapackage is a special kind of package meant to gather a subset of packages in it. It allows you to organize your packages within a single folder and compile all packages inside at the same time.

Creating metapackages is straightforward, but I haven't found much documentation, so here is a simple tutorial on creating your own metapackages!

First of all, you need to know what the structure of a stack of metapackages looks like:

```
metapackage
│
└───package1
│   │   CMakeLists.txt
│   │   package.xml
│   └─── ...
│       │   ...
│   
└───package2
│   │   CMakeLists.txt
│   │   package.xml
│   └─── ...
│       │   ...
│ 
└───metapackage
    │   CMakeLists.txt
    │   package.xml
```

That is, you will have a folder with the same name as your metapackage, containing the metapackage itself and the other packages that you want to gather in it.
In this example, you will create a folder called metapackage in your source folder, and then move inside your packages.

First, create the folder that will contain your metapackage and `cd` into it:
```
mkdir metapackage && cd metapackage
```
Next, create a package like this:
```
catkin_create_pkg metapackage --meta
```
The `--meta` flag indicates to catkin that you are creating a metapackage, and thus it creates a CMakeLists.txt and a package.xml accordingly.
Let's take a look to what's inside those files:
```
cmake_minimum_required(VERSION 2.8.3)
project(metapackage)
find_package(catkin REQUIRED)
catkin_metapackage()
```
The *CMakeLists* of a metapackage is quite empty; we only have the *catkin_metapackage()* directive. We won't need to modify it.
On the other hand, we will need to modify the *package.xml* to indicate which packages are included within the metapackage. We can do so by writing:
```
    <exec_depend>package1</exec_depend>
    <exec_depend>package2</exec_depend>
```
Just with that, your metapackage is ready to run. You can build it, and it will build all the packages within it at the same time:
```
catkin build metapackage
```
And that's it! Now you can have your workspace perfectly organized :smile:

![](https://media.giphy.com/media/mBCTckh8N3YKfrNDqm/giphy.gif)


