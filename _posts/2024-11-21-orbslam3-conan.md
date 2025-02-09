---
layout: post
cover: assets/images/roscon/rosconattendees.png
title: Compiling ORB-SLAM3 with conan
date: 2024-10-25 12:00:00 +0545
categories: Cpp conan vlsam
author: olaya
featured: true
summary: Compiling ORB-SLAM3 with conan
---


# What is conan
Conan is a C++ package manager that simplifies the management of libraries and dependencies for your projects. It automates tasks such as downloading, building, and managing dependencies, ensuring that all required components are compatible with each other. 

A core component of Conan is the `conanfile.py`, which describes how a package should be built and defines its dependencies, build steps, and configurations. Here's a breakdown:

- **Class name:** Specifies the package name in the Conan package manager. This name is used when referencing the package in other projects.
- **Version:** Defines the version of the package. Ideally, it follows semantic versioning or the conventions defined in the repository.
- **`source` method:** Specifies how to fetch the source code. This method usually works in conjunction with a `conandata.yml` file, which contains URLs or Git repository links to download the source for specific versions.

Conan makes dependency management less error-prone and more consistent, especially when working on large projects with many dependencies.



# What is ORB-SLAM3?

ORB-SLAM3 is a state-of-the-art visual SLAM (Simultaneous Localization and Mapping) system designed for monocular, stereo, and RGB-D cameras. It is capable of tracking camera poses and building 3D maps of the environment in real-time. 

ORB-SLAM3 relies on several third-party libraries, such as Pangolin (for visualization) and Eigen (for linear algebra computations). Managing these dependencies manually can be challenging, especially when compiling the project on different systems or configurations. Using Conan simplifies this process, enabling developers to automate and standardize the build process.


# Step 1: build conan dependencies

To compile ORB-SLAM3, we created a repository called `orbslam3-conan`. This repository contains Conan files describing the dependencies needed for ORB-SLAM3 and their respective configurations.

## Dependencies 

ORB-SLAM3 depends on several libraries, including:

1. **Pangolin Viewer:** A lightweight and portable library for 3D visualization.
2. **Eigen:** A high-performance C++ library for linear algebra operations.

Each dependency may also have its own set of dependencies, which are resolved recursively by Conan.

### Example: Building Pangolin

You can organize your project with a folder for each dependency. For ilsnstance, create a `Pangolin` folder inside your repository. Then, navigate to the folder containing the Conan files for Pangolin and run the following commands:

```bash
conan source .  # or conan source conanfile.py
```
This command invokes the source method defined in the conanfile.py and clones the repository version specified in the file. The source code will be downloaded into a src folder.




You need to specify requirements for each package, by targeting it in the requirements funciton. E.g. Pangolin requires eigen, so [...]. If not installed in your pc, conan will try to locate the package in the conan center.

conan build . -pr:b=your-preferred-profile -pr:h=your-preferred-profile
(b is build and h is for host)