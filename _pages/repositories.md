---
layout: page
permalink: /repositories/
title: repositories
description: My GitHub repositories, grouped by theme. Cards link to the repos; some are my own projects, others are forks I use or reference.
nav: true
nav_order: 4
---

{% if site.data.repositories.github_users %}

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for user in site.data.repositories.github_users %}
    {% include repository/repo_user.liquid username=user %}
  {% endfor %}
</div>

---

{% endif %}

{% assign slam = "monocular_visual_slam_survey,robot_visual_localization,gtsam-python-tutorials,SLAM-course,awesome_slam_dataset_downloaders,dv-ros,icra2026,ORB_SLAM3,DROID-SLAM,dso,DEVO,MAC-VO,TrianFlow,DF-VO,tartanvo,relpose-gnn,ESVO_extension,orb_slam3_ros_wrapper,dso_ros,rpg_dvs_ros" | split: "," %}
{% assign underwater = "sonar_obstacle_avoidance,uw_img_sim,remaro_uw_sim,rov_control_uwsim,buoyager,tritech_profiler,valeport_altimeter,uwsim_profiler,bluerov_ros2_driver,bluerov2-setup-checklist,uuv_simulator,underwater_simulation,rexrov2,remaro_worlds,bluerov_ros_playground" | split: "," %}
{% assign ros = "imu_filtering,imcpy_ros_bridge,stams_gui,turtle_bar,gazebo_basics_tutorial,ROSSkill,Dockerfiles,imc_ros_bridge,AirSim,rrt_navigation,rotors_simulator,turtlebot3_simulations,hackathon_oslo" | split: "," %}
{% assign ml = "Plate-Recognition,chatbot_tutorial,HMM-course,equiadapt,gabor_data,ACE-Step" | split: "," %}
{% assign misc = "olayasturias.github.io,olayasturias,epoch-time-display,chonizador" | split: "," %}

## Visual SLAM & Visual Odometry

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for r in slam %}{% assign repo = r | prepend: "olayasturias/" %}{% include repository/repo.liquid repository=repo %}{% endfor %}
</div>

## Underwater Robotics & Simulation

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for r in underwater %}{% assign repo = r | prepend: "olayasturias/" %}{% include repository/repo.liquid repository=repo %}{% endfor %}
</div>

## Robotics & ROS Tooling

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for r in ros %}{% assign repo = r | prepend: "olayasturias/" %}{% include repository/repo.liquid repository=repo %}{% endfor %}
</div>

## Machine Learning & Computer Vision

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for r in ml %}{% assign repo = r | prepend: "olayasturias/" %}{% include repository/repo.liquid repository=repo %}{% endfor %}
</div>

## Web & Miscellaneous

<div class="repositories d-flex flex-wrap flex-md-row flex-column justify-content-between align-items-center">
  {% for r in misc %}{% assign repo = r | prepend: "olayasturias/" %}{% include repository/repo.liquid repository=repo %}{% endfor %}
</div>
