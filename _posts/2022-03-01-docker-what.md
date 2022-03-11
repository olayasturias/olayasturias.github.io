---
layout: post
cover: assets/images/underwater-enhancement/helo.PNG
title: I made a docker image. Now what?
date: 2022-02-28 12:00:00 +0545
categories: ROS docker
author: olaya
featured: true
summary: tutorial on starting with docker.
---

First of all, note that your dockerfiles cannot be in the home directory. I don't remember why but I remember this.

# 1. Build it



```
docker build -t "NAME:Dockerfile" .
```



