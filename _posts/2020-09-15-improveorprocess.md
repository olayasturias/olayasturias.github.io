---
layout: post
cover: assets/images/underwater-enhancement/helo.PNG
title: To enhance or not to enhance? Computer Vision in challenging imaging conditions
date: 2020-05-28 12:00:00 +0545
categories: deep learning dataset database
author: olaya
featured: true
summary: comparative.
---

The question that gives title to this post is one that I've been asking myself a lot recently.
But before I get into it, let me give you some context on how I got here:

My first -serious- computer vision project was a Visual Odometry algorithm for a drone camera.
It was my Bachelor Thesis, which means that I implemented the whole thing (looking at David Scaramuzza and Andrew Zisserman works) from scratch and in the "traditional" way. Surely that was reinventing the wheel, and there were more modern techniques to solve the problem, but the whole point was to learn how it works.

After that I started working with an underwater robot in the European project STAMS, which made me find out that: 1. there's very little work in underwater computer vision, and 2. there's a very good reason for that: the imaging conditions make it very difficult to extract any kind of information from the pictures.

![](https://media1.tenor.com/images/bba8fcb26bf3ff32fd1125c175b8268d/tenor.gif)

For my Master Thesis I decided then to implement a Visual Odometry algorithm again... But let's make it underwater now!
However, I found out that I could not extract information the traditional way (i.e., keypoints) with the picture as it is. So, the next step I took was to process the image in order to improve the saliency of those keypoints, and remove all the noise that came from the underwater image formation.

Again, I avoided the use of Deep Learning, because I wanted to get to know very well **why** and **how** was this challenging image formation occurring, and how to solve it. If you're interested in my findings, you can read [my journal article published](https://doi.org/10.3390/s19245497) in Sensors. You'll find that I did a simulation of this imaging conditions too. That was because, now that I know how this happens, why not simulating it?



Anyway, after all this research, and taking now into consideration Deep Learning techniques, a new question arises in my head: What's better now, processing the image first and then extracting the information (both with DL algorithms), or just directly training a Deep Learning algorithm to extract the information from a raw underwater picture?
