---
layout: post
cover: assets/images/underwater-enhancement/helo.PNG
title: To enhance or not to enhance? Computer Vision in challenging imaging conditions
date: 2020-09-15 12:00:00 +0545
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

There are some works which perform a preprocessing of the image prior to extracting information from it.
The work proposed by [Li Yujie et al.](https://doi.org/10.1016/j.compeleceng.2016.08.008) performs a de-scattering and color correction based on a model on the scattering and the color distortion. They don't use deep learning for the enhancement process itself, but they have tested their preprocessed image altogether with a classification algorithm, to conclude that the performance of the classification had improved slightly.

https://openaccess.thecvf.com/content_CVPRW_2019/papers/UG2+%20Prize%20Challenge/Uplavikar_All-in-One_Underwater_Image_Enhancement_Using_Domain-Adversarial_Learning_CVPRW_2019_paper.pdf
https://www.sciencedirect.com/science/article/pii/S0031320319303401?casa_token=Wp4-QfWwk3EAAAAA:-xX1kd9LphRm4tXn1JJ3AiWKMp8K9LJmuXwUtVc4iHc_sSaAGsLWlU1fKOGxcrjImSunGsfiwQ
https://ieeexplore.ieee.org/abstract/document/8296508?casa_token=dvk3hwg__LQAAAAA:ghB8AxFo0wxdOcmP3sIM5kVXQwNuNOB0Dz-7oF1KdnwVxqsMrFW1txGwbK-Vl5Yzfc7azXg-iQ
https://www.sciencedirect.com/science/article/pii/S0923596520301478?casa_token=rlj7oV6Ml7cAAAAA:U6rX3yuX3FniElb25ZpVN0MEMZIzCS6lDQUkPRUhf1QKLhu0vpaqwzArwGeel1G0-OLZv0CHEQ

What my intuition first told me is that deep learning algorithms such as image classification, if trained with underwater images, would perform the required enhancement (or preprocessing in general) themselves. Kind of like an underwater black box for image classification.
However, it turns out that doing that preprocessing separately improves the performance of the classification algorithm. This makes me think of some advantages that doing those two steps separately could imply:

  - More modular algorithms: that means, if you want to take only the enhancement part, or just the image classification part, you can separate them easily as they don't directly depend on each other. That is, it's easier to recycle the code. Maybe this advantage is not such a big deal for developers who have high computing resources, as they can just quickly train a new deep net. But for you, it might be something to take into consideration.

  - Recycle code: maybe you have a better (or specific to your use-case) database for image enhancement. Or maybe you don't have an image enhancement database, but want to train your neural net in your specific object-scene-whatever recognition database. If you're focused in a very specific part of your algorithm, it seems like a good idea to rely in a previous work from someone more experienced than you (or with better data than yours) to do that other part. That third-party software will probably work better than yours... and if you save that time, you'll be able to focus better in your field of expertise!

Anyway, let's evaluate it from the object detection perspective. I want to do a comparative between the two approaches, with and without a prior image enhancement, and see which performs better for object detection. I will be using State of the Art algorithms for such comparative.
But first of all, let's think about which metrics we could use:

# Metrics for object detection

### Intersection Over Union
Intersection Over Union (IOU) evaluates the overlap between two bounding boxes, that is, between the ground truth ($B_{gt}$) and the predicted ($B_{p}$) bounding boxes.
It basically divides the area of the overlapping between the area of the union, i.e.:

![IOU](http://www.sciweavers.org/tex2img.php?eq=IOU%20%3D%20%5Cfrac%7Barea%28B_%7Bgt%7D%20%5Cbigcap%20B_%7Bp%7D%29%7D%7Barea%28B_%7Bgt%7D%20%5Cbigcup%20B_%7Bp%7D%29%7D%20&bc=Transparent&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

According to the returned value, we could have:

- True positive  (TP): IOU > threshold. There is a ground truth object and we fairly detected it.
- False negative (FP): a ground truth object not detected.
- False positive (FN): IOU< threshold. The object was detected, but missclassified.

The value of the threshold depends on the metric. MS COCO and PASCAL VOC use an IOU of 0.5, the minimum threshold, but also 0.75 or 0.95 are often used.

### Precision and recall
These two are common Machine Learning concepts.
Precision measures the percentage of correct positive predictions:
![precision](http://www.sciweavers.org/tex2img.php?eq=Precision%20%3D%20%5Cfrac%7BTP%7D%7BTP%2BFP%7D&bc=Transparent&fc=Black&im=jpg&fs=12&ff=arev&edit=0)
Recall is the percentage of  true positives among all ground truths:
![recall](http://www.sciweavers.org/tex2img.php?eq=Precision%20%3D%20%5Cfrac%7BTP%7D%7BTP%2BFN%7D&bc=Transparent&fc=Black&im=png&fs=12&ff=arev&edit=0)


# PASCAL VOC Metrics

Used, of course, in the [PASCAL VOC project](http://host.robots.ox.ac.uk:8080/pascal/VOC/index.html),



-under construction-
