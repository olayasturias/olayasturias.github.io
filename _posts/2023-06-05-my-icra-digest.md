---
layout: post
cover: assets/images/icra23/meaticra.jpeg
title: My ICRA 2023 digest
date: 2023-06-05 12:00:00 +0545
categories: phdstuff conference icra
author: olaya
featured: true
summary: a digest of all things I saw in the conference.
---



# Day 1. Workshops and Tutorials

## W.1. Distributed graphs workshop
![](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/icra23/wsgraph.PNG?raw=true)

The chair for this workshop was [Andrew Davison](https://www.doc.ic.ac.uk/~ajd/), who did a very short but interesting presentation as introduction to the workshop. He definitely left me with the wish of listening more from him, but two things he mentioned particularly catched my interest:

1. The importance of graphs (well, yes, of course, is a workshop about graphs), and how now that the software aiming for graph structures is raising, we need **hardware** that is particularly designed for graphs: the Graphcore Intelligence Processing Unit (IPU). When he talked about it, it sounded a lot like something I used during my bachelors: the FPGAs. Basically, a computer specifically designed for parallel computing. I did a quick read online, and it seems like both are optimized for parallel computing, with the Graphcore being specially designed for (surprise surprise) graph structures, and the FPGA being more flexible for any kind of parallelized computing that you need. [This post in Medium](https://jonathan-hui.medium.com/ai-chips-technology-trends-landscape-gpu-tpu-fpga-startups-4798bfad78a2) does quite a throughout analysis.
2. FutureMapping: factorized computation for spatial AI. A framework that parallelizes computing between the nodes in a graph. Then, those nodes randomly communicate with each other in order to optimize computational resources. Here is [a link to the paper](https://arxiv.org/abs/1910.14139) and [a link to an online tutorial](https://gaussianbp.github.io/).


### W.1.A. A new factor-graph lens on (Clustered) Belief Propagation
#### by Frank Dellaert

[<img src="https://dellaert.github.io/images/NeRF/NeRF-setup.png" width="50%">](https://cseweb.ucsd.edu/~viscomp/projects/LF/papers/ECCV20/nerf/website_renders/depth_reflower.mp4 "NeRFs")

Before getting into factor graphs, he quickly mentioned his recent work in NeRFs. See a post about it [here](https://dellaert.github.io/NeRF/). He also advertised a free book with jupyter notebooks with different aspects about robotics available
[here](http://www.roboticsbook.org/intro.html). Definitely worth looking into!

#### **Brief review of factor graphs** 
Frank Dellaert's work on factor graphs is very well known and available on a [free book](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf). At the begginning of the talk, he did a quick review on the theory within that book.

My quick notes:
> The use of factor graphs allows Sparse matrix factorization. Then we can apply QR factorization on Factor Graphs.
The Bayes tree is a powerful graphical model that enables incremental Smoothing and Mapping (iSAM). An example on iSAM is available [here](https://gtsam.org/tutorials/intro.html) at section 7 and [here](https://gtbook.github.io/gtsam-examples/RangeISAMExample_plaza2.html).


#### **Gibbs sampling via elimination**
This procedure was explained with code examples available [here](https://gtbook.github.io/gtsam-examples/PlanarSLAMExample_sampling.html).
> Take a nonlinear factor graph, for example, one in which the measurements come from a bearing sensor. When obtaining joint Marginals of graph variables results are “wrong” because the problem is nonlinear, and the result is a Laplace approximation.
We can get better results by Gibbs sampling. 
First pick Markov blanket.
Keep sucesion of samples given the current state of the other variables (elimination).
Instead of Laplace approximation, use it as Metropolis proposal.

#### **A different take on loopy BP**
Similarly, code examples available [here](https://gtbook.github.io/gtsam-examples/PlanarSLAMExample_lbp.html).
> Belief propagation: variation approximation. Approximate true posterior with simpler density. Pretend true posterior is multiplication of factors.
Converges to true mean but covariances are overconfident.
Loopy SAM (2007): Gauss-Seidel propagation.
Opposite to Gibbs Sampling: eliminate separator last.

#### **Clustered belief propagation**
More [code examples](https://gtbook.github.io/gtsam-examples/GaussianMRFExample_cbp.html).
> Identify clusters and for a variational approximation at the cluster level – rather than individual belief factors, have Bayes nets/trees as variational components. Basically cluster the graph. Uses all sparsity available in the system. Converges much faster because explores inherent structure of the domain.

### W.2. ICRA 2023 Workshop on Unconventional spatial representations: Opportunities for robotics 
![](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/icra23/usr.PNG?raw=true)

I only attended one talk in this workshop. It seemed to be about using unconventional sensors for perception tasks in general, for what I could see on the posters. Unfortunately, the posters are not available online and I didn't take photos of them.
#### W.2.A. Understanding and Representing the Sea environment for Autonomous Ship Navigation
##### by Hyun-Taek Choi
As a professor in the Korea Research Institute of Ships and Ocean Engineering, he presented quite some interesting field words in that context. I noted down this one, available as a paper [here](https://doi.org/10.1109/JSEN.2023.3259471):
>"Marine Object Segmentation and Tracking by Learning Marine Radar Images for Autonomous Surface Vehicles". The title is self-explanatory. Some capabilities they claim for they network are
> - Noise reduction (because radar is extremely noisy)
> - Detecting Small object and fast moving objects.

![](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/icra23/dpsenet.jpg?raw=true)

### W.3. Workshop on effective Representations, Abstractions, and Priors for Robot Learning (RAP4Robots)
![](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/icra23/rap4robots.PNG?raw=true)
When I joined this workshop there was a Pannel session taking part, with professors discussing around the question: *"Do better representation lead to better generalization?"*. E.g., about the possibility of using an alternative to Euclidean maps and object poses, and about applying representation learning. There was no clear conclusion on the question, which I think is a good sign for this being an open research question.

#### W.3.A. A trilogy of priors: the vision the design and the data
##### by Edward Johns
Edward introduced to us a series of very works within his field of research, which is robot manipulation. [This one](https://www.robot-learning.uk/dall-e-bot) called my attention, since they use generative models (more concretely, DALL-E) to generate different arrangements for the objects in the image. 

### W.4. Pretraining for Robotics (PT4R)
![](https://github.com/olayasturias/olayasturias.github.io/blob/master/assets/images/icra23/pt4r.PNG?raw=true)
#### W.4.A. Where are we in the search for an Artificial Visual Cortex for Embodied Intelligence?
##### by Franziska Meier

From the paper under the same title which can be found [here](https://openreview.net/pdf?id=MhTRXNv7Pc)