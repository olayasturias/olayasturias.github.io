---
layout: post
cover: assets/images/Screenshot from 2023-01-30 15-12-00.png
title: LateX table tricks for no more table flips (╯°□°)╯︵ ┻━┻
date: 2022-02-30 12:00:00 +0545
categories: latex
author: olaya
featured: true
summary: cool latex table tricks.
---

Do you know when you have an idea for something that can be done quickly, but then you start to have more ideas, one on top of the other and suddenly the small task has become a big task?
Well, that's what happened in this post!

Here, I've gathered together a small set of helpful latex commands that I always use, but still, I have to constantly google them because they don't want to stick into my mind.
This set of commands produces the featured image you can see on top of this text. Those are:
- An amazing placeholder for images containing ducks, from the package duckuments (loving the package and its name).
- An image that consists of a grid of images by using the tabularx package
- Merging multiple columns with centred alignment.
- Merging multiple rows. Interestingly enough, the vertical alignment has to be manually set (the [3em] you see in the latex text below).
- Merging multiple columns and rows simultaneously.

  ```
  \documentclass[12pt,a4paper]{article}
  \usepackage{graphicx}

  \usepackage{duckuments}
  \usepackage{tabularx}
  \usepackage{adjustbox}
  \usepackage{multicol}
  \usepackage{multirow}
  \begin{document}

  \begin{figure}[b!]
  \centering
  \resizebox{\textwidth}{!}{\begin{tabular}{ccccc}

  %%%%%%%%%%%%%%%%%%%%%%5
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}\\\relax
  %%%%%%%%%%%%%%%%%%%%%%%%
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \multicolumn{2}{c}{\multirow{2}{*}[3em]{\includegraphics[width=.4\linewidth]{example-image-duck}}}
  & \multicolumn{2}{c}{\includegraphics[width=.2\linewidth]{example-image-duck}}\\
  %%%%%%%%%%%%%%%%%%%%%%5
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \multicolumn{2}{c}{}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}\\\relax
  %%%%%%%%%%%%%%%%%%%%%%%%
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}\\
  %%%%%%%%%%%%%%%%%%%%%%%%
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \multicolumn{2}{c}{\multirow{2}{*}[3em]{\includegraphics[width=.4\linewidth]{example-image-duck}}}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \multirow{2}{*}[1em]{\includegraphics[width=.2\linewidth]{example-image-duck}}\\
  %%%%%%%%%%%%%%%%%%%%%%5
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \multicolumn{2}{c}{}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  &  \\\relax
  %%%%%%%%%%%%%%%%%%%%%%%%
  \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}
  & \includegraphics[width=.2\linewidth]{example-image-duck}


  \end{tabular}}
  %The more favourable lighting conditions in SeaFloor yield better results than SandPipe's dark environment
  \label{fig:segmentation}
  \end{figure}


  \end{document}
  ```
	





