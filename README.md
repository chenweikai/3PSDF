# 3-Pole Signed Distance Function (3PSDF)
 
![alt text](https://github.com/chenweikai/3PSDF/blob/main/images/3psdf_teaser.png?raw=true)

This repository contains the code for "[3PSDF: Three-Pole Signed Distance Function for Learning Surfaces with Arbitrary Topologies](https://arxiv.org/abs/2205.15572)" (CVPR 2022) by [Weikai Chen](http://chenweikai.github.io/), [Cheng Lin](https://clinplayer.github.io/), Weiyang Li and [Bo Yang](https://sites.google.com/site/boyanghome/home). The project page can be found [here](http://chenweikai.github.io/projects/proj_cvpr22_3psdf.html).


## Contents

1. [Introduction](#introduction)
2. [Install](#install)
3. [Contacts](#contacts)

## Introduction

3-Pole Signed Distance Function (3PSDF) is a learnable implicit representation that is capable of representing surfaces with arbitrary topologies, including open surfaces. Unlike unsiged distance functions, 3PSDF can be easily converted into mesh using the classic iso-surface extraction technique, e.g. the Marching Cubes algorithm. 3PSDF can be learned in a manner as simple as 3-way classification, which only requires a slight change for existing frameworks based on occupancy prediction. 

This code repository currently contains two parts of code: (1) C++ code for computing 3PSDF of an input mesh and the corresponding sample data for network training, and (2) Python code for training single-view reconstruction using 3PSDF.

## Install



## Contacts

Weikai Chen: <chenwk891@gmail.com>

Any discussions or concerns are welcomed!

### Citation

If you find our project useful in your research, please consider citing:

```
@article{chen_2022_3psdf,
  title={3PSDF: Three-Pole Signed Distance Function for Learning Surfaces with Arbitrary Topologies},
  author={Chen, Weikai and Lin, Cheng and Li, Weiyang and Yang, Bo},
  journal={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  month={June},
  year={2022}
}
```
