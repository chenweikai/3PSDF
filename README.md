# 3-Pole Signed Distance Function (3PSDF)

This repository contains the code for "[3PSDF: Three-Pole Signed Distance Function for Learning Surfaces with Arbitrary Topologies](https://arxiv.org/abs/2205.15572)" (CVPR 2022) by [Weikai Chen](http://chenweikai.github.io/), [Cheng Lin](https://clinplayer.github.io/), Weiyang Li and [Bo Yang](https://sites.google.com/site/boyanghome/home). The project page can be found [here](http://chenweikai.github.io/projects/proj_cvpr22_3psdf.html).

<!-- 
![alt text](https://github.com/chenweikai/3PSDF/blob/master/images/3psdf_teaser.jpg?raw=true) -->

<img src="https://raw.githubusercontent.com/chenweikai/3PSDF/master/images/3psdf_teaser.png" width="60%">

## Contents

1. [Introduction](#introduction)
2. [Install](#install)
3. [Usage](#usage)
4. [Contacts](#contacts)

## Introduction

3-Pole Signed Distance Function (3PSDF) is a learnable implicit representation that is capable of representing surfaces with arbitrary topologies, including open surfaces. Unlike unsiged distance functions, 3PSDF can be easily converted into mesh using the classic iso-surface extraction technique, e.g. the Marching Cubes algorithm. 3PSDF can be learned in a manner as simple as 3-way classification, which only requires a light change for existing frameworks based on occupancy prediction. 

This code repository currently contains two parts of code: (1) C++ code for computing 3PSDF and corresponding samples for training from an input mesh, and (2) Python code for training single-view reconstruction using 3PSDF.

## Install

#### 1. Data generation (C++)

The folder `data_generation` contains the C++ code for generating 3PSDF samples. 

This repository is self-contained -- you do not need to install any external libraries. 
Simply run the following lines to compile the code:

  ```
  cd data_generation
  mkdir build
  cd build
  cmake ..
  make -j8
  ```

The code is only tested on Ubuntu 16.04 and 18.04.

#### 2. Code for Single-view Reconstruction (Python)

TODO

## Usage

#### 1. Data generation 

After compilation, binaries named `gen_3psdf_samples` and `batch_generate` will be generated.

(1) `gen_3psdf_samples`: the binary generates 3PSDF sampling points using octree-based sampling. 

Simply run `./gen_3psdf_samples` under the `build` folder will activate a demo setting that reconstructs an open box `soldier_fight.obj` under `/data` folder and saves (a) the generated 3PSDF samples (the`.sdf`file), (b) the mesh reconstructed from the computed 3PSDF field (the`.obj`file), and (c) the sampling point positions (the`.ply`file) to the `/output` folder.

To customize the usage of the program:

```
./gen_3psdf_samples input.obj output.sdf output_recon.obj output_sample_points.ply octree_depth [default=9] flag_writeSDF [default=1] flag_recon_3PSDF [default=1] flag_writePLY [default=1]"
```


(2) `batch_generate`: the binary that generates 3PSDF sampling points in batch. 


To customize the usage of the program:

```
./batch_generate todo_list.txt inDir outSDFDir outObjDir outPlyDir octree_depth [default=9] \
      flag_writeSDF [default=0] flag_writeOBJ [default=1] flag_writePLY [default=1]"
```


#### 2. Single-view Reconstruction (SVR)

TODO


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
