# 3PSDF Data Generation
 
This repository contains code for 1) computing 3PSDF field from a given mesh and 2) generating raw sampling data for network training.



## Install

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


## Contacts

Weikai Chen: <chenwk891@gmail.com>
