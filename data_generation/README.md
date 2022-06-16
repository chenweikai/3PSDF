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


After compilation, binaries named `gen_3psdf_samples` and `batch_generate` will be generated.

(1) `gen_3psdf_samples`: the binary generates 3PSDF sampling points using octree-based sampling. 

Simply run `./gen_3psdf_samples` under the `build` folder will activate a demo setting that reconstructs `soldier_fight.obj` (a 3D character with both closed and open surfaces) under `/data` folder and saves (a) the generated 3PSDF samples (the`.sdf`file), (b) the mesh reconstructed from the computed 3PSDF field (the`.obj`file), and (c) the sampling point positions (the`.ply`file) to the `/output` folder.

To customize the usage of the program:

```
./gen_3psdf_samples input.obj output.sdf output_recon.obj output_sample_points.ply octree_depth [default=9] flag_writeSDF [default=1] flag_recon_3PSDF [default=1] flag_writePLY [default=1]"
```

`flag_writeSDF`: whether to generate `.sdf` file (currenlty can be viewed in any text editor for illustration purpose, you are welcomed to customized it into a binary format for acceleration) that encodes the raw training samples for 3PSDF learning. By default, it is set to 1 to activate sdf generation; it is disabled when set to 0. 

`flag_recon_3PSDF`: whether to reconstruct the generated 3PSDF field to a mesh for debug purpose. By default, it is set to 1 to activate the generation; it is disabled when set to 0. 

`flag_writePLY`: whether to generate `.ply` file that encodes the sampling points used in the `.sdf' training data. It is used for debug and visualization purpose. By default, it is set to 1 to activate the generation; it is disabled when set to 0. 


(2) `batch_generate`: the binary that generates 3PSDF sampling points in batch. 


To customize the usage of the program:

```
./batch_generate todo_list.txt inDir outSDFDir outObjDir outPlyDir octree_depth [default=9] \
      flag_writeSDF [default=0] flag_writeOBJ [default=1] flag_writePLY [default=1] [todo_list.txt (optional)]"
```


## Contacts

Weikai Chen: <chenwk891@gmail.com>

