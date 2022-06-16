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

`input.obj`: file name of the input mesh for computing its corresponding 3PSDF field and training data. 

`output.sdf`: file name of the output `.sdf` file. It should be provided no matter whether you would like to generate it or not. If you choose not to generate it (flag_writeSDF set as 0), the specified `output.sdf` will not be generated.

`output_recon.obj`: file name of the output obj mesh that is reconstructed from the generated 3PSDF field. It should be provided no matter whether you would like to generate it or not. If you choose not to generate it (flag_recon_3PSDF set as 0), the specified `output_recon.obj` will not be generated.

`output_sample_points.ply`: file name of the output point cloud that is the sampling points of training data. It should be provided no matter whether you would like to generate it or not. If you choose not to generate it (flag_writePLY set as 0), the specified `output_sample_points.ply` will not be generated.

`octree_depth`: the depth of the octree that is used to generate sampling points. The larger the depth, the more accurate is the 3PSDF reconstruction.

`flag_writeSDF`: whether to generate `.sdf` file (currenlty can be viewed in any text editor for illustration purpose, you are welcomed to customized it into a binary format for acceleration) that encodes the raw training samples for 3PSDF learning. By default, it is set to 1 to activate sdf generation; it is disabled when set to 0. 

`flag_recon_3PSDF`: whether to reconstruct the generated 3PSDF field to a mesh for debug purpose. By default, it is set to 1 to activate the generation; it is disabled when set to 0. 

`flag_writePLY`: whether to generate `.ply` file that encodes the sampling points used in the `.sdf' training data. It is used for debug and visualization purpose. By default, it is set to 1 to activate the generation; it is disabled when set to 0. 


(2) `batch_generate`: the binary that generates 3PSDF sampling points in batch. 


To customize the usage of the program:

```
./batch_generate inDir outSDFDir outObjDir outPlyDir octree_depth [default=9] \
      flag_writeSDF [default=0] flag_writeOBJ [default=1] flag_writePLY [default=1] [todo_list.txt (optional)]"
```


## Contacts

If you have any questions, please contact Weikai Chen: <chenwk891@gmail.com>

