# #3PSDF-SVR

This repository contains the source code for applying 3PSDF to 3D reconstruction tasks based on single-view images.


## Code
### Installation
This repository depends on [Tensorflow](https://pytorch.org/), [NumPy](https://numpy.org/), [Scikit-image](https://scikit-image.org/) and [Horovod](https://github.com/horovod/horovod). The code is tested under the following package versions with CUDA 11.2 and Ubuntu 18.04:
```
tensorflow-gpu==2.6.0
numpy==1.19.5
scikit-image==0.18.3
horovod==0.23.0
```

### Training

* Example command with required parameters to indicate the data folder:
```
horovodrun -np 2 python -m src.train
  --sdf_dir data/sdf-depth7-tfrecord
  --cam_dir data/cam-tfrecord
  --img_dir data/img-tfrecord
  --split_file data/datasplit/train.lst
``` 
* Use ```horovodrun -np GPU_NUM``` to indicate the number of GPUs used for distributed training. If you only have one GPU, set ```GPU_NUM``` to 1. It is strongly recommended to use multiple GPUs. 
* See `python -m src.train --help` for all the detailed training options.

### Testing
* Example command with required parameters to indicate the data folder and pre-trained model:
```
python -m src.test
  --sdf_dir data/sdf-depth7-tfrecord
  --cam_dir data/cam-tfrecord
  --img_dir data/img-tfrecord
  --split_file data/datasplit/test.lst
  --load_model_path weights/3psdf_svr_weights
``` 
* See `python -m src.test --help` for all the detailed testing options. 

## Data & Download 
To run the code:
* Train/test data (in TFRecord format): [data.zip](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chlin_connect_hku_hk/Eumlg1Wr9NNPoCYCp2lW1rAB-jptB_EiABrjDj4sMZEYsQ?e=yGVM8h).
* Pre-trained model: [weights.zip](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/chlin_connect_hku_hk/EvqZNm7ns_1An17j7ui7TigBHMbnzWW4u7UOnvCrR7NePA?e=65iurs).

To obtain the raw data:
* Raw ShapeNet meshes with consistent normals: [shapenet_consistent_normal.zip](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/EX6nk1bMAyJJhGMm8avoyIsBYyEir82jIzFTGjTAforlyQ?e=DQ4N8V)
* 3PSDF values sampled for ShapeNet shapes: [shapnet_3psdf.zip](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/ESaGhz9TnZJIveoky_g2DU0BpCsrl4N2_qIiKD6n5gvRiQ?e=1CAfwZ)
* Raw ShapeNet rendered images and camera parameters from [3DR2N2](https://github.com/chrischoy/3D-R2N2): [shapenet_renderings.zip](http://cvgl.stanford.edu/data2/ShapeNetRendering.tgz)

To convert data:
* We provide example scripts to convert the raw data to TFRecord in ```src/utils/shapenet_tfrecord_generator.py```


## Contact
If you have any questions, please email Weikai Chen and Cheng Lin at chlin@connect.hku.hk.
