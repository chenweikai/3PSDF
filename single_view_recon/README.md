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
* Use ```horovodrun -np GPU_NUM``` to indicate the number of GPUs used for distributed training. If you only have one GPU, set ```GPU_NUM``` to 1. You are recommended to use multiple GPUs. 
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

### Data & Download 
* Train/test data (in TFRecord format): [data.zip](https://drive.google.com/file/d/1sNU1av82qrEMq0mTQVl-e_4eo0mkKXsZ/view?usp=sharing).
* Pre-trained model: [weights.zip](https://drive.google.com/file/d/1pPboSALmvZJTEjFLgmKBXm32RNVVR1zt/view?usp=sharing).
* Raw ShapeNet meshes with consistent normals: [shapenet_consistent_normal.zip]()
* 3PSDF values sampled for ShapeNet shapes: [shapnet_3psdf.zip]()
* Raw ShapeNet rendered images and camera parameters: [shapenet_renderings.zip]()
* We provide example scripts to convert the raw data to TFRecord in ```src/utils/shapenet_tfrecord_generator.py```


## Contact
If you have any questions, please email [Cheng Lin](https://clinplayer.github.io/) at chlin@connect.hku.hk.
