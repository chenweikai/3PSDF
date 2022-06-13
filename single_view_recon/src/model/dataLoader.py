import glob
import math
import numpy as np
import os
import sys
import timeit
import random
from collections import defaultdict
import tensorflow as tf
from tqdm import tqdm
from src.utils.io_utils import walklevel
from src.utils.transform_utils import getBlenderProj, getShapenetRot

class Dataloader:
    def __init__(self):
        pass

    @staticmethod
    def load_ShapeNet_IMG_TFRecord(file_name, num_view=24):
        keys_to_features = {
            'name': tf.io.FixedLenFeature((), tf.string),
            'height': tf.io.FixedLenFeature([], tf.int64, default_value=0),
            'width': tf.io.FixedLenFeature([], tf.int64, default_value=0),
            'channels': tf.io.FixedLenFeature([], tf.int64, default_value=0)
        }

        for i in range(num_view):
            view_id = '0' + str(i) if i < 10 else str(i)
            keys_to_features[view_id] = tf.io.FixedLenFeature((), tf.string)

        def _parse_function(example_proto):
            return tf.io.parse_single_example(example_proto, keys_to_features)

        raw_dataset = tf.data.TFRecordDataset(file_name)
        parsed = raw_dataset.map(_parse_function)

        return parsed

    @staticmethod
    def load_ShapeNet_CAM_TFRecord(file_name):
        keys_to_features = {
            'name': tf.io.FixedLenFeature((), tf.string),
            'cam_pos': tf.io.FixedLenFeature((), tf.string),
            'cam_rot': tf.io.FixedLenFeature((), tf.string),
            'cam_K': tf.io.FixedLenFeature((), tf.string)
        }

        def _parse_function(example_proto):
            return tf.io.parse_single_example(example_proto, keys_to_features)

        raw_dataset = tf.data.TFRecordDataset(file_name)
        parsed = raw_dataset.map(_parse_function)

        return parsed

    @staticmethod
    def parse_a_random_view(imgs, is_first=False, max_view=23):
        views = []
        count = 0
        for sample in imgs:
            count += 1
            if is_first:
                rand_id = 0
            else:
                rand_id = random.randint(0, max_view)

            view_id = '0' + str(rand_id) if rand_id < 10 else str(rand_id)
            view = tf.io.decode_image(sample[view_id], channels=4, dtype=tf.dtypes.uint8).numpy()
            views.append(view)
            views = list(map(np.float32, views))
            views = [(m - 127.5) / 127.5 for m in views]
            views = [tf.clip_by_value(m, -1.0, 1.0) for m in views]
            views = [tf.image.per_image_standardization(m) for m in views]
            views = np.asarray(views, dtype=np.float32)

        return views, rand_id

    @staticmethod
    def get_camera_dict(camera_parsed, view_id):
        camera_dict = {}
        for sample in camera_parsed:
            cam_rot = tf.io.decode_raw(sample['cam_rot'], out_type=tf.float32)
            cam_pos = tf.io.decode_raw(sample['cam_pos'], out_type=tf.float32)
            cam_K = tf.io.decode_raw(sample['cam_K'], out_type=tf.float32)

            cam_rot = tf.reshape(cam_rot, [24, 3, 3]).numpy()
            cam_pos = tf.reshape(cam_pos, [24, 1, 3]).numpy()
            cam_K = tf.reshape(cam_K, [24, 3, 3]).numpy()

            camera_dict['cam_rot'] = cam_rot[None, view_id, :, :]
            camera_dict['cam_pos'] = cam_pos[None, view_id, :, :]
            camera_dict['cam_K'] = cam_K[None, view_id, :, :]

        return camera_dict

    @staticmethod
    def get_batched_data(file_list, indices, sdf_dir, img_dir, cam_dir, point_num, test=False):
        '''
        Given a set of indices, pack a data batch that can be directly feed into the network for training
        [input] file_list: a list of file names
        [input] indices: selected indices to be packed
        [input] sdf_dir: directory that contains the 3psdf data in tfrecord format
        [input] img_dir: directory that contains the image data in tfrecord format
        [input] cam_dir: directory that contains the camera data in tfrecord format
        [input] point_num: number of samples for a shape
        [input] test: True or False, indicate if the data batch is for testing
        [output]: a set of batched data used for training
        '''
        # list that stores data from batches
        xyz_batch = []
        flag_batch = []
        views_batch = []
        cams_batch = []
        names_batch = []

        # assemble batch data
        for index in indices:
            file = file_list[index]
            cat = file.split('/')[0]
            fn = file.split('/')[1]
            sdf_dir_cat = sdf_dir + cat
            img_dir_cat = img_dir + cat
            cam_dir_cat = cam_dir + cat

            # load the pre-defined SDF tfrecord
            sdf_file = os.path.join(sdf_dir_cat, fn + '.tfrecord')
            if not os.path.exists(sdf_file):
                print('SDF file does not exist.')
                continue
            sdf_data = Dataloader.load_SDF_TFRecord(sdf_file)
            sdf_dict = Dataloader.load_parsed_SDF(sdf_data)

            # parse the SDF tfrecord
            for name, value in sdf_dict.items():
                # obtain the image and camera
                imgs_file = os.path.join(img_dir_cat, name + '_imgs.tfrecord')
                cams_file = os.path.join(cam_dir_cat, name + '_cams.tfrecord')
                if not os.path.exists(imgs_file) or not os.path.exists(cams_file):
                    print('IMG or CAM files do not exist.')
                    continue

                # load and parse the image and camera
                imgs = Dataloader.load_ShapeNet_IMG_TFRecord(imgs_file)
                view, view_id = Dataloader.parse_a_random_view(imgs, is_first=test)
                cams = Dataloader.load_ShapeNet_CAM_TFRecord(cams_file)
                camera_dict = Dataloader.get_camera_dict(cams, view_id)

                # obtain the fixed number of training sample points
                xyz = value['xyz']
                flags = value['flag']
                if xyz.shape[0] > point_num:
                    sample_indices = random.sample(range(xyz.shape[0]), point_num)
                else:
                    sample_indices = np.random.choice(xyz.shape[0], point_num)
                xyz = xyz[sample_indices]
                flags = flags[sample_indices]

                # print('Batch data:', 'category', cat, 'name', fn, 'view', view_id)

                # pack all the data for batch training
                xyz_batch.append(xyz[None, :])
                flag_batch.append(flags[None, :])
                cams_batch.append(camera_dict)
                views_batch.append(view)
                names_batch.append(cat + '-' + fn)

        # combine the batched data into tensors
        # xyz and label tensor
        xyz_final_batch = tf.concat(xyz_batch, 0)
        flag_final_batch = tf.concat(flag_batch, 0)
        # camera data tensor
        cam_rot_list = [cam['cam_rot'] for cam in cams_batch]
        cam_pos_list = [cam['cam_pos'] for cam in cams_batch]
        cam_K_list = [cam['cam_K'] for cam in cams_batch]
        camera_dict_batch = {}
        camera_dict_batch['cam_rot'] = tf.concat(cam_rot_list, 0)
        camera_dict_batch['cam_pos'] = tf.concat(cam_pos_list, 0)
        camera_dict_batch['cam_K'] = tf.concat(cam_K_list, 0)
        # image data tensor
        view_final_batch = tf.concat(views_batch, 0)

        return view_final_batch, xyz_final_batch, flag_final_batch, camera_dict_batch, names_batch

    @staticmethod
    def load_parsed_SDF(parsed):
        '''
        Load the parsed raw sdf data and return a structured data dictionary for each data query
        [input] parsed: parsed sdf raw data
        [output]: a data dictionary that use subject name as key and corresponding sdf data as value
        '''
        dataMap = defaultdict(list)
        for sample in parsed:
            xyz = tf.io.decode_raw(sample['xyz'], tf.float32).numpy()
            flags = tf.io.decode_raw(sample['outFlag'], tf.int64).numpy()
            dists = tf.io.decode_raw(sample['distance'], tf.float32).numpy()
            xyz = xyz.reshape(flags.shape[0], 3)
            tmp_name = sample['name'].numpy().decode('UTF-8')
            if "-" in tmp_name:
                name = tmp_name
                if name in dataMap:
                    dataMap[name]["xyz"] = np.append(dataMap[name]["xyz"], xyz, axis=0)
                    dataMap[name]["flag"] = np.append(dataMap[name]["flag"], flags, axis=0)
                    dataMap[name]["distance"] = np.append(dataMap[name]["distance"], dists, axis=0)
                else:
                    dataDict = {"xyz": xyz, "flag": flags, "distance": dists}
                    dataMap[name] = dataDict
            else:
                dataDict = {"xyz": xyz, "flag": flags, "distance": dists}
                dataMap[tmp_name] = dataDict
        return dataMap

    @staticmethod
    def load_SDF_TFRecord(file_name):
        '''
        Load sdf (signed distance field) tfrecord
        [input] file_name: file name of the input sdf tfrecord
        [output]: the parsed raw data
        '''
        feature_description = {
            'name': tf.io.FixedLenFeature((), tf.string),
            'xyz': tf.io.FixedLenFeature((), tf.string),
            'outFlag': tf.io.FixedLenFeature([], tf.string),
            'distance': tf.io.FixedLenFeature([], tf.string)
        }

        def _parse_function(example_proto):
            return tf.io.parse_single_example(example_proto, feature_description)

        raw_dataset = tf.data.TFRecordDataset(file_name)
        parsed = raw_dataset.map(_parse_function)

        return parsed

    @staticmethod
    def create_ShapeNet_SDF_TFRecord(data_dir, file_ext, out_dir):
        '''
        create separate TFRecord for sampling points of signed distance field
        [input] data_dir: directory to samples of signed distance field (SDF)
        [input] file_ext: file extension of input files
        [input] out_dir:  output directory for the generated tfrecords
        '''
        if not os.path.isdir(data_dir):
            print('data repository does NOT exist!')

        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        for infile in sorted(glob.glob(os.path.join(data_dir, '*.' + file_ext))):
            print("Current file is : ", infile)
            start = timeit.default_timer()
            filename = os.path.splitext(os.path.basename(infile))[0]
            out_name = os.path.join(out_dir, filename + ".tfrecord")
            writer = tf.io.TFRecordWriter(out_name)

            f = open(infile, "r")
            line = f.readline()
            num = int(line.split()[0])  # number of sampling points

            positions = np.zeros((num, 3), dtype=np.float32)
            outFlags = np.zeros((num, 1), dtype=np.int64)
            distances = np.zeros((num, 1), dtype=np.float32)
            cnt = 0
            pbar = tqdm(total=num)
            while line:
                line = f.readline()
                eachline = line.split()
                if (len(eachline) == 0):
                    break
                xyz = [float(x) for x in eachline[0:3]]  # (x,y,z) position
                flag = [int(eachline[3])]  # in/out flag: 1 - outside; 0 - inside; 2 - NAN region
                dist = [float(eachline[4])]  # [ float(x) for x in eachline[4] ]
                xyz = np.array(xyz).astype('float32')
                flag = np.array(flag).astype('int64')
                dist = np.array(dist).astype('float32')
                positions[cnt] = xyz
                outFlags[cnt] = flag
                distances[cnt] = dist
                cnt += 1
                pbar.update(1)

            assert positions.shape[0] == num
            assert outFlags.shape[0] == num
            assert distances.shape[0] == num

            totalSize = sys.getsizeof(positions) + sys.getsizeof(distances) + sys.getsizeof(outFlags)
            GBSize = totalSize / 1024 / 1024 / 1024
            maxSize = 1.9
            if (GBSize) > maxSize:
                # we need to split it into more proto files
                num_split = math.ceil(GBSize / maxSize)
                partSize = math.ceil(num / num_split)
                for i in range(num_split):
                    start = i * partSize
                    stop = (i + 1) * partSize
                    if stop > num:
                        stop = num
                    print("processing part {} to {}".format(start, stop))
                    part_positions = positions[start:stop]
                    part_distances = distances[start:stop]
                    part_flags = outFlags[start:stop]
                    part_name = filename + "-" + str(i)
                    # create tf example proto object
                    features = {
                        'name': tf.train.Feature(bytes_list=tf.train.BytesList(value=[part_name.encode()])),
                        'xyz': tf.train.Feature(
                            bytes_list=tf.train.BytesList(value=[part_positions.flatten().tostring()])),
                        'outFlag': tf.train.Feature(
                            bytes_list=tf.train.BytesList(value=[part_flags.flatten().tostring()])),
                        'distance': tf.train.Feature(bytes_list=tf.train.BytesList(value=[part_distances.tostring()]))
                    }
                    example_proto = tf.train.Example(features=tf.train.Features(feature=features))
                    writer.write(example_proto.SerializeToString())
            else:
                # create tf example proto object
                features = {
                    'name': tf.train.Feature(bytes_list=tf.train.BytesList(value=[filename.encode()])),
                    # 'gridSize': tf.train.Feature(bytes_list=tf.train.BytesList(value=[grid_size.tostring()])),
                    'xyz': tf.train.Feature(bytes_list=tf.train.BytesList(value=[positions.flatten().tostring()])),
                    'outFlag': tf.train.Feature(bytes_list=tf.train.BytesList(value=[outFlags.flatten().tostring()])),
                    'distance': tf.train.Feature(bytes_list=tf.train.BytesList(value=[distances.tostring()]))
                }
                example_proto = tf.train.Example(features=tf.train.Features(feature=features))
                writer.write(example_proto.SerializeToString())
                print("Finished writing to output tfrecord: ", out_name)
                stop = timeit.default_timer()
                print('Time: {} mins'.format((stop - start) / 60))

    @staticmethod
    def create_ShapeNet_IMG_TFRecord(data_dir, file_ext, out_dir):
        '''
        create TFRecord for for ShapeNet images with arbitrary views; one file one shape
        [input] data_dir: directory to samples of image data
        [input] file_ext: file extension, png or jpg
        [input] out_dir: the folder name of output tfrecord
        '''
        if not os.path.isdir(data_dir):
            print('data repository does NOT exist!')

        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        folder_names = walklevel(data_dir, depth=1, is_folder=True)
        count = 0

        # for each shape
        for f in sorted(folder_names):

            object_name = os.path.basename(f)
            out_name = os.path.join(out_dir, object_name + "_imgs.tfrecord")
            writer = tf.io.TFRecordWriter(out_name)

            print("current shape is:", object_name)

            # get the image files under this folder
            items = glob.glob(f + '/rendering/*.' + file_ext)
            img_dict = {}
            height = -1
            width = -1
            channels = -1
            features = {'name': tf.train.Feature(bytes_list=tf.train.BytesList(value=[object_name.encode()]))}

            for item in sorted(items):
                img = cv2.imread(item, cv2.IMREAD_UNCHANGED)
                img = cv2.resize(img, (224, 224))
                img_encode = cv2.imencode('.png', img)[1].tostring()

                height, width, channels = img.shape

                filename = os.path.splitext(item)[0]  # full image path without ext
                basename = os.path.basename(filename)  # exact image name
                view_id = basename
                print("processing ", item, " view id: ", view_id)
                features[view_id] = tf.train.Feature(bytes_list=tf.train.BytesList(value=[img_encode]))

            features['height'] = tf.train.Feature(int64_list=tf.train.Int64List(value=[height]))
            features['width'] = tf.train.Feature(int64_list=tf.train.Int64List(value=[width]))
            features['channels'] = tf.train.Feature(int64_list=tf.train.Int64List(value=[channels]))

            example_proto = tf.train.Example(features=tf.train.Features(feature=features))
            count = count + 1
            print("processed item: ", count)
            writer.write(example_proto.SerializeToString())

    @staticmethod
    def create_ShapeNet_CAM_TFRecord(data_dir, out_dir):
        '''
        create TFRecord for ShapeNet camera parameters of synthetic renderings
        [input] data_dir: directory to samples of camera parameter
        [input] file_ext: file extension of the original render meta file
        [input] out_name: file name of output camera tfrecord
        '''

        if not os.path.isdir(data_dir):
            print('data repository does NOT exist!')

        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        folder_names = walklevel(data_dir, depth=1, is_folder=True)

        # for each shape
        for f in sorted(folder_names):
            object_name = os.path.basename(f)
            cam_file = os.path.join(data_dir, object_name, 'rendering', 'rendering_metadata.txt')
            out_name = os.path.join(out_dir, object_name + "_cams.tfrecord")
            writer = tf.io.TFRecordWriter(out_name)
            cam_params = np.loadtxt(cam_file)

            features = {'name': tf.train.Feature(bytes_list=tf.train.BytesList(value=[object_name.encode()]))}
            cam_rot_all = []
            cam_pos_all = []
            cam_K_all = []
            for index, param in enumerate(cam_params):
                az, el, distance_ratio = param[0], param[1], param[3]
                K, RT = getBlenderProj(az, el, distance_ratio, img_w=224, img_h=224)
                rot_mat = getShapenetRot(-np.pi / 2)

                trans_mat = np.linalg.multi_dot([RT, rot_mat])
                trans_mat_right = np.transpose(trans_mat)

                T = trans_mat_right[3, :]
                R = trans_mat_right[:3, :]

                cam_rot_all.append(R)
                cam_pos_all.append(T)
                cam_K_all.append(K)

            cam_rot_all = np.array(cam_rot_all).astype(np.float32)
            cam_pos_all = np.array(cam_pos_all).astype(np.float32)
            cam_K_all = np.array(cam_K_all).astype(np.float32)

            features['cam_rot'] = tf.train.Feature(bytes_list=tf.train.BytesList(value=[cam_rot_all.tobytes()]))
            features['cam_pos'] = tf.train.Feature(bytes_list=tf.train.BytesList(value=[cam_pos_all.tobytes()]))
            features['cam_K'] = tf.train.Feature(bytes_list=tf.train.BytesList(value=[cam_K_all.tobytes()]))

            example_proto = tf.train.Example(features=tf.train.Features(feature=features))
            writer.write(example_proto.SerializeToString())


