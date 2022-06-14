
"""Code for testing pretrained model."""

import copy
import numpy as np
import os

import argparse
from skimage import measure
import tensorflow as tf
from tqdm import tqdm

from model.data_loader import Dataloader
from src.utils.io_utils import save_obj_mesh_filterNAN, load_filelist
from src.utils.transform_utils import getShapenetBbox, computeOctreeSamplingPointsFromBoundingBox


def parse_args():
    parser = argparse.ArgumentParser(description='3PSDF_test')
    parser.add_argument('--sdf_dir', type=str, default='data/sdf-depth7-tfrecord/',
                        help='data folder for 3psdf training samples')
    parser.add_argument('--cam_dir', type=str, default='data/cam-tfrecord/',
                        help='data folder for camera parameters')
    parser.add_argument('--img_dir', type=str, default='data/img-tfrecord/',
                        help='data folder for input images')
    parser.add_argument('--split_file', type=str, default='data/data_split/test.lst',
                        help='data list split for testing')

    parser.add_argument('--batch_size', type=int, default=8,
                        help='batch size')
    parser.add_argument('--point_num', type=int, default=20000,
                        help='number of point samples for a single model')
    parser.add_argument('--octree_depth', type=int, default=8,
                        help='sampling density in the implicit field')
    parser.add_argument('--load_model_path', type=str, default='weights/3psdf_svr_weights',
                        help='path to load the pre-trained model')
    parser.add_argument('--save_result_path', type=str, default='output/',
                        help='path to save the reconstructed meshes')
    args = parser.parse_args()

    return args


def sampleImpField(model, images, samples, cameras, grid_size, batch_size, point_num, level_value=0.5):
    '''
    Generate a sampling grid to uniformly sample the implicit field generated by network
    :param model: trained model
    :param images: input images
    :param samples: sampling point positions for evaluation
    :param cameras: camera parameters
    :param grid_size: xyz length of the sampling voxel grids
    :param batch_size: the number of shapes in one batch
    :param level_value: iso value for marching cubes algorithm
    '''

    print("Sampling the field ...")
    num_pts = samples.shape[0]
    sdf = np.zeros((batch_size, num_pts, 1), dtype=np.float32)
    num_feed_iter = num_pts / point_num
    num_feed_iter = int(np.ceil(num_feed_iter))
    with tqdm(total=num_feed_iter) as pbar:
        for i in range(num_feed_iter):
            startID = i * point_num
            stopID = startID + point_num

            if stopID > num_pts:
                startID = num_pts - point_num
                stopID = num_pts

            points = samples[startID: stopID].astype(np.float32)

            # tile the sample points to match the batch size
            points = np.tile(points[None, :, :], (batch_size, 1, 1))

            raw = model.__call__(images, points, cameras, view_num=1)
            raw = raw.numpy()
            predicts = raw.argmax(axis=2)

            filtered_predicts = np.where(predicts == 2, float('nan'), predicts)
            filtered_predicts = filtered_predicts.reshape(batch_size, point_num, 1)
            sdf[:, startID:stopID] = filtered_predicts[:, ]
            pbar.update(1)

    sdf = sdf.reshape(batch_size, grid_size[1], grid_size[0], grid_size[2])

    print("Start marching cube ... ")
    all_vertices = []
    all_faces = []
    for i in range(batch_size):
        verts, faces, _, _ = measure.marching_cubes_lewiner(sdf[i], level_value)
        all_vertices.append(verts)
        all_faces.append(faces)
    return all_vertices, all_faces


def evaluate(args):
    sdf_dir = args.sdf_dir
    cam_dir = args.cam_dir
    img_dir = args.img_dir
    split_file = args.split_file

    batch_size = args.batch_size
    point_num = args.point_num
    octree_depth = args.octree_depth

    load_model_path = args.load_model_path
    save_result_path = args.save_result_path

    if not os.path.exists(load_model_path):
        print('Network weight file does not exist!')
        return

    model = tf.saved_model.load(load_model_path)
    print('Loaded network parameters successfully!')

    if not os.path.exists(save_result_path):
        os.makedirs(save_result_path)

    bmin, bmax = getShapenetBbox()
    xyz_samples, grid_size, bbox_size = computeOctreeSamplingPointsFromBoundingBox(bmin, bmax, octree_depth)

    file_list = load_filelist(split_file)
    for i in range(int(len(file_list) / batch_size)):
        indices = [i * batch_size + n for n in range(batch_size)]
        img_batch, xyz_batch, gt_batch, camera_batch, name_batch = Dataloader.get_batched_data(file_list, indices,
                                                                                               sdf_dir,
                                                                                               img_dir, cam_dir,
                                                                                               point_num, test=True)
        vertices_batch, faces_batch = sampleImpField(model, img_batch, xyz_samples, camera_batch, grid_size, batch_size,
                                                     point_num)
        for j in range(batch_size):
            verts = vertices_batch[j]
            faces = faces_batch[j]
            # align the reconstruction results with GT
            tmp = copy.copy(verts[:, 0])
            verts[:, 0] = copy.copy(verts[:, 1])
            verts[:, 1] = tmp
            grid_l = grid_size[0]
            bbox_l = bmax - bmin
            verts[:, 0] = verts[:, 0] / grid_l * bbox_l[0] + bmin[0]
            verts[:, 1] = verts[:, 1] / grid_l * bbox_l[1] + bmin[1]
            verts[:, 2] = verts[:, 2] / grid_l * bbox_l[2] + bmin[2]
            # output meshes
            output_file_name = os.path.join(save_result_path, name_batch[j] + ".obj")
            save_obj_mesh_filterNAN(verts, faces, output_file_name)
            print("saved obj to ", output_file_name)


if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    tf.config.experimental.set_memory_growth(physical_devices[0], True)

    args = parse_args()
    evaluate(args)
