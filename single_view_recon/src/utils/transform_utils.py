"""Utility functions for transformations."""

import numpy as np

import tensorflow as tf


def grid_sample(input, grid):
    '''
    TF equivalent of torch.nn.functional.grid_sample
    Sample the multi-channel input using the indices specified by grid
    The value of grid should lie in [-1, 1]
    :param inp: input image features [Nv, H, W, C] Nv is the number of images
    :param grid: UV coordinate [Nv, 1, N, 2] N is number of points
                 Here, (1, N) can be perceived as we sample 1*N samples on each feature channel
    :return [Nv, 1, N, C]  sampled image feature
    '''
    in_shape = tf.shape(input)
    in_h = in_shape[1]
    in_w = in_shape[2]

    # Find interpolation sides
    i, j = grid[..., 1], grid[..., 0] # i - height, j - width
    i = tf.cast(in_h - 1, grid.dtype) * (i + 1) / 2
    j = tf.cast(in_w - 1, grid.dtype) * (j + 1) / 2
    
    i_1 = tf.maximum(tf.cast(tf.floor(i), tf.int32), 0)
    i_2 = tf.minimum(i_1 + 1, in_h - 1)
    j_1 = tf.maximum(tf.cast(tf.floor(j), tf.int32), 0)
    j_2 = tf.minimum(j_1 + 1, in_w - 1)

    # Gather pixel values
    a = tf.range(in_shape[0])[:, tf.newaxis, tf.newaxis]
    b = tf.concat([[1], tf.shape(i)[1:]], axis=0)

    n_idx = tf.tile(a, b)
    c_11 = tf.stack([n_idx, i_1, j_1], axis=-1)
    q_11 = tf.gather_nd(input, c_11)
    q_12 = tf.gather_nd(input, tf.stack([n_idx, i_1, j_2], axis=-1))
    q_21 = tf.gather_nd(input, tf.stack([n_idx, i_2, j_1], axis=-1))
    q_22 = tf.gather_nd(input, tf.stack([n_idx, i_2, j_2], axis=-1))

    # Interpolation coefficients
    di = tf.cast(i, input.dtype) - tf.cast(i_1, input.dtype)
    di = tf.expand_dims(di, -1)
    dj = tf.cast(j, input.dtype) - tf.cast(j_1, input.dtype)
    dj = tf.expand_dims(dj, -1)

    # Compute interpolations
    q_i1 = q_11 * (1 - di) + q_21 * di
    q_i2 = q_12 * (1 - di) + q_22 * di
    q_ij = q_i1 * (1 - dj) + q_i2 * dj

    return q_ij


def computeOctreeSamplingPointsFromBoundingBox(bbox_min, bbox_max, depth):
    length = bbox_max - bbox_min
    num_cells = np.exp2(depth)
    x = np.linspace(bbox_min[0], bbox_max[0], int(num_cells) + 1, dtype=np.float32)
    y = np.linspace(bbox_min[1], bbox_max[1], int(num_cells) + 1, dtype=np.float32)
    z = np.linspace(bbox_min[2], bbox_max[2], int(num_cells) + 1, dtype=np.float32)
    [xs, ys, zs] = np.meshgrid(x, y, z)
    xyz = np.stack((xs, ys, zs), axis=-1)
    xyz = xyz.reshape(-1, 3)
    grid_size = [int(num_cells) + 1, int(num_cells) + 1, int(num_cells) + 1]
    return xyz, grid_size, length


def getBlenderProj(az, el, distance_ratio, img_w=137, img_h=137):
    """Calculate 4x3 3D to 2D projection matrix given viewpoint parameters for ShapeNet images."""
    F_MM = 35.  # Focal length
    SENSOR_SIZE_MM = 32.
    PIXEL_ASPECT_RATIO = 1.  # pixel_aspect_x / pixel_aspect_y
    RESOLUTION_PCT = 100.
    SKEW = 0.
    CAM_MAX_DIST = 1.75
    CAM_ROT = np.asarray([[1.910685676922942e-15, 4.371138828673793e-08, 1.0],
                          [1.0, -4.371138828673793e-08, -0.0],
                          [4.371138828673793e-08, 1.0, -4.371138828673793e-08]])

    # Calculate intrinsic matrix.
    # 2 atan(35 / 2*32)
    scale = RESOLUTION_PCT / 100
    f_u = F_MM * img_w * scale / SENSOR_SIZE_MM
    f_v = F_MM * img_h * scale * PIXEL_ASPECT_RATIO / SENSOR_SIZE_MM
    u_0 = img_w * scale / 2
    v_0 = img_h * scale / 2
    K = np.matrix(((f_u, SKEW, u_0), (0, f_v, v_0), (0, 0, 1)))

    # Calculate rotation and translation matrices.
    # Step 1: World coordinate to object coordinate.
    sa = np.sin(np.radians(-az))
    ca = np.cos(np.radians(-az))
    se = np.sin(np.radians(-el))
    ce = np.cos(np.radians(-el))
    R_world2obj = np.transpose(np.matrix(((ca * ce, -sa, ca * se),
                                          (sa * ce, ca, sa * se),
                                          (-se, 0, ce))))

    # Step 2: Object coordinate to camera coordinate.
    R_obj2cam = np.transpose(np.matrix(CAM_ROT))
    R_world2cam = R_obj2cam * R_world2obj
    cam_location = np.transpose(np.matrix((distance_ratio * CAM_MAX_DIST,
                                           0,
                                           0)))
    T_world2cam = -1 * R_obj2cam * cam_location

    # Step 3: Fix blender camera's y and z axis direction.
    R_camfix = np.matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))
    R_world2cam = R_camfix * R_world2cam
    T_world2cam = R_camfix * T_world2cam

    RT = np.hstack((R_world2cam, T_world2cam))

    return K, RT


def getShapenetRot(rotation_angle):
    '''
    Align ShapeNet coordinates with world coordinates
    '''
    cosval = np.cos(rotation_angle)
    sinval = np.sin(rotation_angle)

    rotation_matrix_x = np.array([[1, 0, 0, 0],
                                  [0, cosval, -sinval, 0],
                                  [0, sinval, cosval, 0],
                                  [0, 0, 0, 1]])
    rotation_matrix_y = np.array([[cosval, 0, sinval, 0],
                                  [0, 1, 0, 0],
                                  [-sinval, 0, cosval, 0],
                                  [0, 0, 0, 1]])
    rotation_matrix_z = np.array([[cosval, -sinval, 0, 0],
                                  [sinval, cosval, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
    scale_y_neg = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    neg = np.array([
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    return np.linalg.multi_dot([neg, rotation_matrix_z, rotation_matrix_z, scale_y_neg, rotation_matrix_x])

def getShapenetBbox():
    bmin = np.array([-0.49839, -0.498965, -0.49685])
    bmax = np.array([0.49839, 0.498965, 0.49685])
    return bmin, bmax
