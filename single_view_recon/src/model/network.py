import tensorflow as tf
from tensorflow.keras import Model
from src.model.mlpClassifier import Classifier
from src.model.pointConv import PointConv
from src.utils.transform_utils import grid_sample
from src.model.imgEncoder import ImageEncoder

class DeepImpNet(Model):
    def __init__(self):
        super(DeepImpNet, self).__init__()
        # modules / functions
        self.pointConv = PointConv()
        self.img_encoder = ImageEncoder()
        self.local_classifier = Classifier()
        self.global_classifier = Classifier()

    def projection_shapenet(self, pts, camera_dict):
        '''
        Compute the orthogonally projected UV coordinates of 3D points given transform matrices
        :param pts: [Nv, N ,3] Tensor of 3D points, N is number of points
        :param cam_pos: [Nv, 3, 1] camera position
        :param cam_rot: [Nv, 3, 3] camera rotation
        :param image_size: resolution of image
        :param f: derived from the intrinsic parameters K
        :return uv: [Nv, N, 3] xyz coordinates for each point on multiview images
        '''

        point_num = pts.shape[1]

        #parse camera
        cam_pos = camera_dict['cam_pos']
        cam_rot = camera_dict['cam_rot']
        cam_K = camera_dict['cam_K']

        cam_pos = tf.convert_to_tensor(cam_pos, tf.float32)
        cam_rot = tf.convert_to_tensor(cam_rot, tf.float32)
        cam_K = tf.convert_to_tensor(cam_K, tf.float32)

        cam_pos = tf.tile(cam_pos, [1, point_num, 1])

        # projection
        pts_cam = tf.einsum('aij,ajk->aik', pts, cam_rot) + cam_pos
        X, Y, Z = pts_cam[:, :, 0], pts_cam[:, :, 1], pts_cam[:, :, 2]
        pts_cam = pts_cam / Z[:, :, None]
        pts_img = tf.einsum('aij,ajk->aik', cam_K, tf.transpose(pts_cam, perm=[0, 2, 1]))
        pts_img = tf.transpose(pts_img, perm=[0, 2, 1])
        uv = pts_img[:, :, 0:2]
        uvz = tf.concat([uv, Z[:, :, None]], 2)

        return uvz

    def index(self, feat, uv):
        '''
        Extract the local feature from according to the UV coordinates
        :param feat: [Nv, H, W, F] image features, Nv is the number of images, F is num. of feat. channels
        :param uv: [Nv, N, 2] uv coordinate, Nv is the number of images, N is number of points
        :return [N, Nv, F] 
        '''
        uv = tf.expand_dims(uv, 1)
        local_feat = grid_sample(feat, uv)
        local_feat = tf.squeeze(local_feat)

        if len(local_feat.shape) == 2:
            local_feat = tf.expand_dims(local_feat, axis = 0)
        local_feat = tf.transpose(local_feat, perm = [1,0,2])

        return local_feat

    @tf.function
    def __call__(self, imgs, pts, camera_dict, view_num=1):
        '''
        Forward pass of the network
        :param imgs: [B, H, W, C] input images, B is number of input images
        :param pts: [B, N, 3] N is number of sampling points
        :param camera_dict:  A dict contains camera matrices, cam_rot [B,3,3], cam_pos [B,1,3], cam_K [B,3,3]
        :return [B, N, 3] possibility of 3-way classification
        '''

        # transfer sampled points from world coordinates to image coordinates
        img_xyz = self.projection_shapenet(pts, camera_dict)
        point_num = pts.shape[1]

        # normalize image coordinate to [-1,1]
        img_scale = float(imgs.shape[1]) - 1
        img_xyz = tf.clip_by_value(img_xyz, 0, img_scale)
        img_u_normalized = 2 * img_xyz[:, :, 0] / img_scale - 1.0
        img_v_normalized = 2 * img_xyz[:, :, 1] / img_scale - 1.0
        img_uv_normalized = tf.concat([img_u_normalized[:, :, None], img_v_normalized[:, :, None]], axis=2)

        # compute point feature (using world coordinate)
        xyz = tf.expand_dims(pts, 1)
        point_coord_feat = self.pointConv(xyz)
        point_coord_feat = tf.squeeze(point_coord_feat)

        # encode image feature
        feat_local, feat_global = self.img_encoder(imgs)
        feat_global = tf.expand_dims(feat_global, 1)

        # compute global image feature
        feat_global = tf.tile(feat_global, [1, point_num, 1])

        # compute local image feature for points
        img_local_feat_list = [self.index(feat, img_uv_normalized) for feat in feat_local]
        final_img_local_feat = tf.concat(img_local_feat_list, 2)
        final_img_local_feat = tf.transpose(final_img_local_feat, perm=[1,0,2])

        # pred sdf from two branches
        pred1 = self.local_classifier(final_img_local_feat, point_coord_feat)
        pred2 = self.global_classifier(feat_global,  point_coord_feat)

        pred_final = tf.nn.softmax(pred1 + pred2)

        return pred_final
