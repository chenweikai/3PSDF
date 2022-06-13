from src.model.dataLoader import Dataloader

if __name__ == '__main__':
    # demo script to generate tfrecord of shapenet data for training
    category_list = ['02691156', '03001627', '02958343', '04530566', '03636649']

    sdf_input_dir = ''
    sdf_tfrecord_output_dir = ''

    image_input_dir = ''
    image_tfrecord_output_dir = ''

    camera_input_dir = ''
    camera_tfrecord_output_dir = ''

    for category in category_list:
        Dataloader.create_ShapeNet_SDF_TFRecord(data_dir=sdf_input_dir + category,
                                                file_ext="sdf",
                                                out_dir=sdf_tfrecord_output_dir + category)

        Dataloader.create_ShapeNet_IMG_TFRecord(data_dir=image_input_dir,
                                                file_ext="png",
                                                out_dir=image_tfrecord_output_dir + category)

        Dataloader.create_ShapeNet_CAM_TFRecord(data_dir=camera_input_dir + category,
                                                out_dir=camera_tfrecord_output_dir + category)
