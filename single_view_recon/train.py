import datetime
import os
import tensorflow as tf
import random
import argparse
import horovod.tensorflow as hvd
from src.model.dataLoader import Dataloader
from src.model.network import DeepImpNet
from src.utils.io_utils import load_filelist


# 3DReconstruction_shapenet_mb
def parse_args():
    parser = argparse.ArgumentParser(description='3PSDF_train')
    parser.add_argument('--sdf_dir', type=str, default='data/sdf-depth7-tfrecord/',
                        help='data folder for 3psdf training samples')
    parser.add_argument('--cam_dir', type=str, default='data/cam-tfrecord/',
                        help='data folder for camera parameters')
    parser.add_argument('--img_dir', type=str, default='data/img-tfrecord/',
                        help='data folder for input images')
    parser.add_argument('--split_file', type=str, default='data/data_split/train.lst',
                        help='data list split for training')

    parser.add_argument('--batch_size', type=int, default=8,
                        help='batch size')
    parser.add_argument('--learning_rate', type=float, default=0.0001,
                        help='learning rate')
    parser.add_argument('--epoch_num', type=int, default=500,
                        help='number of epochs to train')
    parser.add_argument('--point_num', type=int, default=20000,
                        help='number of point samples for a single model')

    parser.add_argument('--save_model_step', type=int, default=2000,
                        help='frequency to save the network parameters (number of iterations)')
    parser.add_argument('--save_model_dir', type=str, default='checkpoint/',
                        help='path to save the network parameters')
    parser.add_argument('--save_log_step', type=int, default=5,
                        help='frequency to save the tensorboard logs (number of iterations)')
    parser.add_argument('--save_log_dir', type=str, default='log/',
                        help='path to save the tensorboard logs')
    parser.add_argument('--is_finetune', type=int, default=0,
                        help='1 - finetune the network, 0 - train the network from scratch')
    parser.add_argument('--model_weights_file', type=str, default='weights/3psdf_svr_weights',
                        help='path to the pretrained weights, only used if is_finetune is set to 1')
    args = parser.parse_args()

    return args


def train(args):
    sdf_dir = args.sdf_dir
    cam_dir = args.cam_dir
    img_dir = args.img_dir
    split_file = args.split_file
    flag_finetune = args.is_finetune
    batch_size = args.batch_size
    learning_rate = args.learning_rate
    epoch_num = args.epoch_num
    point_num = args.point_num
    save_model_step = args.save_model_step
    save_model_dir = args.save_model_dir
    save_log_step = args.save_log_step
    save_log_dir = args.save_log_dir
    model_weights_file = args.model_weights_file

    # set up distributed training
    hvd.init()
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if gpus:
        tf.config.experimental.set_visible_devices(gpus[hvd.local_rank()], 'GPU')
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)
    if hvd:
        print('Total workers: {}, local workers: {}'.format(hvd.size(), hvd.local_size()))
        print('Global rank: {}, local rank: {}'.format(hvd.rank(), hvd.local_rank()))

    # set up basic functions
    optimizer = tf.keras.optimizers.Adam(learning_rate=learning_rate)
    loss_func = tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False)
    train_loss_metric = tf.keras.metrics.Mean(name='train_loss')
    train_acc_metric = tf.keras.metrics.SparseCategoricalAccuracy()

    # model initialization
    if flag_finetune:
        if not os.path.exists(model_weights_file):
            print('Network weight file does not exist!')
            return
        model = tf.saved_model.load(model_weights_file)
        print('Loaded pretrained network weights successfully!')
    else:
        model = DeepImpNet()

    # set up checkpoint path and log path
    if hvd.rank() == 0:
        if not os.path.exists(save_model_dir):
            os.makedirs(save_model_dir)
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        save_log_path_current = os.path.join(save_log_dir, current_time)
        if not os.path.exists(save_log_path_current):
            os.makedirs(save_log_path_current)
        train_summary_writer = tf.summary.create_file_writer(save_log_path_current)

    # arrange data for different gpu
    all_file_list = load_filelist(split_file)
    chunk_size = int(len(all_file_list) / hvd.size())
    split_file_list = [all_file_list[i: i + chunk_size] for i in range(0, len(all_file_list), chunk_size)]
    file_list = split_file_list[hvd.rank()]

    total_steps = 0
    # start training
    for epoch in range(epoch_num):
        random.shuffle(file_list)
        # how many iterations needed
        for i in range(int(len(file_list) / batch_size)):
            # obtain the shape indices and load the batched data
            indices = [i * batch_size + n for n in range(batch_size)]
            img_batch, xyz_batch, gt_batch, camera_batch, name_batch = Dataloader.get_batched_data(file_list, indices,
                                                                                                   sdf_dir, img_dir,
                                                                                                   cam_dir, point_num)
            # training
            with tf.GradientTape() as tape:
                predictions = model(img_batch, xyz_batch, camera_batch, view_num=1)
                loss = loss_func(gt_batch, predictions)

            if flag_finetune:
                # finetune the network
                tape = hvd.DistributedGradientTape(tape)
                variables = tape.watched_variables()
                gradients = tape.gradient(loss, variables)
                optimizer.apply_gradients(zip(gradients, variables))
            else:
                # train network normally
                tape = hvd.DistributedGradientTape(tape)
                gradients = tape.gradient(loss, model.trainable_variables)
                optimizer.apply_gradients(zip(gradients, model.trainable_variables))

            if epoch == 0 and total_steps == 0:
                hvd.broadcast_variables(tape.watched_variables(), root_rank=0)
                hvd.broadcast_variables(optimizer.variables(), root_rank=0)

            train_loss_metric(loss)
            train_acc_metric(gt_batch, predictions)

            # print log, save to tensorboard, save checkpoint network weights
            total_steps += 1
            if hvd.local_rank() == 0:
                output_info = 'Epoch {}, Step {}, Loss: {}, Accuracy: {}'
                print(output_info.format(epoch + 1, total_steps, train_loss_metric.result(),
                                         train_acc_metric.result() * 100))

            if hvd.rank() == 0 and total_steps % save_log_step == 0:
                with train_summary_writer.as_default():
                    tf.summary.scalar('Loss', train_loss_metric.result(), step=total_steps)
                    tf.summary.scalar('Accuracy', train_acc_metric.result() * 100, step=total_steps)

            if hvd.rank() == 0 and total_steps % save_model_step == 0:
                filename = os.path.join(save_model_dir, "model_checkpoint")
                filename = filename + "_step{}_hvd{}".format(total_steps, hvd.rank())
                tf.saved_model.save(model, filename)
                print("Saved checkpoint for step {}: {}".format(total_steps, filename))

            train_loss_metric.reset_states()
        train_acc_metric.reset_states()


if __name__ == '__main__':
    # parse arguments
    args = parse_args()
    train(args)
