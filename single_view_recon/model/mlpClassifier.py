from __future__ import absolute_import, division, print_function, unicode_literals
import tensorflow as tf
from tensorflow.keras.layers import Dense, Flatten, Dropout, BatchNormalization, ReLU
from tensorflow.keras import Model


class Classifier(Model):
    def __init__(self):
        super(Classifier, self).__init__()
        self.layer_add1 = Dense(1024, activation=None, trainable=True, name="dense_add1")
        self.bn_a1 = BatchNormalization()
        self.relu_a1 = ReLU()

        self.layer1 = Dense(1024, activation=None, trainable=True, name="dense4")
        self.bn1 = BatchNormalization()
        self.relu1 = ReLU()

        self.layer_add2 = Dense(768, activation=None, trainable=True, name="dense_add2")
        self.bn_a2 = BatchNormalization()
        self.relu_a2 = ReLU()

        self.layer2 = Dense(512, activation=None, trainable=True, name="dense5")
        self.bn2 = BatchNormalization()
        self.relu2 = ReLU()

        self.layer3 = Dense(256, activation=None, trainable=True, name="dense6")
        self.bn3 = BatchNormalization()
        self.relu3 = ReLU()

        self.layer4 = Dense(128, activation=None, trainable=True, name="dense7")
        self.bn4 = BatchNormalization()
        self.relu4 = ReLU()

        self.layer5 = Dense(3, activation=None, trainable=True, name="dense8")

    def call(self, img_feat, point_feat):
        x = tf.keras.layers.Concatenate(axis=2)([img_feat, point_feat])
        x = self.layer_add1(x)
        x = self.relu_a1(self.bn_a1(x))

        x = self.layer1(x)
        x = self.relu1(self.bn1(x))

        x = self.layer_add2(x)
        x = self.relu_a2(self.bn_a2(x))

        x = self.layer2(x)
        x = self.relu2(self.bn2(x))

        x = self.layer3(x)
        x = self.relu3(self.bn3(x))

        x = self.layer4(x)
        x = self.relu4(self.bn4(x))

        x = self.layer5(x)

        return x
