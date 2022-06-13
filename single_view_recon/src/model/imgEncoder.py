from __future__ import absolute_import, division, print_function, unicode_literals
from tensorflow.keras import layers, models, Input
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Dense, Flatten, Dropout, ReLU, BatchNormalization, GlobalAveragePooling2D


def ImageEncoder():

    input_image = Input(shape=(224, 224, 4))

    # Block1
    x = Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block1_conv1')(input_image)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block1_conv2')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x1 = MaxPooling2D(pool_size=(2, 2), strides=(2, 2))(x)

    # Block2
    x = Conv2D(filters=128, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block2_conv1')(x1)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=128, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block2_conv2')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x2 = MaxPooling2D(pool_size=(2, 2), strides=(2, 2))(x)

    # Block3
    x = Conv2D(filters=256, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block3_conv1')(x2)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=256, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block3_conv2')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=256, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block3_conv3')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=256, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block3_conv4')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x3 = MaxPooling2D(pool_size=(2, 2), strides=(2, 2))(x)

    # Block4
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block4_conv1')(x3)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block4_conv2')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block4_conv3')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block4_conv4')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x4 = MaxPooling2D(pool_size=(2, 2), strides=(2, 2))(x)

    # Block5
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block5_conv1')(x4)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block5_conv2')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block5_conv3')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x = Conv2D(filters=512, kernel_size=(3, 3), strides=(1, 1), padding='same', name='block5_conv4')(x)
    x = BatchNormalization()(x)
    x = ReLU()(x)
    x5 = MaxPooling2D(pool_size=(2, 2), strides=(2, 2))(x)

    # Global features
    x = GlobalAveragePooling2D()(x5)
    x = Dense(units=1024, activation= None, name='output_predictions')(x)

    # Output global and local features
    model = Model(inputs=input_image, outputs=([x1, x2, x3, x4, x5], x), name='Classifier')

    return model
