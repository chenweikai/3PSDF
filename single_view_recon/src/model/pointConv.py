"""Class definition of point convolutional network for extracting 3D point features."""


from tensorflow.keras.layers import Conv1D, ReLU, BatchNormalization
from tensorflow.keras import Model


class PointConv(Model):
    def __init__(self):
        super(PointConv, self).__init__()
        self.conv1 = Conv1D(filters = 64, kernel_size = 1, strides = 1, activation = None)
        self.bn1 = BatchNormalization()
        self.relu1 = ReLU()

        self.conv2 = Conv1D(filters = 256, kernel_size = 1, strides = 1, activation = None)
        self.bn2 = BatchNormalization()
        self.relu2 = ReLU()

        self.conv3 = Conv1D(filters = 512, kernel_size = 1, strides = 1, activation = None)
        self.bn3 = BatchNormalization()
        self.relu3 = ReLU()

        self.conv4 = Conv1D(filters = 768, kernel_size = 1, strides = 1, activation = None)
        self.bn4 = BatchNormalization()
        self.relu4 = ReLU()

    def call(self, x):
        x = self.conv1(x)
        x = self.relu1(self.bn1(x))
        x = self.conv2(x)
        x = self.relu2(self.bn2(x))
        x = self.conv3(x)
        x = self.relu3(self.bn3(x))
        x = self.conv4(x)
        x = self.bn4(x)
        return x
