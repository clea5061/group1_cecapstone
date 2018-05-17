import tensorflow as tf
from tensorflow_util import conv_layer, fc_layer, flattened
from tensorflow_util import weight_variable, bias_variable

class lidar_car(object):

    def __init__(self):
        self.x = tf.placeholder(tf.float32, [None, 100, 100, 1])
        self.y_ = tf.placeholder(tf.float32, [None, 1])
        (self.h_conv1, _) = conv_layer(self.x, conv=(7,7), stride=1, n_filters=6, use_bias=True)
        (self.h_conv2, _) = conv_layer(self.h_conv1, conv=(5,5), stride=2, n_filters=12, use_bias=True) #50x50x8
        (self.h_conv3, _) = conv_layer(self.h_conv2, conv=(5,5), stride=2, n_filters=18, use_bias=True) #25x25x12
        (self.h_conv4, _) = conv_layer(self.h_conv3, conv=(4,4), stride=2, n_filters=24, use_bias=True) #12x12x16
        (self.h_conv5, _) = conv_layer(self.h_conv4, conv=(3,3), stride=2, n_filters=30, use_bias=True) #6x6x20
        (self.h_conv6, _) = conv_layer(self.h_conv5, conv=(3,3), stride=2, n_filters=36, use_bias=True) #3x3x20
        self.h_conv5_flat = flattened(self.h_conv6)
        (self.h_fc1, _, _, self.keep_prob_fc1) = fc_layer(self.h_conv5_flat, n_neurons=512, activation=tf.nn.relu, use_bias=True, dropout=True)
        (self.h_fc2, _, _, self.keep_prob_fc2) = fc_layer(self.h_fc1, n_neurons=256, activation=tf.nn.relu,use_bias=True, dropout=True)
        W_fcf = weight_variable([512,1])
        b_fcf = bias_variable([1])
        self.y_out = tf.matmul(self.h_fc1, W_fcf) + b_fcf
        self.loss = tf.reduce_mean(tf.abs(tf.subtract(self.y_, self.y_out)))