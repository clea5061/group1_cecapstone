import tensorflow as tf
from tensorflow_util import conv_layer, fc_layer, flattened
from tensorflow_util import weight_variable, bias_variable


class cnn_car(object):

    def __init__(self):
        self.x = tf.placeholder(tf.float32, [None, 115, 200, 3])
        self.y_ = tf.placeholder(tf.float32, [None, 1])
        (self.h_conv1, _) = conv_layer(self.x, conv=(5, 5), stride=2, n_filters=24, use_bias=True)
        (self.h_conv2, _) = conv_layer(self.h_conv1, conv=(5, 5), stride=2, n_filters=36, use_bias=True)
        (self.h_conv3, _) = conv_layer(self.h_conv2, conv=(5, 5), stride=2, n_filters=48, use_bias=True)
        (self.h_conv4, _) = conv_layer(self.h_conv3, conv=(3, 3), stride=1, n_filters=64, use_bias=True)
        (self.h_conv5, _) = conv_layer(self.h_conv4, conv=(3, 3), stride=1, n_filters=64, use_bias=True)
        self.h_conv5_flat = flattened(self.h_conv5)
        (self.h_fc1_drop, _, _, self.keep_prob_fc1) = fc_layer(x=self.h_conv5_flat, n_neurons=512, activation=tf.nn.relu, use_bias=True, dropout=True)
        (self.h_fc2_drop, _, _, self.keep_prob_fc2) = fc_layer(self.h_fc1_drop, 100, tf.nn.relu, True, True)
        (self.h_fc3_drop, _, _, self.keep_prob_fc3) = fc_layer(self.h_fc2_drop, 50, tf.nn.relu, True, True)
        (self.h_fc4_drop, _, _, self.keep_prob_fc4) = fc_layer(self.h_fc3_drop, 10, tf.nn.relu, True, True)
        W_fc5 = weight_variable([10, 1])
        b_fc5 = bias_variable([1])
        self.y_out = tf.matmul(self.h_fc4_drop, W_fc5) + b_fc5
        self.loss = tf.reduce_mean(tf.abs(tf.subtract(self.y_, self.y_out)))
