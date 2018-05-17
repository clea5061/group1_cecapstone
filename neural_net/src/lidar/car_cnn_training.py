import tensorflow as tf
import cv2
import numpy as np

from cnn import lidar_car
from dataset import load_data

path = r'/media/deepLearn/training_data/lidar_train_set'

num_epochs = 100
batch_size = 100

train_x, train_y, test_x, test_y = load_data(path, 0.1)

train_x = train_x[0:-1*(len(train_x) % batch_size)]
train_y = train_y[0:-1*(len(train_y) % batch_size)]
test_x = test_x[0:-1*(len(test_x) % batch_size)]
test_y = test_y[0:-1*(len(test_y) % batch_size)]

batches_per_epoch = int(len(train_x)/batch_size)

sess = tf.InteractiveSession()
model = lidar_car()
train_step = tf.train.AdamOptimizer(1e-4).minimize(model.loss)
saver = tf.train.Saver()
sess.run(tf.global_variables_initializer())

for i in range(num_epochs):

    for j in range(0, batches_per_epoch):
        batch = [train_x[j*batch_size:j*batch_size+batch_size], train_y[j*batch_size:j*batch_size+batch_size]]
        batch_ = [[],[]]
        for k in range(len(batch[0])):
            batch_[0].append(batch[0][j].astype(dtype=np.float32)/255.0)
            batch_[1].append(np.array([batch[1][j][0]], dtype=np.float32))
        batch = batch_
        train_step.run(feed_dict={model.x:batch[0], model.y_:batch[1], model.keep_prob_fc1:0.8, model.keep_prob_fc2:0.8})
    if i % 5 == 0:
        test_error = 0.0
        for b in range(0, len(test_x), batch_size):
            batch = [test_x[b:b + batch_size], test_y[b:b + batch_size]]
            # --- normalize batch ---
            batch_ = [[], []]
            for j in range(len(batch[0])):
                batch_[0].append(batch[0][j].astype(dtype=np.float32) / 255.0)
                batch_[1].append(np.array([batch[1][j][0]], dtype=np.float32))
            batch = batch_
            # print('batch =', len(batch[0]), len(batch[1]))
            test_error_ = model.loss.eval(feed_dict={model.x: batch[0], model.y_: batch[1],
                                                     model.keep_prob_fc1: 1.0,
                                                     model.keep_prob_fc2: 1.0})
            # -----------------------
            test_error += test_error_
        test_error /= len(test_x) / batch_size
        test_accuracy = 1.0 - test_error
        print("test accuracy %g" % test_accuracy)

filename = saver.save(sess, path + '/model.ckpt')
