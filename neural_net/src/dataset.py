from functools import partial
from multiprocessing import Pool
import os
import cv2

def conv_image(path, fname):
        img = cv2.imread(path + '/' + fname)
        img = cv2.resize(img, (200, 150), interpolation=cv2.INTER_CUBIC)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for black and white
        img = img[35:,:,:]
        _, timestamp, steering, throttle = fname.split('_')
        return (img, (steering, throttle))

def load_data(path, percent_testing=None):
    assert percent_testing is None or (percent_testing >= 0.0 and percent_testing <= 1.0)
    x, y, fnames = [], [], []
    for i in os.walk(path):
        (d, sub_dirs, files_) = i
        fnames.extend(files_)
        print(i)
    seq_fname = []
    for fname in fnames:
        if not fname.endswith('jpeg'):
            continue
        seq = str(fname.split('_')[0])
        seq_fname.append((seq, fname))
    seq_fname.sort()

    #Create thread pool to do image pre-processing
    pool = Pool(4)
    func = partial(conv_image, path)
    #Execute the training images on the thread pool
    data = pool.map(func, (x for _, x in seq_fname))
    for (i, l) in data:
        x.append(i)
        y.append(l)

    train_x, train_y, test_x, test_y = [], [], [], []
    if percent_testing is not None:
        tst_strt = int(len(x)*(1.0-percent_testing))
        train_x, train_y, test_x, test_y = x[:tst_strt], y[:tst_strt], x[tst_strt:], y[tst_strt:]
    else:
        train_x, train_y = x, y
    return train_x, train_y, test_x, test_y
