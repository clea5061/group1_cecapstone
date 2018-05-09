import os
import cv2

def load_data(path, percent_testing=None):
    assert percent_testing is None or (percent_testing >= 0.0 and percent_testing <= 1.0)
    x, y, fnames = [], [], []
    for i in os.walk(path):
        (d, sub_dirs, files_) = i
        fnames.extend(files_)
        print(i)
    seq_fname = []
    for fname in fnames:
        seq = str(fname.split('_')[0])
        seq_fname.append((seq, fname))
    seq_fname.sort()
    for (seq, fname) in seq_fname:
        img = cv2.imread(path + '/' + fname)
        img = cv2.resize(img, (200, 150), interpolation=cv2.INTER_CUBIC)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for black and white
        img = img[35:,:,:]
        x.append(img)
        timestamp, steering, throttle = fname.split('_')
        y.append((steering, throttle))
    train_x, train_y, test_x, test_y = [], [], [], []
    if percent_testing is not None:
        tst_strt = int(len(x)*(1.0-percent_testing))
        train_x, train_y, test_x, test_y = x[:tst_strt], y[:tst_strt], x[tst_strt:], y[tst_strt:]
    else:
        train_x, train_y = x, y
    return train_x, train_y, test_x, test_y
