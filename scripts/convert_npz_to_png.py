#!/usr/bin/env python

import cv2
import numpy as np
import os.path as osp


def convert(npz_file):
    if not osp.exists(npz_file):
        print('File [{}] does not exist.'.format(npz_file))
        return
    depth_array = np.load(npz_file)['arr_0'] * 1000
    if depth_array.dtype != np.uint16:
        depth_array_uint16 = depth_array.astype(np.uint16)
    else:
        depth_array_uint16 = depth_array.copy()
    png_file = "depth.png"
    cv2.imwrite(png_file, depth_array_uint16)


if __name__ == '__main__':
    convert('depth.npz')
