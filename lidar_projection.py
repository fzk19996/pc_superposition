import Kitti_projection
import cv2
import numpy as np
import os

yaw_deg = 0

if __name__ == '__main__':
    sequence = '04'
    index = '000000'
    kitti_path = '/home/fzkgod/dataset/kittivo/sequences/'
    calib_path = os.path.join(kitti_path, sequence, 'calib.txt')
    bin_path = os.path.join(kitti_path,sequence,'volodyne', index+'.bin')
    img_path = os.path.join(kitti_path, sequence, 'image_2', index+'.png')
    label_path = os.path.join(kitti_path, sequence, 'labels', index+'.label')
    label = np.fromfile(label_path, dtype=np.uint32)
    lane_idx = np.where(label==40)
    calib = KITTI_projection.load_calib(calib_path)
    pc = KITTI_projection.load_pc(bin_path)
    img = cv2.imread(img_path)
    img_size = img.T.shape[1:]
    img_lidar_project = KITTI_projection.project_lidar_to_image(pc, img_size, calib, label, yaw_deg=yaw_deg)
    img_lidar_project = img_lidar_project[120:, ...]
    cv2.imshow('', img_lidar_project.astype(np.uint8))
    cv2.waitKey(0)
    print('end')