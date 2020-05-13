import numpy as np
# from open3d import *
import open3d
import os
import array
import yaml

sequence = '04'
index = '000000'
kitti_path = '/home/fzkgod/dataset/dataset/kittivo/sequences/'
predict_path = '/home/fzkgod/src/net/RandLA-Net-master/test/sequences/'
calib_path = os.path.join(kitti_path, sequence, 'calib.txt')
bin_path = os.path.join(kitti_path,sequence,'velodyne', index+'.bin')
img_path = os.path.join(kitti_path, sequence, 'image_2', index+'.png')
# label_path = os.path.join(kitti_path, sequence, 'labels', index+'.label')
label_path = os.path.join('/home/fzkgod/dataset/dataset/kittivo/sequences/04/labels',index+'.label')
with open(bin_path, 'rb') as bin_file:
    pc = array.array('f')
    pc.frombytes(bin_file.read())
    pc = np.array(pc).reshape(-1, 4)
predict_label_path = os.path.join(predict_path,sequence,'predictions',index+'.label')
label = np.fromfile(label_path, dtype=np.uint32)
CFG = yaml.safe_load(open('config/semantic-kitti.yaml', 'r'))
colors = np.zeros((label.shape[0],3))
d = np.where(label==40)
for i in range(colors.shape[0]):
    # if label[i] == 40:
    #     colors[i] = [0, 0, 255]
    # else:
    #     colors[i] = [255,0,0]
    colors[i] = CFG['color_map'][label[i]]
point_cloud = open3d.PointCloud()
point_cloud.colors = open3d.Vector3dVector(colors)
point_cloud.points = open3d.Vector3dVector(pc[:,0:3])
open3d.draw_geometries([point_cloud])
print('end')

