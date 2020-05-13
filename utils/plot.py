import open3d
import numpy as np
import os

def plot_pc(pc):
    point_cloud = open3d.PointCloud()
    point_cloud.points = open3d.Vector3dVector(pc[:,0:3])
    open3d.draw_geometries([point_cloud])

if __name__ == '__main__':
    sequence = '04'
    index = '000000'
    kitti_path = '/home/fzkgod/dataset/kittivo/sequences_train/'
    calib_path = os.path.join(kitti_path, sequence, 'calib.txt')
    pc_path = os.path.join(kitti_path,sequence,'velodyne', index+'.npy')
    pc = np.load(pc_path)
    plot_pc(pc)
    print('end')