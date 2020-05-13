import numpy as np
# from open3d import *
import open3d
import os
import array
import yaml

def load_pc(bin_file_path):
    """
    load pointcloud file (velodyne format)
    :param bin_file_path:
    :return:
    """
    with open(bin_file_path, 'rb') as bin_file:
        pc = array.array('f')
        pc.frombytes(bin_file.read())
        pc = np.array(pc).reshape(-1, 4)
        return pc

def load_calib(calib_path):
    f = open(calib_path, "r")
    line = f.readline()
    calib = {}
    while line:
        tmp = line.split(":")
        tr = tmp[1].split(" ")[1:]
        tr = list(map(eval, tr))
        m = np.identity(4)
        for j in range(12):
            m[j//4, j- (j//4)*4] = tr[j]
        calib[tmp[0]] = np.mat(m)
        line = f.readline()
    f.close()
    return calib


def load_pose(pose_path):
    f = open(pose_path,"r")
    line = f.readline()[:-1]
    poses = []
    while line:
        poses.append(np.array([float(m) for m in line.split(' ')]))
        line = f.readline()[:-1]
    f.close()
    res = []
    for i in range(len(poses)):
        pose = np.identity(4)
        for j in range(12):
            pose[j//4, j - (j//4)*4] = poses[i][j]
        res.append(pose)
    return res


if __name__ == '__main__':
    sequence = '01'
    kitti_path = '/home/fzkgod/dataset/dataset/kittivo/sequences/'
    calib_path = os.path.join(kitti_path, sequence, 'calib.txt')
    pose_path = os.path.join(kitti_path, sequence, 'poses.txt')
    pc_array = []
    CFG = yaml.safe_load(open('config/semantic-kitti.yaml', 'r'))

    poses = load_pose(pose_path)
    indexs = np.arange(10)
    calib = load_calib(calib_path)
    for i in range(len(poses)):
        Tr = calib['Tr']
        Tr = np.mat(Tr)
        Tr_inv = Tr.I
        poses[i] = np.dot(np.dot(Tr_inv, poses[i]), Tr)
    res = []
    colors = []
    for index in indexs:
        bin_path = os.path.join(kitti_path,sequence,'velodyne', str(index).zfill(6)+'.bin')
        label_path = os.path.join(kitti_path,sequence,'labels', str(index).zfill(6)+'.label')
        label = np.fromfile(label_path, dtype=np.uint32)
        pc = load_pc(bin_path)
        pc[:,3] = 1
        # pc = np.transpose(pc)
        for j in range(pc.shape[0]):
            point_tmp = pc[j]
            point_tmp = np.reshape(point_tmp, (4,1))
            # point_tmp = np.transpose(point_tmp)
            point_res = np.squeeze(np.transpose(np.dot(poses[index], point_tmp).A))
            res.append(point_res)
            # res.append(pc[j])
            if label[j] in CFG['color_map']:
                colors.append(CFG['color_map'][label[j]])
            else:
                colors.append([255,255,255])
                
        # pc = np.mat(pc)
        # pc_tmp = poses[index]*np.mat(pc)
        # res.append(np.transpose(pc_tmp.A))
    
    pc_array = np.array(res)
    # pc_array = np.concatenate(pc_array)
    point_cloud = open3d.PointCloud()
    point_cloud.colors = open3d.Vector3dVector(np.array(colors))
    point_cloud.points = open3d.Vector3dVector(pc_array[:,0:3])
    open3d.draw_geometries([point_cloud])
    print('end')