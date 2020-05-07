import multiprocessing
import os

import numpy as np
import array
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import cv2
from scipy.interpolate import griddata, Rbf, interpolate
from PIL import Image

def color_map(pc, img_size):
    """
    将前视投影之后的点云绘制到image上
    :param pc: 输入点云
    :param img_size: 图像大小
    :return:
    """
    # 构建mask
    mask = np.zeros([img_size[1], img_size[0]], dtype=np.uint8)
    mask[np.int_(pc[:, 1]), np.int_(pc[:, 0])] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 泛洪算法 填补图像中的空洞
    flood_fill = mask.copy().astype(np.uint8)
    cv2.floodFill(flood_fill, np.zeros((img_size[1] + 2, img_size[0] + 2), np.uint8), (0, 0), 255)
    mask = mask | cv2.bitwise_not(flood_fill)

    # mask = cv2.bitwise_not(mask)
    # contour, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # for cnt in contour:
    #     cv2.drawContours(mask, [cnt], 0, 255, -1)
    # mask = cv2.bitwise_not(mask)
    #
    # cv2.imshow('', mask)
    # cv2.waitKey(0)

    # colors = plt.get_cmap('gist_ncar_r')(np.maximum((100 - pc[:, 2]) / 100, 0))
    colors = plt.get_cmap('hot')(1.5 * pc[:, 3] ** 1.3)
    grid_x, grid_y = np.mgrid[0:img_size[0]:1, 0:img_size[1]:1]
    chs = [griddata(pc[:, 0:2], colors[:, 2 - idx], (grid_x, grid_y), method='linear').T for idx in range(3)]
    img = np.stack(chs, axis=-1)
    img = np.int_(img * 255)

    # 和mask向掩码
    img = img * np.expand_dims(mask / 255, -1)
    return img

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

def load_calib(calib_file_path):
    """
    load calibration file(KITTI object format)
    :param calib_file_path:
    :return:
    """
    calib_file = open(calib_file_path, 'r').readlines()
    calib_file = [line
                      .replace('Tr_velo_to_cam', 'Tr_velo_cam')
                      .replace('R0_rect', 'R_rect')
                      .replace('\n', '')
                      .replace(':', '')
                      .split(' ')
                  for line in calib_file]
    calib_file = {line[0]: [float(item) for item in line[1:] if item != ''] for line in calib_file if len(line) > 1}
    return calib_file


def parse_calib_file(calib_file):
    """
    parse calibration file to calibration matrix
    :param calib_file:
    :return:
    """

    # 外参矩阵
    Tr_velo_cam = np.array(calib_file['Tr_velo_cam']).reshape(3, 4)
    Tr_velo_cam = np.concatenate([Tr_velo_cam, [[0, 0, 0, 1]]], axis=0)
    # 矫正矩阵
    R_rect = np.array(calib_file['R_rect']).reshape(3, 3)
    R_rect = np.pad(R_rect, [[0, 1], [0, 1]], mode='constant')
    R_rect[-1, -1] = 1
    # 内参矩阵
    P2 = np.array(calib_file['P2']).reshape(3, 4)

    return np.matmul(np.matmul(P2, R_rect), Tr_velo_cam)
  
  
def project_lidar_to_image(pc, img_size, calib_file, label, yaw_deg):
    """
    获取点云的前视投影
    :param pc: 输入点云(N, 4)
    :param img_size: (w, h)
    :param calib_file: KITTI calib文件的path
    :param yaw_deg: 将点云按z轴旋转的角度，默认设置为0就可以
    :return:
    """
    yaw_deg = yaw_deg / 180 * np.pi
    calib_mat = parse_calib_file(calib_file)
    # calib_mat = parse_calib_file(load_calib(calib_file))

    # 投影
    intensity = np.copy(pc[:, 3]).reshape(-1, 1)
    height = np.copy(pc[:, 2]).reshape(-1, 1)
    pc[:, 3] = 1
    # yaw旋转
    rotate_mat = np.array([
        [np.cos(yaw_deg), -np.sin(yaw_deg), 0, 0],
        [np.sin(yaw_deg), np.cos(yaw_deg), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    pc = np.matmul(rotate_mat, pc.T).T

    # 限制前视 并且限制fov在90度之内
    # 计算fov
    fov_h = np.arctan(np.abs(pc[:, 1] / pc[:, 0]))
    fov_v = np.arctan(np.abs(pc[:, 2] / pc[:, 0]))
    indice = np.where(np.bitwise_and(
        pc[:, 0] > 0.5,
        np.bitwise_and(fov_h < np.pi / 4, fov_v < np.pi / 10, )
    ))
    pc = pc[indice]
    print(pc.shape)

    # mlab.points3d(pc[:, 0], pc[:, 1], pc[:, 2], color=(1, 0, 0), mode='point')
    # pc = pc[np.random.permutation(len(pc))[:28000], :]
    # mlab.points3d(pc[:, 0], pc[:, 1], pc[:, 2], color=(0, 1, 0), mode='point')
    # mlab.show()

    intensity = intensity[indice]
    height = height[indice]
    label = label[indice]
    label=label[:, np.newaxis]

    # 进行投影变换
    pc = np.matmul(calib_mat, pc.T).T
    
    # z深度归一化
    pc[:, :2] /= pc[:, 2:]
    
    # 还原intensity
    pc = np.concatenate([pc, intensity, height, label], axis=1)
    
    # 按照原图大小裁剪
    pc = pc[np.where(pc[:, 0] >= 0)]
    pc = pc[np.where(pc[:, 0] < img_size[0])]
    pc = pc[np.where(pc[:, 1] >= 0)]
    pc = pc[np.where(pc[:, 1] < img_size[1])]
        
    # 方法1 对点云做color map之后再绘制到前视图上
    # img = color_map(pc, img_size)
    
    # 方法2 下边是不对投影之后的点云做任何处理，直接以点的形式绘制到前视图上
    img = np.zeros([375, 1242, 3])
    # # BGR
    tmp1 = np.int_(pc[:, 1])
    img[np.int_(pc[:, 1]), np.int_(pc[:, 0]), 2] = pc[:, 2] / 60 # depth
    img[np.int_(pc[:, 1]), np.int_(pc[:, 0]), 1] = (pc[:, 3] + 0.1) # intensity

    lane_pc = np.where(pc[:, 5]==70)
    lane_point = np.zeros((pc.shape[0],))
    lane_point[lane_pc] = 1
    img[np.int_(pc[:, 1]), np.int_(pc[:, 0]), 0] = lane_point
    img = np.int_(img * 255)
    
    return img
  
if __name__ == '__main__':
    # object
    yaw_deg = 0
    object_origin_dataroot = '/path/to/KITTI_object_root/{}/'
    for pc_file in os.listdir(object_origin_dataroot.format('velodyne')):
        # 读取标定文件
        calib_file_path = os.path.join(object_origin_dataroot.format('calib'), pc_file[:-4] + '.txt')
        calib_file = pointcloud.load_calib(calib_file_path)
        # 读取lidar
        bin_file_path = os.path.join(object_origin_dataroot.format('velodyne'), pc_file)
        # 读取图像尺寸
        origin = cv2.imread(os.path.join(object_origin_dataroot.format('image_2'), pc_file[:-4] + '.png'))
        img_size = origin.T.shape[1:]
        # img_size = Image.open(os.path.join(object_origin_dataroot.format('image_2'), pc_file[:-4] + '.png')).size
        print(bin_file_path, img_size)
        # 投影
        pc = load_pc(bin_file_path)
        img = project_lidar_to_image(pc, img_size, calib_file, yaw_deg=yaw_deg)
        # 裁剪到仅有lidar的部分
        img = img[120:, ...]

        # cv2.imshow('', origin)
        # cv2.waitKey(0)
        # plt.imshow(img)
        # plt.show()
        # break
        cv2.imshow('', img.astype(np.uint8))
        cv2.waitKey(0)
        # cv2.imwrite(
        #     # '/media/mdisk/hvt/cgan/object_linear_crop/train_A/{}.png'.format(pc_file[:-4]), img,
        #     # '/media/hvt/95f846d8-d39c-4a04-8b28-030feb1957c6/dataset/cgan/object_linear_crop_testing/train_A/{}.png'.format(
        #     '/media/hvt/95f846d8-d39c-4a04-8b28-030feb1957c6/dataset/cgan/tracking_linear_crop/test_A/{}.png'.format(
        #         pc_file[:-4]), img,
        #     [cv2.IMWRITE_PNG_COMPRESSION, 0]  # 原图质量
        # )

        # # 保存裁剪之后的原图
        # cv2.imwrite(
        #     '/media/mdisk/hvt/cgan/object_linear_crop/train_B/{}.png'.format(pc_file[:-4]), origin[120:, ...],
        #     [cv2.IMWRITE_PNG_COMPRESSION, 0]  # 原图质量
        # )