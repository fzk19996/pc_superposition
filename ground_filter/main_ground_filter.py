import os
import numpy as np
import KITTIReader as reader
import preProcessor
import glob
import yaml
import open3d

kitti_path = '/home/fzkgod/dataset/kittivo/sequences/'
infile_path = os.path.join(kitti_path, "04", "velodyne", "*.bin")
file_list = sorted(glob.glob(infile_path))
#print(file_list)

#for raw_data in reader.yield_velo_scans(file_list[0]):
raw_data = reader.load_velo_scan(file_list[0])
#print(raw_data.shape[0])
preProcess = preProcessor.preProcessor()

#utils.plot_pointClouds(preProcess.get_rawPointCloud())
#utils.plot_pointClouds(preProcess.ground_filter_linefit(raw_data, 15))
# utils.plot_pointClouds(preProcess.ground_filter_heightDiff(raw_data, 400, 400, 0.3, 0.5))
_, pc = preProcess.ground_filter_heightDiff(raw_data, 400, 400, 0.3, 0.5)
CFG = yaml.safe_load(open('config/semantic-kitti.yaml', 'r'))
colors = np.zeros((pc.shape[0],3))
d = pc[:, 3]
tmp = np.where(d<=0)
d = np.where(d>0, 70, 20)
colors[:] = [255,0,0]
colors[tmp] = [0,0,255]
# for i in range(colors.shape[0]):
#     if d[i] == 20:
#         colors[i] = [0, 0, 255]
#     else:
#         colors[i] = [255,0,0]
    
    # colors[i] = CFG['color_map'][label[i]]
point_cloud = open3d.PointCloud()
point_cloud.colors = open3d.Vector3dVector(colors)
point_cloud.points = open3d.Vector3dVector(pc[:,0:3])
open3d.draw_geometries([point_cloud])
