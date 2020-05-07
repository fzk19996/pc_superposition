import numpy as np
from mayavi import mlab

def plot_pointClouds(pointClouds):
    x = pointClouds[:, 0]  # x position of point
    y = pointClouds[:, 1]  # y position of point
    z = pointClouds[:, 2]  # z position of point
    d = pointClouds[:, 3]  # if -1000 ground
    maxD = np.max(d)
    print(maxD)
    minD = np.min(d)
    print(minD)
    #d = (70) * (d-minD)
    #d = np.where(d>40, 70, 20)
    maxD = np.max(d)
    print(maxD)
    minD = np.min(d)
    print(minD)
    fig = mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mlab.points3d(x, y, z, d, mode="point", colormap='spectral', figure=fig)
    mlab.show()

