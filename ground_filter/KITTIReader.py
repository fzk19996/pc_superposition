import numpy as np

def load_velo_scan(file):
    '''Load and parse a velodyne binary file'''
    scan = np.fromfile(file, dtype=np.float32)
    return scan.reshape((-1, 4))


def yield_velo_scans(velo_files):
    '''
    input:  velo_files (a file path list)
    output: Lidar point cloud iterator
    '''
    for file in velo_files:
        yield load_velo_scan(file)
