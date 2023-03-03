import os
import glob
import open3d as o3d
import numpy as np
import pandas as pd


def find_filelist(dir, file_ext):
    ls = []
    os.chdir(dir)
    for file in glob.glob("*"+file_ext):
        ls.append(file)
    return ls


def find_data(dir, data_sort):
    os.chdir(dir)
    filename = str(glob.glob("*"+data_sort+".obj"))
    return filename[2:-2]


def find_framelist(dir):
    ls = [x for x in sorted(os.listdir(dir))]
    return ls


def read_pclobj(dir, filename):
    """
    Convert .obj point cloud to np.array
    """
    data_pd = pd.read_csv(
        os.path.join(dir, filename), header=None, delimiter=' ')
    data_np = data_pd.to_numpy()
    return data_np[:, 1:]


def read_verts(obj_filename):
    """
    Convert .obj bbox to np.array of 8 vertexes
    """
    mesh = o3d.io.read_triangle_mesh(obj_filename)
    return np.asarray(mesh.vertices)
