from turtle import width
import cv2
import numpy as np
import matplotlib.pyplot as plt

import torch
from shapely.geometry import Polygon
from operator import itemgetter
import open3d.visualization as o3d

# add project directory to python path to enable relative imports
import os
import sys
import zlib
import math
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2
import misc.objdet_tools as tools

def load_range_image(frame, lidar_name):
    # get laser data structure from frame
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0]
    ri = []
    if len(lidar.ri_return1.range_image_compressed) > 0:
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    return ri

def my_percentile(data, percentile):
    n = len(data)
    p = n * percentile / 100
    if p.is_integer():
        return sorted(data)[int(p)]
    else:
        return sorted(data)[int(math.ceil(p)) - 1]

def show_range_image(frame, lidar_name):
    # load the data from the function
    ri = load_range_image(frame, lidar_name)

    # extract the range image from frame and map value range to 8-bit
    ri[ri<0]=0.0
    ri_range = ri[:,:,0]
    ri_range = ri_range * 255 / (np.amax(ri_range) - np.amin(ri_range))
    img_range = ri_range.astype(np.uint8)

    # map the intensity range to 8-bit scale
    ri_intensity = ri[:,:,1]

    # calculate the largest outlier and lowest outlier and normalize it
    #largest_outlier = my_percentile(ri_intensity, 99)
    largest_outlier = np.percentile(ri_intensity, 99)
    lowest_outlier = np.percentile(ri_intensity, 1)
    ri_intensity = np.clip(ri_intensity, lowest_outlier, largest_outlier) * 255 / largest_outlier

    # convert the image intensity to 8-bit
    img_intensity = ri_intensity.astype(np.uint8)

    # stack the range and the image and convert the result to unsigned 8-bit integer
    img_range_intensity = np.vstack((img_range, img_intensity))
    img_range_intensity = img_range_intensity.astype(np.uint8)

    # focus on +/- 90° around the image center
    deg90_range = int(img_range_intensity.shape[1] / 4)
    ri_center_range = int(img_range_intensity.shape[1] / 2)
    img_range_intensity = img_range_intensity[:, ri_center_range - deg90_range:ri_center_range + deg90_range]

    return img_range_intensity
    
def show_pcl(pcl, vis=True):
    # setting the view window and callback key
    vis_window = o3d.VisualizerWithKeyCallback()
    vis_window.create_window(width=1280, height=980)
    