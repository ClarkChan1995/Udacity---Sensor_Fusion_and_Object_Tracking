from turtle import width
import cv2
import numpy as np
import matplotlib.pyplot as plt

import torch
from shapely.geometry import Polygon
from operator import itemgetter
import open3d.visualization as o3d_vis
import open3d as o3d

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
    
def show_pcl(pcl):
    # setting the view window and callback key
    vis_window = o3d_vis.VisualizerWithKeyCallback()
    vis_window.create_window(width=1280, height=980)

    global next
    next = True
    def key_callback(vis_window):
        global next
        print("Press Q for the next")
        next = False
        return
    
    # register Q as the switch key
    vis_window.register_key_callback(81, key_callback)
    # create the instance of 3d-point cloud and convert into 3d vectors
    vis_pcd = o3d.geometry.PointCloud()
    vis_pcd.points = o3d.utility.Vector3dVector(pcl[:, :3])\
    # display the frame one by one
    vis_window.add_geometry(vis_pcd)

    while next:
        vis_window.poll_events()
        vis_window.update_renderer()
        
def bev_from_pcl(lidar_pcl, configs):
    # setup for BEV
    # get the lidar points only in between detection area, the rest will be deleted
    area = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[area]

    # avoid flipping for neighbouring pixels more than 255
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]

    # Convert sensor coordinates to bev-map coordinates (ID_S2_EX1) #
    # compute the discretization of x-range with respective height
    x_discrete = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    lidar_pcl_copy = np.copy(lidar_pcl)
    lidar_pcl_copy[:, 0] = np.int_(np.floor(lidar_pcl_copy[:, 0] / x_discrete)) # transform x-coord

    lidar_pcl_copy[:, 1] = np.int_(np.floor(lidar_pcl_copy[:, 1] / x_discrete) + (configs.bev_width + 1) /2) # tranform y-coord
    lidar_pcl_copy[:, 1] = np.abs(lidar_pcl_copy[:, 1]) # cancel all the negative values

    #show_pcl(lidar_pcl_copy) # display the frame by using the showing function above

    # Compute intensity layer of bev-map (ID_S2_EX2) #
    # using numpy function to create a same dimesion as BEV map where zeros are inside
    intensity_map = np.zeros((configs.bev_height, configs.bev_width))

    # using lexsort to rearrange the elements [-z, y, x]
    lidar_pcl_copy[lidar_pcl_copy[:, 3]>1.0, 3] = 1.0
    intensity = np.lexsort((-lidar_pcl_copy[:, 2], lidar_pcl_copy[:, 1], lidar_pcl_copy[:, 0]))
    lidar_pcl_top = lidar_pcl_copy[intensity]

    # select the most top value of z-coord with respective x-coord and y-coord
    lidar_pcl_num, lidar_pcl_indices, lidar_pcl_count = np.unique(lidar_pcl_copy[:, 0:2], axis=0, return_index=True, return_counts=True)
    lidar_pcl_top = lidar_pcl_copy[lidar_pcl_indices]

    # normalize the intensity by the difference of maximum and minimum calue of point cloud
    intensity_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = lidar_pcl_top[:, 3] / (np.amax(lidar_pcl_top[:, 3]) - np.amin(lidar_pcl_top[:, 3]))

    # display the intensity map
    img = intensity_map * 256
    img = img.astype("uint8")
    '''
    cv2.imshow("Intensity Map", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''

    # Compute height layer of bev-map (ID_S2_EX3) #
	# using numpy function to create an empty array same as BEV map
    height_map = np.zeros((configs.bev_height, configs.bev_width))

    # normalize the height with defined upper and lower height in the config file
    height_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = lidar_pcl_top[:, 2] / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))

    # visualize the intensity map (height_map)
    img_height = height_map * 256
    img_height = img_height.astype("uint8")
    '''
    cv2.imshow("Height_map", img_height)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''

    # TODO remove after implementing all of the above steps
    #lidar_pcl_copy = []
    #lidar_pcl_top = []
    #height_map = []
    #intensity_map = []
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_copy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalized = np.minimum(1.0, np.log(counts + 1)/ np.log(64))
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalized

    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]

    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps