#!/usr/bin/env python

import plyfile
import os
import numpy as np
import tf_conversions

import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, Vector3

import argparse
import yaml
import binvox_rw

from process_reachability_data_from_csv import load_reachability_data_from_dir, generate_SDF_from_binary_space, \
    save_reachability_space_to_dir
from reachability_experiments_mongo.msg import SceneObject


def binvox_to_points(vox):
    """
    Takines a binvox_rw.binvox_rw.Voxels, and converts it to a nx3 numpy array
    representing the occupied voxels as a pointcloud.
    :type vox: binvox_rw.binvox_rw.Voxels
    :rtype numpy.ndarray
    """

    # get indices for occupied voxels:
    # pts.shape == (n, 3)
    pts = np.array(vox.data.nonzero()).T

    # single_voxel_dim is the dimensions in meters of a single voxel
    # single_voxel_dim == array([ 0.00092833,  0.00092833,  0.00092833])
    single_voxel_dim = vox.scale / np.array(vox.dims)

    # convert pts back to meters:
    pts_offset = pts * single_voxel_dim

    # translate the points back to their original position
    pts = pts_offset + vox.translate

    return pts


def read_vertex_points_from_ply_filepath(ply_filepath):
    ply = plyfile.PlyData.read(ply_filepath)

    mesh_vertices = np.ones((ply['vertex']['x'].shape[0], 3))
    mesh_vertices[:, 0] = ply['vertex']['x']
    mesh_vertices[:, 1] = ply['vertex']['y']
    mesh_vertices[:, 2] = ply['vertex']['z']

    return mesh_vertices


def transform_points(vertices, transform):
    vertices_hom = np.ones((vertices.shape[0], 4))
    vertices_hom[:, :-1] = vertices

    # Create new 4xN transformed array
    transformed_vertices_hom = np.dot(transform, vertices_hom.T).T

    transformed_vertices = transformed_vertices_hom[:, :-1]

    return transformed_vertices


def add_obstacles_to_reachability_space_full(points, mins, step_size, dims):
    voxel_grid = np.zeros(shape=dims)

    bbox_min = np.min(points, axis=0)
    bbox_max = np.max(points, axis=0)

    grid_points_min = np.floor((bbox_min - np.array(mins)) / step_size)
    grid_points_max = np.ceil((bbox_max - np.array(mins)) / step_size)

    # grid_points_min[np.where( grid_points_min < 0 )] = 0
    grid_points_min = np.maximum(grid_points_min, 0)
    grid_points_max = np.minimum(grid_points_max, dims - 1)

    if (grid_points_min == grid_points_max).any():
        return voxel_grid

    grid_points_min = grid_points_min.astype(int)
    grid_points_max = grid_points_max.astype(int)

    voxel_grid[grid_points_min[0]:grid_points_max[0] + 1,
    grid_points_min[1]:grid_points_max[1] + 1,
    grid_points_min[2]:grid_points_max[2] + 1] = 1

    return voxel_grid


def add_obstacles_to_reachability_space_full_binvox(points, mins, step_size, dims):
    voxel_grid = np.zeros(shape=dims)

    grid_indicies = np.floor((points - np.array(mins)) / step_size).astype('int')

    for i, dim_min in enumerate(mins):
        mask = np.logical_and(grid_indicies[:, i] < dims[i], grid_indicies[:, i] > 0)
        grid_indicies = grid_indicies[mask]

    for voxel_index in grid_indicies:
        voxel_grid[voxel_index[0], voxel_index[1], voxel_index[2]] = 1

    return voxel_grid


def create_occupancy_grid_from_obstacles(obstacles, mins_xyz, step_size_xyz, dims_xyz, use_binvox=False):
    voxel_grid = np.zeros(shape=dims_xyz)

    for obstacle in obstacles:
        if use_binvox:
            vox = binvox_rw.read_as_3d_array(open(obstacle.mesh_filepath.replace('.ply', '.binvox'), 'r'))
            vertices = binvox_to_points(vox)
        else:
            vertices = read_vertex_points_from_ply_filepath(obstacle.mesh_filepath)
        frame = tf_conversions.fromMsg(obstacle.pose_stamped.pose)
        transform = tf_conversions.toMatrix(frame)
        vertices_transformed = transform_points(vertices, transform)

        if use_binvox:
            voxel_grid += add_obstacles_to_reachability_space_full_binvox(vertices_transformed, mins_xyz, step_size_xyz,
                                                                          dims_xyz)
        else:
            voxel_grid += add_obstacles_to_reachability_space_full(vertices_transformed, mins_xyz, step_size_xyz,
                                                                   dims_xyz)

    voxel_grid[np.where(voxel_grid > 0)] = 1
    return voxel_grid


def get_obstacle_list(args):
    obstacle_info_dict = yaml.load(open(args.obstacle_info_file_path))

    obstacle_list = []
    for obstacle in obstacle_info_dict['obstacle_info']:
        object_name = obstacle['file_name'].split('.')[0]
        mesh_filepath = os.path.join(args.mesh_root, obstacle['file_name'])
        pose = Pose(Point(*obstacle['pose'][:3]), Quaternion(*obstacle['pose'][3:]))

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = obstacle_info_dict['frame_id']
        obstacle = SceneObject(object_name, mesh_filepath, pose_stamped)
        obstacle_list.append(obstacle)
    return obstacle_list


def get_args():
    parser = argparse.ArgumentParser(description='Embed obstacles into a reachability space.')

    parser.add_argument('--config_file',
                        default='embed_obstacles_in_reachability_space.yaml',
                        type=str)

    args = parser.parse_args()
    # args.config_file = os.path.abspath(__file__).replace('.py', '.yaml')

    config = yaml.load(open(args.config_file))
    for k, v in config.items():
        args.__dict__[k] = v

    args.obstacle_info_file_path = os.path.join(args.obstacle_info_dir, args.obstacle_info_filename)
    args.mesh_root = os.path.join(rospkg.RosPack().get_path('reachability_experiments'), 'meshes')

    return args


if __name__ == '__main__':
    args = get_args()

    # load original reachability space
    data = load_reachability_data_from_dir(args.original_reachability_space_dir_path)
    binary_reachability_space, mins, step_size, dims, sdf_reachability_space = data

    # get obstacle list
    obstacle_list = get_obstacle_list(args)

    # embed obstacles in space
    obstacles_mask_3d = create_occupancy_grid_from_obstacles(obstacles=obstacle_list,
                                                             mins_xyz=mins[:3],
                                                             step_size_xyz=step_size[:3],
                                                             dims_xyz=dims[:3],
                                                             use_binvox=args.use_binvox)

    # mask out regions in reachability space occupied by obstacles
    binary_reachability_space[obstacles_mask_3d > 0] = 0

    # Generate sdf
    sdf_reachability_space = generate_SDF_from_binary_space(binary_reachability_space)

    # save to file
    reach_data_dir = os.path.join(args.original_reachability_space_dir_path, 'obstacle_embedded_reachability_space')
    save_reachability_space_to_dir(binary_reachability_space, mins, step_size, dims, sdf_reachability_space,
                                   reach_data_dir)
