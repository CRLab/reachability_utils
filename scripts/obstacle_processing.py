
import plyfile
import ConfigParser
import json
import numpy as np
import os

import rospkg
import tf_conversions
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3


def read_vertex_points_from_ply_filepath(ply_filepath):

	ply = plyfile.PlyData.read(ply_filepath)

	mesh_vertices = np.ones((ply['vertex']['x'].shape[0], 3))
	mesh_vertices[:, 0] = ply['vertex']['x']
	mesh_vertices[:, 1] = ply['vertex']['y']
	mesh_vertices[:, 2] = ply['vertex']['z']

	return mesh_vertices

def xyzrpy_to_transform_array(xyzrpy):

	r = tf_conversions.Rotation.RPY(*xyzrpy[3:])	
	pose = Pose(Point(*xyzrpy[:3]), Quaternion(*r.GetQuaternion()))

	frame = tf_conversions.fromMsg(pose)
	transform = tf_conversions.toMatrix(frame)
	return transform

def transform_points(vertices, transform):

	vertices_hom = np.ones((vertices.shape[0], 4))
	vertices_hom[:,:-1] = vertices

	#Create new 4xN transformed array
	transformed_vertices_hom = np.dot(transform, vertices_hom.T).T

	transformed_vertices = transformed_vertices_hom[:,:-1]

	return transformed_vertices

def load_reachability_params (reachability_config_filename):
	config = ConfigParser.ConfigParser()
	config.read(reachability_config_filename)
	reachability_data_filename = config.get('reachability_config', 'reachability_data_filename')

	step_size = np.loadtxt(reachability_data_filename+'.step', delimiter=',')
	mins = np.loadtxt(reachability_data_filename+'.mins', delimiter=',')
	dims = np.loadtxt(reachability_data_filename+'.dims', delimiter=',', dtype=int)

	return step_size, mins, dims

def load_obstacles(obstacles_dir):
	config_file = os.path.join(obstacles_dir, "config.json")

	with open(config_file) as obstacles_file:
		obstacles_list = json.load(obstacles_file)
	return obstacles_list

def collate_points_from_obstacles(obstacles_dir, obstacles_list):
	obstacles_point_cloud = []
	for obstacle in obstacles_list:

		mesh_file = os.path.join(obstacles_dir, obstacle['filename'])
		pose_xyzrpy = obstacle['pose']

		vertices = read_vertex_points_from_ply_filepath(mesh_file)
		transform = xyzrpy_to_transform_array(pose_xyzrpy)
		vertices_transformed = transform_points(vertices, transform)

		obstacles_point_cloud.extend(vertices_transformed)

	obstacles_point_cloud = np.array(obstacles_point_cloud)

	return obstacles_point_cloud

def add_obstacles_to_reachability_space(points, mins, step_size, dims):

	voxel_grid = np.zeros(shape=dims)

	grid_points = np.floor( (points - np.array(mins)) / step_size )

	# remove obstacles not within reachability space
	mask = grid_points < dims
	grid_points = grid_points[~(~mask).any(axis=1)]

	if grid_points.shape[0] == 0:
		return voxel_grid

	mask = grid_points >= 0
	grid_points = grid_points[~(~mask).any(axis=1)]

	if grid_points.shape[0] == 0:
		return voxel_grid

	grid_points_int = grid_points.astype(int)
	voxel_grid[grid_points_int] = 1

	return voxel_grid

def combine_reachability_space_with_obstacles_space(reachability_space, obstacles_space):
	
	obs_shape = obstacles_space.shape
	reach_shape = reachability_space.shape[:len(obs_shape)]
	assert reach_shape==obs_shape

	reachability_space[obstacles_space>1] = 0

	return reachability_space

def combine_reachability_space_with_obstacles_spaceSDF(reachability_space, obstacles_space):
	pass


if __name__ == '__main__':

	# load reachability space parameters
	reachability_config_filename = rospkg.RosPack().get_path('reachability_utils') + '/config/'
	reachability_config_filename = os.path.join(reachability_config_filename, 'reachability.conf')
	step_size, mins, dims = load_reachability_params (reachability_config_filename)

	# load obstacle data
	obstacles_dir = rospkg.RosPack().get_path('reachability_utils') + '/meshes/obstacles'
	obstacles_list = load_obstacles(obstacles_dir)

	# collect points from obstacles
	obstacles_point_cloud = collate_points_from_obstacles(obstacles_dir, obstacles_list)

	# add obstacles to reachability space
	reachability_space_with_obstacles = add_obstacles_to_reachability_space(obstacles_point_cloud, mins[:3], step_size[:3], dims[:3])

	import IPython
	IPython.embed()

	# TODO:
	# curate obstacle list
	# load obstacles in rviz
	# visualize obstacle reach space and check it lines




