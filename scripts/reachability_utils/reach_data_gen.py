#!/usr/bin/env python

import numpy as np
import rospkg
import datetime
import os
import skfmm
import math
import ConfigParser

def generate_reachability_space(reach_data_location):

	half_cube_size = 0.5
	grid_size = 0.2
	xs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	ys = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	zs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)

	reach_data_location += '/reachability_data_'
	# fmt = '%Y-%m-%d-%H-%M-%S'
	# reach_data_location += datetime.datetime.now().strftime(fmt)
	reach_data_location += '.csv'
	fd = open(reach_data_location, 'w')


	count = 0

	for x in xs:
		for y in ys:
			for z in zs:
				reachable = 0
				reachability_list = [[0.3,0.3,0.3],[-0.3,-0.3,0.3]]
				if np.any([np.allclose([x,y,z],p) for p in reachability_list]):
					reachable = 1

				data = ("{:6d} " + "{:.1f} "*3 + "{:1d} \n").format(count, x, y, z, reachable)

				fd.write(data)
				count += 1
	fd.close()


def generate_reachability_space_6D(reach_data_location):

	half_cube_size = 0.5
	grid_size = 0.2
	xs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	ys = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	zs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)

	half_cube_size = math.pi
	grid_size = math.pi/4
	rs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	ps = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
	yaws = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)

	reach_data_location += '/reachability_data_'
	# fmt = '%Y-%m-%d-%H-%M-%S'
	# reach_data_location += datetime.datetime.now().strftime(fmt)
	reach_data_location += '.csv'
	fd = open(reach_data_location, 'w')

	count = 0

	for x in xs:
		for y in ys:
			for z in zs:
				for r in rs:
					for p in ps:
						for yaw in yaws:
							reachable = 0
							reachability_list = [[0.3,0.3,0.3,rs[rs.size//2],ps[ps.size//2],yaws[yaws.size//2]]]
							if np.any([np.allclose([x,y,z,r,p,yaw],reach) for reach in reachability_list]):
								reachable = 1
								print 'reachable point'

							# reachability_list = [[rs[rs.size//2],ps[ps.size//2],yaws[yaws.size//2]]]
							# if np.any([np.allclose([r,p,yaw],reach) for reach in reachability_list]):
							# 	reachable = 1
							# 	print 'reachable point'

							data = ("{:6d} " + "{:.6f} "*6 + "{:1d} \n").format(count, x, y, z, r, p, yaw, reachable)

							fd.write(data)
							count += 1
	fd.close()


def process_reachability_data(reach_data_raw, processed_file_name):

	data = np.loadtxt(reach_data_raw)
	print "data loaded."

	print "creating ND data full ..."

	data = data[:, 1:]
	unique_elements = map(np.unique, data.transpose())
	unique_elements = unique_elements[:-1]

	grid_size = [e.shape[0] for e in unique_elements]
	data_ND = np.zeros(shape=grid_size)
	# data_ND = np.zeros(shape=grid_size, dtype=np.int8)

	for i, row in enumerate(data):
		idx = [np.argwhere(u_elems==val)[0][0] for val,u_elems in zip(row,unique_elements)]
		data_ND[tuple(idx)]=int(data[i,-1])

	print "now generating sdf ..."
	# Generate sdf
	data_ND -= 0.5
	if len(data_ND.shape) == 3:
		data_ND_sdf = skfmm.distance(data_ND)
	if len(data_ND.shape) == 6:
		data_ND_sdf = skfmm.distance(data_ND, periodic=[False,False,False,True,True,True])
	data_ND += 0.5	# undo previous operation


	# save dimension to file
	def save_array(filename, array):
		with open (filename, 'w') as f:
			f.write(', '.join(str(x) for x in array))\


	step_size = map(lambda x: x[1]-x[0], unique_elements)
	step_size = np.array(step_size)

	mins = map(lambda x: x[0], unique_elements)
	mins = np.array(mins)

	dims = np.array(data_ND.shape)

	print "now saving data to file..."

	# processed_file_name = './processed/reach_data'
	data_ND.tofile(open(processed_file_name+'.full', 'w'))
	data_ND_sdf.tofile(open(processed_file_name+'.sdf', 'w'))

	save_array(processed_file_name+'.step', step_size)
	save_array(processed_file_name+'.mins', mins)
	save_array(processed_file_name+'.dims', dims)

	data_ND_sdf2 = np.fromfile(processed_file_name+'.sdf', dtype=float)
	data_ND_sdf2 = data_ND_sdf2.reshape(dims)
	np.max(data_ND_sdf2-data_ND_sdf)

	# check that data loaded in c++ matches
	if len(data_ND_sdf.shape) == 3:
		print "data_ND_full_sdf[2,1,0]:\t", data_ND_sdf[2,1,0]
	if len(data_ND_sdf.shape) == 6:
		print "data_ND_full_sdf[2,1,0,0,3,2]:\t", data_ND_sdf[2,1,0,0,3,2]

	# import IPython
	# IPython.embed()

	# compute reachability statistics for simulated annealing f(x) = m(x - c)
	alpha = 95
	c = np.percentile(data_ND_sdf, 100-alpha)
	gamma = 0.99
	m = -np.log(1/gamma - 1) / (np.max(data_ND_sdf) - c)

	print "reachability constants: m: {} \t c:{}".format(m, c)

	sdf_stats = {}
	sdf_stats['min'] = np.min(data_ND_sdf)
	sdf_stats['max'] = np.max(data_ND_sdf)
	sdf_stats['alpha_percentile'] = np.percentile(data_ND_sdf, alpha)
	sdf_stats['inv_alpha_percentile'] = np.percentile(data_ND_sdf, 100-alpha)
	print "reachability constants: \n min value: {} \n max value: {}".format(sdf_stats['min'], sdf_stats['max'])



def generate_graspit_config_file(config_args, reachability_config_filename):

    config = ConfigParser.ConfigParser()
    section_name = 'reachability_config'
    config.add_section(section_name)

    for key in config_args.keys():
		config.set(section_name, key, config_args[key])

    cfgfile = open(reachability_config_filename,'w')
    config.write(cfgfile)
    cfgfile.close()


if __name__ == '__main__':

	is_3D_space = True

	# generate custom reachability space
	reach_data_location = rospkg.RosPack().get_path('reachability_utils') + '/data/'
	if not os.path.exists(reach_data_location):
		os.makedirs(reach_data_location)
	if is_3D_space:
		generate_reachability_space(reach_data_location)
	else:
		generate_reachability_space_6D(reach_data_location)

	# process the generated reachability space
	processed_reach_data_location = reach_data_location+'processed/'
	if not os.path.exists(processed_reach_data_location):
		os.makedirs(processed_reach_data_location)
	reach_data_raw = 'reachability_data_.csv'
	reach_data_raw = os.path.join(reach_data_location, reach_data_raw)
	processed_file_name = os.path.join(reach_data_location, 'processed/reach_data')
	process_reachability_data(reach_data_raw, processed_file_name)

	# set object to reference frame transform
	object_pose_in_reference_frame = np.eye(4)
	obj_pose_filename = 'object_pose_in_reference_frame.csv'
	obj_pose_filename = os.path.join(reach_data_location, obj_pose_filename)
	np.savetxt(obj_pose_filename, object_pose_in_reference_frame, fmt='%1.6f', delimiter=" ")

	# create config file for graspit energy plugin
	config_args = {}
	config_args['reachability_data_filename'] = processed_file_name
	config_args['object_pose_in_reference_frame'] = obj_pose_filename
	config_args['log_energy_flag'] = 'false' #False
	config_args['annealing_data_log_file'] = processed_file_name
	config_args['contact_energy_coeff'] = 0.1
	config_args['potential_energy_coeff'] = 0.1
	config_args['reachability_energy_coeff'] = 0.1

	reachability_config_filename = rospkg.RosPack().get_path('reachability_utils') + '/config/'
	if not os.path.exists(reachability_config_filename):
		os.makedirs(reachability_config_filename)
	reachability_config_filename += 'reachability.conf'
	generate_graspit_config_file(config_args, reachability_config_filename)