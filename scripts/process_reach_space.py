#!/usr/bin/env python

import numpy as np
import math
import rospkg
import os
import skfmm


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
	data_ND_sdf = skfmm.distance(data_ND)


	# save dimension to file
	def save_array(filename, array):
		with open (filename, 'w') as f:
			f.write(', '.join(str(x) for x in array))\


	step_size = map(lambda x: x[1]-x[0], unique_elements)
	step_size = np.array(step_size)

	mins = map(lambda x: x[0], unique_elements)
	mins = np.array(mins)

	dims = np.array(data_ND.shape)


	# import IPython
	# IPython.embed()
	# assert False

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

	# import IPython
	# IPython.embed()

	# check that data loaded in c++ matches
	# print "data_ND_full_sdf[2,1,0,0,10,1]:\t", data_ND__sdf[2,1,0,0,10,1]
	print "data_ND_full_sdf[2,1,0]:\t", data_ND_sdf[2,1,0]



folder = rospkg.RosPack().get_path('reachability_utils') + '/data/'
reach_data_raw = 'reachability_data_.csv'
reach_data_raw = os.path.join(folder, reach_data_raw)
processed_file_name = os.path.join(folder, 'processed/reach_data')


process_reachability_data(reach_data_raw, processed_file_name)

