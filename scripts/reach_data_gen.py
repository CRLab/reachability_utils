#!/usr/bin/env python

import numpy as np
import rospkg
import datetime

half_cube_size = 0.5
grid_size = 0.2
xs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
ys = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)
zs = np.arange(-half_cube_size, half_cube_size+0.0005, grid_size)


filename = rospkg.RosPack().get_path('reachability_utils')
filename += '/data/reachability_data_'
# fmt = '%Y-%m-%d-%H-%M-%S'
# filename += datetime.datetime.now().strftime(fmt)
filename += '.csv'
fd = open(filename, 'w')


count = 0

for x in xs:
	for y in ys:
		for z in zs:
			reachable = 0
			if np.allclose([x,y,z],[0.3,0.3,0.3]):
				reachable = 1

			data = ("{:6d} " + "{:.1f} "*3 + "{:1d} \n").format(count, x, y, z, reachable)

			fd.write(data)
			count += 1
fd.close()