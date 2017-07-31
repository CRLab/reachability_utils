# coding: utf-8

import visualization_msgs.msg
import rospy
import PyKDL
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
import std_msgs.msg
import numpy as np
import math
import rospkg
import os



def make_marker(m_id, pose, frame_id="base_link", 
					color=(1,0,0,1), scale=(.1,.01,.01),
					m_type=visualization_msgs.msg.Marker.ARROW):

	m = visualization_msgs.msg.Marker()
	m.id = m_id
	m.type = m_type

	m.header.frame_id=frame_id
	m.header.stamp = rospy.Time.now()
	m.pose = pose

	m.color = std_msgs.msg.ColorRGBA(*color)
	m.scale = Vector3(*scale)

	return m


def display_reachability_space(publisher, filename, frame_id="object_0"):
	
	data_np = np.loadtxt(filename)
	ma = visualization_msgs.msg.MarkerArray()

	for count, data in enumerate(data_np):

		if True:
		# if count % 5 == 0:
			rospy.sleep(0.001)

			x = data[1]
			y = data[2]
			z = data[3]

			if len(data)>6:
				rot = PyKDL.Rotation.RPY(data[4], data[5], data[6])
			else:
				pitch = math.atan2(-z, math.sqrt(x*x + y*y))
				yaw = math.atan2(y, x)
				rot = PyKDL.Rotation.RPY(0,pitch,yaw)

			quarternion = rot.GetQuaternion()
			p = Pose(Point(x,y,z), Quaternion(*quarternion))

			is_reachable=(data[-1]==1.)
			if is_reachable:
				color=(0,1,0,1)
				print "reachable point"
			else:
				color=(1,0,0,1)
					

			marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color)
			ma.markers.append(marker)
			
	publisher.publish(ma)


def display_grasps_approach(publisher, grasps, frame_id="object_0"):
	
	ma = visualization_msgs.msg.MarkerArray()

	for count, grasp in enumerate(grasps):

		p = grasp.pose
		color=(1,0,0,1)					

		marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color)
		ma.markers.append(marker)
			
	publisher.publish(ma)


def load_sdf_space(processed_file_name):

	step_size = np.loadtxt(processed_file_name+'.step', delimiter=',')
	mins = np.loadtxt(processed_file_name+'.mins', delimiter=',')
	dims = np.loadtxt(processed_file_name+'.dims', delimiter=',', dtype=int)

	data_ND_sdf = np.fromfile(processed_file_name+'.sdf', dtype=float)
	data_ND_sdf = data_ND_sdf.reshape(dims)

	sdf_data = []
	for i in range(data_ND_sdf.size):
		idx = np.unravel_index(i, data_ND_sdf.shape)
		val = data_ND_sdf[idx]

		# position = mins+ idx*step_size
		f = lambda idx_,min_,step_: min_+idx_*step_
		position = map(f, idx, mins, step_size)
		entry = [i]+ position+ [val]	# [i, position, val]
		sdf_data.append(entry)

	return sdf_data

def display_sdf_space():

	folder = rospkg.RosPack().get_path('reachability_utils') + '/data/'
	processed_file_name = os.path.join(folder, 'processed/reach_data')

	sdf_data = load_sdf_space(processed_file_name)
	sdf_data = np.array(sdf_data)
	min_sdf = np.min(sdf_data[:,-1])
	max_sdf = np.max(sdf_data[:,-1])

	publisher = rospy.Publisher("marker_topic", visualization_msgs.msg.MarkerArray, queue_size=100000)
	frame_id="object_0"
	
	ma = visualization_msgs.msg.MarkerArray()
	scale = (.03,.03,.03)
	m_type = visualization_msgs.msg.Marker.CUBE

	for count, data in enumerate(sdf_data):

		p = Pose(Point(data[1],data[2],data[3]), Quaternion(0,0,0,1))
		r = (data[4]-min_sdf) / (max_sdf-min_sdf)
		color=(r,0,1-r,1)

		marker = make_marker(m_id=count, pose=p, frame_id=frame_id, 
			color=color, m_type=m_type, scale=scale)
		ma.markers.append(marker)
		
	publisher.publish(ma)
	import IPython
	IPython.embed()
	assert False


####################
##### Display sdf
rospy.init_node("marker_pub")
display_sdf_space()



####################
##### Display original reachability space
rospy.init_node("marker_pub")
pub = rospy.Publisher("marker_topic", visualization_msgs.msg.MarkerArray, queue_size=100000)
filename = rospkg.RosPack().get_path('reachability_utils') + '/data/reachability_data_.csv'

display_reachability_space(publisher=pub, filename=filename, frame_id="object_0")



# https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
# https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector