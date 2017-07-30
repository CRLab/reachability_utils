# coding: utf-8

import visualization_msgs.msg
import rospy
import PyKDL
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
import std_msgs.msg
import numpy as np
import math
import rospkg



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


rospy.init_node("marker_pub")
pub = rospy.Publisher("marker_topic", visualization_msgs.msg.MarkerArray, queue_size=100000)
filename = rospkg.RosPack().get_path('reachability_utils') + '/data/reachability_data_.csv'

display_reachability_space(publisher=pub, filename=filename, frame_id="object_0")

import IPython
IPython.embed()



# https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
# https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector