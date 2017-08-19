#!/usr/bin/env python

import rospy
import world_manager.srv
import rospkg
import os
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander

from reachability_space_marker_plot import display_reachability_space, display_sdf_space, display_grasps_approach


def get_grasp_from_graspit(
	model_name,
	mesh_path,
	robot="fetch_gripper",
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()

	gc.importRobot(robot)
	gc.importGraspableBody(mesh_path)

	result = gc.planGrasps(search_energy="REACHABILITY_ENERGY", max_steps=70000)
	# result = gc.planGrasps(max_steps=70000)

	# gl = GridSampleClient()
	# result = gl.computePreGrasps(20, 1)
	# pre_grasps = result.grasps
	# result = gl.evaluatePreGrasps(pre_grasps, pre_grasp_dofs=(4,))

	return result
	

if __name__ == '__main__':

	rospy.init_node("reachability_visualization")

	wm_add_service_proxy = rospy.ServiceProxy("/world_manager/add_object",
											   world_manager.srv.AddObject)
	wm_add_service_proxy.wait_for_service()


	meshdir = rospkg.RosPack().get_path('reachability_utils') + "/meshes/"
	mesh_name = "object_0"
	mesh_filepath = "soft_scrub_2lb_4oz/soft_scrub_2lb_4oz.ply"
	mesh_filepath = os.path.join(meshdir, mesh_filepath)

	pose_stamped = PoseStamped()
	pose_stamped.header.frame_id = "base_link"
	pose_stamped.pose = Pose(Point(1,1,1), Quaternion(0,0,0,1))

	wm_add_service_proxy(mesh_name, mesh_filepath, pose_stamped)

	# import IPython
	# IPython.embed()

	# ####################
	# ##### Display original reachability space
	# filename = rospkg.RosPack().get_path('reachability_utils') + '/data/reachability_data_.csv'

	# display_reachability_space(filename=filename, marker_topic="marker_topic", frame_id="object_0")


	# ####################
	# ##### Display sdf	
	# folder = rospkg.RosPack().get_path('reachability_utils') + '/data/'
	# processed_file_name = os.path.join(folder, 'processed/reach_data')

	# display_sdf_space(processed_file_name)


	# ####################
	# ##### Display grasps
	# grasp_results = get_grasp_from_graspit(
	#     model_name="object_0",
	#     mesh_path=mesh_filepath)
	# grasps = grasp_results.grasps
	# energies = grasp_results.energies

	# display_grasps_approach(grasps=grasps, energies=energies, marker_topic="marker_topic", frame_id="object_0")



