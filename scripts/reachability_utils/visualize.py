#!/usr/bin/env python

import os
from collections import namedtuple

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander
import grid_sample_client

import world_manager.srv
from reachability_space_marker_plot import display_reachability_space, display_sdf_space, display_grasps_approach

GraspEnergies = namedtuple('GraspEnergies', ['grasps', 'energies'], verbose=False)

def get_grasp_from_graspit_sim_ann(
	mesh_path,
	search_energy="REACHABILITY_ENERGY",
	max_steps=70000,
	robot="fetch_gripper",
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()

	gc.importRobot(robot)
	gc.importGraspableBody(mesh_path)

	result = gc.planGrasps(search_energy=search_energy, max_steps=max_steps)

	return result

def get_grasp_from_graspit_ellipse(
	mesh_path,
	search_energy="REACHABILITY_ENERGY",
	robot="fetch_gripper",
	pre_grasp_dofs=(4,),
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()

	gc.importRobot(robot)
	gc.importGraspableBody(mesh_path)

	gl = grid_sample_client.GridSampleClient()
	result = gl.computePreGrasps(10, 2)
	pre_grasps = result.grasps
	grasps = gl.evaluatePreGrasps(pre_grasps, pre_grasp_dofs=pre_grasp_dofs)

	# evaluating grasps
	result = evaluate_grasp_list(
		grasps,
		search_energy=search_energy)

	return result

def evaluate_grasp_complete(
	grasps,
	mesh_path,
	search_energy="REACHABILITY_ENERGY",
	robot="fetch_gripper",
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()
	gc.importRobot(robot)
	gc.importGraspableBody(mesh_path)

	grasp_results = evaluate_grasp_list(grasps,	search_energy)
	return grasp_results

def evaluate_grasp_list(
	grasps,
	search_energy="REACHABILITY_ENERGY"):
	# this assumes the hand and object is already loaded

	gc = graspit_commander.GraspitCommander()

	energies = []
	for g in grasps:
		gc.autoOpen()
		gc.setRobotPose(g.pose)
		gc.forceRobotDof(g.dofs)
		grasp_energy = gc.computeEnergy(search_energy)
		energies.append(grasp_energy.energy)

	energies, grasps = zip(*sorted(zip(energies, grasps)))

	grasp_results = GraspEnergies(grasps=grasps, energies=energies)
	return grasp_results


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

	import IPython
	IPython.embed()

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
	# grasp_results = get_grasp_from_graspit_sim_ann(
	# 	mesh_path=mesh_filepath,
	# 	max_steps=70000)
	# grasps = grasp_results.grasps
	# energies = grasp_results.energies

	# display_grasps_approach(grasps=grasps, energies=energies, marker_topic="marker_topic", frame_id="object_0")


	# grasp_results = get_grasp_from_graspit_ellipse(
	# 	mesh_path=mesh_filepath,
	# 	robot="fetch_gripper",
	# 	pre_grasp_dofs=(4,),
	# 	obstacle="table")