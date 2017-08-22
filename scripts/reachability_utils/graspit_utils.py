#!/usr/bin/env python

import os
import subprocess
from collections import namedtuple

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander
import grid_sample_client

GraspEnergies = namedtuple('GraspEnergies', ['grasps', 'energies'], verbose=False)

def get_grasp_from_graspit_sim_ann(
	mesh_filepath,
	search_energy="REACHABILITY_ENERGY",
	max_steps=70000,
	robot="fetch_gripper",
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()

	gc.importRobot(robot)
	gc.importGraspableBody(mesh_filepath)

	result = gc.planGrasps(search_energy=search_energy, max_steps=max_steps)

	return result

def get_grasp_from_graspit_ellipse(
	mesh_filepath,
	search_energy="REACHABILITY_ENERGY",
	robot="fetch_gripper",
	pre_grasp_dofs=(4,),
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()

	gc.importRobot(robot)
	gc.importGraspableBody(mesh_filepath)

	gl = grid_sample_client.GridSampleClient()
	result = gl.computePreGrasps(10, 0)
	pre_grasps = result.grasps
	grasps = gl.evaluatePreGrasps(pre_grasps, pre_grasp_dofs=pre_grasp_dofs)

	# evaluating grasps
	result = evaluate_grasp_list(
		grasps,
		search_energy=search_energy)

	return result

def evaluate_grasp_complete(
	grasps,
	mesh_filepath,
	search_energy="REACHABILITY_ENERGY",
	robot="fetch_gripper",
	obstacle="table"):

	gc = graspit_commander.GraspitCommander()
	gc.clearWorld()
	gc.importRobot(robot)
	gc.importGraspableBody(mesh_filepath)

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



def start_graspit():
    # cmd_str = "roslaunch reachability_energy_plugin reachability_energy_plugin.launch"
    cmd_str = "rosrun reachability_energy_plugin launch_graspit.sh"
    p = subprocess.Popen(cmd_str.split())


def kill_graspit():
    cmd_str = "rosnode kill /graspit_interface_node"
    p = subprocess.Popen(cmd_str.split())

