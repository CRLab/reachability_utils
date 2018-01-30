#!/usr/bin/env python

import os
import time
import copy
import subprocess
from collections import namedtuple

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander
import grid_sample_client

GraspEnergies = namedtuple('GraspEnergies', ['grasps', 'energies'], verbose=False)


def evaluateSimAnnGrasps(pre_grasps):
    import graspit_commander
    gc = graspit_commander.GraspitCommander()

    grasps = []
    for i, pre_grasp in enumerate(pre_grasps):
        gc.toggleAllCollisions(False)
        gc.setRobotPose(pre_grasp.pose)
        gc.forceRobotDof(pre_grasp.dofs)
        gc.toggleAllCollisions(True)

        gc.autoGrasp()
        robot_state = gc.getRobot(0)

        try:
            quality = gc.computeQuality()
            volume_quality = quality.volume
            epsilon_quality = quality.epsilon
        except:
            volume_quality = -1
            epsilon_quality = -1

        grasp = copy.deepcopy(pre_grasp)
        grasp.pose = robot_state.robot.pose
        grasp.volume_quality = volume_quality
        grasp.epsilon_quality = epsilon_quality
        grasp.dofs = robot_state.robot.dofs
        grasps.append(grasp)
    return grasps


def get_grasp_from_graspit_sim_ann(
        mesh_filepath,
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY",
        max_steps=70000,
        robot="fetch_gripper",
        obstacle="table"):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath)

    result = gc.planGrasps(search_energy=search_energy, max_steps=max_steps)

    # close hand and evaluate grasps
    pre_grasps = result.grasps
    grasps = evaluateSimAnnGrasps(pre_grasps)
    result.grasps = grasps

    return result


def get_grasp_from_graspit_ellipse(
        mesh_filepath,
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY",
        robot="fetch_gripper",
        pre_grasp_dofs=(4,),
        obstacle="table"):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath)

    gl = grid_sample_client.GridSampleClient()
    result = gl.computePreGrasps(20, 0)
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
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY",
        robot="fetch_gripper",
        obstacle="table"):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()
    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath)

    grasp_results = evaluate_grasp_list(grasps, search_energy)
    return grasp_results


def evaluate_grasp_list(
        grasps,
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY"):
    # this assumes the hand and object is already loaded

    gc = graspit_commander.GraspitCommander()

    energies = []
    for g in grasps:
        gc.autoOpen()
        gc.setRobotPose(g.pose)
        gc.forceRobotDof(g.dofs)
        grasp_energy = gc.computeEnergy(search_energy)
        energies.append(grasp_energy.energy)

    # energies, grasps = zip(*sorted(zip(energies, grasps)))

    grasp_results = GraspEnergies(grasps=grasps, energies=energies)
    return grasp_results


def start_graspit(cmd_str):
    # cmd_str = "roslaunch reachability_energy_plugin reachability_energy_plugin.launch"
    p = subprocess.Popen(cmd_str.split())
    time.sleep(3.0)  # Wait for graspit to start


def kill_graspit(cmd_str="rosnode kill /graspit_interface_node"):
    p = subprocess.Popen(cmd_str.split())
    time.sleep(3.0)  # Wait for graspit to start


def visualize_grasps_in_graspit_with_mesh(
        grasp,
        mesh_filepath,
        robot="fetch_gripper"):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()
    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath)
    gc.setRobotPose(grasp.pose)

    rospy.loginfo("Showing grasp in Graspit!")


def visualize_grasps_in_graspit(
        grasp):
    gc = graspit_commander.GraspitCommander()
    gc.setRobotPose(grasp.pose)

    rospy.loginfo("Showing grasp in Graspit!")
