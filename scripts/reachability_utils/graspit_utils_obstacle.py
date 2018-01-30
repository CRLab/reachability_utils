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

import geometry_msgs

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
        target_object_pose,
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY",
        max_steps=70000,
        robot="fetch_gripper",
        obstacle_info=[]):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath, target_object_pose)

    if obstacle_info != []:
        load_object_into_graspit(gc, obstacle_info)
    result = gc.planGrasps(search_energy=search_energy, max_steps=max_steps)

    # close hand and evaluate grasps
    pre_grasps = result.grasps
    grasps = evaluateSimAnnGrasps(pre_grasps)
    result.grasps = grasps

    return result


def get_grasp_from_graspit_ellipse(
        mesh_filepath,
        target_object_pose,
        search_energy="HYBRID_REACHABLE_GRASP_ENERGY",
        robot="fetch_gripper",
        pre_grasp_dofs=(4,),
        obstacle_info=[]):
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath, target_object_pose)

    # if there is any obstacle_info
    if obstacle_info != []:
        load_object_into_graspit(gc, obstacle_info)

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


def start_graspit():
    # cmd_str = "roslaunch reachability_energy_plugin reachability_energy_plugin.launch"
    cmd_str = "rosrun reachability_energy_plugin launch_graspit.sh"
    p = subprocess.Popen(cmd_str.split())
    time.sleep(3.0)  # Wait for graspit to start


def kill_graspit():
    cmd_str = "rosnode kill /graspit_interface_node"
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


# input: [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]
def set_object_pose(object_pose):
    object_pose_in_graspit = geometry_msgs.msg.Pose()
    pos = Point(object_pose[0], object_pose[1], object_pose[2])
    ori = Quaternion(object_pose[3], object_pose[4], object_pose[5], object_pose[6])
    object_pose_in_graspit.position = pos
    object_pose_in_graspit.orientation = ori
    return object_pose_in_graspit


# input: object_info = [[['sofa'], [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]]]
def load_object_into_graspit(gc, object_info):
    for each_object in object_info:
        object_filepath = each_object['file_path']
        # obstacle_mesh_filepath = os.path.join(args.mesh_root, object_name)
        object_pose = each_object['pose']
        object_pose_in_graspit = set_object_pose(object_pose)
        gc.importObstacle(object_filepath, object_pose_in_graspit)
