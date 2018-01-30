#!/usr/bin/env python

import os
from collections import namedtuple

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander
import grid_sample_client

import world_manager.world_manager_client as wm_client
from reachability_space_marker_plot import display_reachability_space, display_sdf_space, display_grasps_approach
from graspit_utils import get_grasp_from_graspit_sim_ann, get_grasp_from_graspit_ellipse

if __name__ == '__main__':
    rospy.init_node("reachability_visualization")

    meshdir = rospkg.RosPack().get_path('reachability_utils') + "/meshes/"
    mesh_name = "object_0"
    mesh_filepath = "soft_scrub_2lb_4oz/soft_scrub_2lb_4oz.ply"
    mesh_filepath = os.path.join(meshdir, mesh_filepath)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.pose = Pose(Point(1, 1, 1), Quaternion(0, 0, 0, 1))

    wm_client.add_mesh(mesh_name, mesh_filepath, pose_stamped)

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
    # 	mesh_filepath=mesh_filepath,
    # 	max_steps=70000)
    # grasps = grasp_results.grasps
    # energies = grasp_results.energies

    # display_grasps_approach(grasps=grasps, energies=energies, marker_topic="marker_topic", frame_id="object_0")


    # grasp_results = get_grasp_from_graspit_ellipse(
    # 	mesh_filepath=mesh_filepath,
    # 	robot="fetch_gripper",
    # 	pre_grasp_dofs=(4,),
    # 	obstacle="table")
    # grasps = grasp_results.grasps
    # energies = grasp_results.energies
    # display_grasps_approach(grasps=grasps, energies=energies, marker_topic="marker_topic", frame_id="object_0")
