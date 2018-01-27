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
from graspit_utils import get_grasp_from_graspit_sim_ann, get_grasp_from_graspit_ellipse
import argparse

if __name__ == '__main__':
    rospy.init_node("reachability_visualization")

    parser = argparse.ArgumentParser(description='Visualize the reachability space.')
    parser.add_argument('--add_obstacle', dest='accumulate', action='store_true', default=max,
                        help='sum the integers (default: find the max)')

    args = parser.parse_args()
    wm_add_service_proxy = rospy.ServiceProxy("/world_manager/add_object",
                                              world_manager.srv.AddObject)
    wm_add_service_proxy.wait_for_service()

    meshdir = rospkg.RosPack().get_path('reachability_utils') + "/meshes/"
    mesh_name = "object_0"
    mesh_filepath = "soft_scrub_2lb_4oz/soft_scrub_2lb_4oz.ply"
    mesh_filepath = os.path.join(meshdir, mesh_filepath)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "staubli_rx60l_link1"
    pose_stamped.pose = Pose(Point(-0.7, -0.2, -0.3), Quaternion(0, 0, 0, 1))

    wm_add_service_proxy(mesh_name, mesh_filepath, pose_stamped, True)

    # filename = "/home/iakinola/ros/reachability_space_ws/src/reachability_space_generation/data/reachability_data_.csv"
    filename = "/home/iakinola/temp.csv"

    display_reachability_space(filename=filename, marker_topic="marker_topic2", frame_id="staubli_rx60l_link1")

# import IPython
# IPython.embed()

# ####################
# ##### Display original reachability space
# filename = rospkg.RosPack().get_path('reachability_utils') + '/data/reachability_data_.csv'

# display_reachability_space(filename=filename, marker_topic="marker_topic", frame_id="staubli_rx60l_link1")


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
