#!/usr/bin/env python

import rospy
import world_manager.srv
import rospkg
import os
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import graspit_commander

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


def get_grasp_from_graspit(
    model_name,
    mesh_path=mesh_filepath,
    robot="fetch_gripper",
    obstacle="table"):

    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_path)

    result = gc.planGrasps()

    # gl = GridSampleClient()
    # result = gl.computePreGrasps(20, 1)
    # pre_grasps = result.grasps
    # result = gl.evaluatePreGrasps(pre_grasps, pre_grasp_dofs=(4,))

    return result
    
    
