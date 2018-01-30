#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from reachability_space_marker_plot import make_marker
import argparse
import visualization_msgs.msg
import PyKDL

RED = (1, 0, 0, .1)
GREEN = (0, 1, 0, 1)
BLUE = (0, 0, 1, 1)
PURPLE = (1, 0, 1, 1)

if __name__ == '__main__':

    rospy.init_node("reachability_visualization")

    parser = argparse.ArgumentParser(description='Visualize the reachability space.')
    parser.add_argument('--reachability_filepath',
                        type=str,
                        #default="/home/iakinola/temp.csv",
                        #default="/home/iakinola/ros/reachability_space_ws/src/reachability_space_generation/data/staubli_reachability.csv",
                        default="/home/iakinola/ros/reachability_space_ws/src/reachability_space_generation/data/staubli_small_reachability.csv",
                        help='CSV file that we are going to visualize')
    parser.add_argument('--marker_topic',
                        default="/reachability_space",
                        type=str,
                        help='MarkerArray topic to display reachability space in RVIZ')

    parser.add_argument('--reference_frame',
                        default="staubli_rx60l_link1",
                        help='The frame of reference that the reachability space is in relation too.')

    parser.add_argument('--downsample',
                        action='store_true',
                        help='Downsample the displayed reachability space')

    parser.add_argument('--only_display_reachable',
                        action='store_true',
                        help='Downsample the displayed reachability space')

    parser.add_argument('--only_display_unreachable',
                        action='store_true',
                        help='Downsample the displayed reachability space')

    parser.add_argument('--z_slice',
                        action='store_true',
                        help='Single slice with z = args.slice_value')

    parser.add_argument('--y_slice',
                        action='store_true',
                        help='Single slice with y = args.slice_value')

    parser.add_argument('--roll_slice',
                        action='store_true',
                        help='Single slice with roll = args.slice_value')

    parser.add_argument('--roll_slice_value',
                        default=0.0,
                        type=float,
                        help='When showing roll_slice, this is the value of roll to plot')

    parser.add_argument('--slice_value',
                        default=-0.1,
                        type=float,
                        help='When showing z_slice, or y_slize, this is the value of z, or y to plot')

    parser.add_argument('--downsample_factor',
                        default=250,
                        type=int,
                        help='factor to downsample the reachability space by. Ex 250 means 1/250 are shown.')

    args = parser.parse_args()

    publisher = rospy.Publisher(args.marker_topic,
                                visualization_msgs.msg.MarkerArray,
                                queue_size=1)

    ma = visualization_msgs.msg.MarkerArray()

    data_np = np.loadtxt(args.reachability_filepath)
    markers = [None]*data_np.shape[0]
    count = 0
    for iter_num, data in enumerate(data_np):

        if args.downsample:
            if iter_num % args.downsample_factor != 0:
                continue

        idx, x, y, z, roll, pitch, yaw, is_reachable = data
        is_reachable = (is_reachable == 1.)

        if args.z_slice:
            if not np.isclose(z, args.slice_value):
                continue
        if args.y_slice:
            if not np.isclose(y, args.slice_value):
                continue
        if args.roll_slice:
            if not np.isclose(roll, args.roll_slice_value):
                continue
        #
        # if z < -0.1:
        #     print "setting z below zero to unreachable"
        #     is_reachable = False
        # if np.isclose(roll, -3.14159265):
        #     y += 0.5
        # if np.isclose(roll, -1.57079633):
        #     y += 1
        # if np.isclose(roll, 0.):
        #     pass

        rotation = PyKDL.Rotation.RPY(roll, pitch, yaw).GetQuaternion()
        p = Pose(Point(x, y, z), Quaternion(*rotation))

        if is_reachable:
            color = GREEN
            if args.only_display_unreachable:
                continue
        else:
            if args.only_display_reachable:
                continue
            color = RED

        marker = make_marker(m_id=count,
                             pose=p, frame_id=args.reference_frame, color=color, arrow_direction='z')

        markers[count] = marker
        count += 1
    markers = markers[:count]
    ma.markers = markers


    print "gathered :" + str(len(ma.markers)) + " markers"
    publisher.publish(ma)
    rospy.sleep(1.0)

