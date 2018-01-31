#!/usr/bin/env python

import visualization_msgs.msg
import rospy
import tf_conversions
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
import std_msgs.msg
import numpy as np
import math


def make_marker(m_id, pose, frame_id="base_link", color=(1, 0, 0, 1), scale=(.1, .03, .01), m_type=visualization_msgs.msg.Marker.ARROW, arrow_direction='x'):

    final_marker_pose = tf_conversions.fromMsg(pose)
    # For a given pose, the marker can point along either the x, y or z axis. By default, ros rvis points along x axis
    if arrow_direction == 'x':
        color = (1, 0, 0, 1)
        pass
    elif arrow_direction == 'y':
        color = (0, 1, 0, 1)
        final_marker_pose.M.DoRotZ(math.pi / 2)
    elif arrow_direction == 'z':
        color = (0, 0, 1, 1)
        final_marker_pose.M.DoRotY(-math.pi / 2)
    else:
        print "Invalid arrow direction, using default x "

    m = visualization_msgs.msg.Marker()
    m.id = m_id
    m.type = m_type

    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.pose = tf_conversions.toMsg(final_marker_pose)

    m.color = std_msgs.msg.ColorRGBA(*color)
    m.scale = Vector3(*scale)

    return m


def parse_reachability_space_to_point_list(obstacle_data, mins, step_size, dims):
    obstacle_data_points = []
    for i in range(obstacle_data.size):
        idx = np.unravel_index(i, obstacle_data.shape)
        val = obstacle_data[idx]

        # position = mins+ idx*step_size
        f = lambda idx_, min_, step_: min_ + idx_ * step_
        position = map(f, idx, mins, step_size)
        entry = [i] + position + [val]  # [i, position, val]
        obstacle_data_points.append(entry)

    return obstacle_data_points


def display_obstacle_space_data(obstacle_data, mins, step_size, dims, marker_topic="marker_topic", frame_id="object_0"):
    obstacle_data_points = parse_reachability_space_to_point_list(obstacle_data, mins, step_size, dims)

    publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.MarkerArray, queue_size=100000)

    ma = visualization_msgs.msg.MarkerArray()

    for count, data in enumerate(obstacle_data_points):

        if data[-1] == 1.:
            x = data[1]
            y = data[2]
            z = data[3]

            p = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
            color = (1, 0, 1, 1)
            m_type = visualization_msgs.msg.Marker.SPHERE
            scale = (.03, .03, .03)

            marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color, m_type=m_type, scale=scale)
            ma.markers.append(marker)

    rospy.sleep(1)
    publisher.publish(ma)


def get_marker_array(data_np, frame_id="object_0", arrow_direction='x'):
    ma = visualization_msgs.msg.MarkerArray()

    for count, data in enumerate(data_np):

        if True:
            # if count % 25 == 0:
            idx, x, y, z, roll, pitch, yaw, is_reachable = data

            rotation = tf_conversions.Rotation.RPY(roll, pitch, yaw)
            quarternion = rotation.GetQuaternion()
            p = Pose(Point(x, y, z), Quaternion(*quarternion))

            if is_reachable:
                color = (0, 1, 0, 1)
                print "reachable point"
            else:
                # continue
                color = (1, 0, 0, 1)

            marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color, arrow_direction=arrow_direction)
            ma.markers.append(marker)

    return ma


def generate_arrow_based_on_location(xyz):
    x, y, z = xyz

    row = 0
    pitch = math.atan2(-z, math.sqrt(x * x + y * y))
    yaw = math.atan2(y, x)
    return [row, pitch, yaw]


def display_reachability_space(filename, marker_topic="marker_topic", frame_id="object_0", arrow_direction='x'):
    data_np = np.loadtxt(filename)

    if data_np.shape[1] < 6:
        # if the reachability data does not have rpy information, generate pose based on location
        data_full = np.zeros((data_np.shape[0], 8))
        data_full[:, :4] = data_np[:, :4]
        data_full[:, -1] = data_np[:, -1]

        rpys = map(generate_arrow_based_on_location, data_np[:, 1:4])
        data_full[:, 4:7] = np.array(rpys)
        data_np = data_full

    publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.MarkerArray, queue_size=100000)
    marker_array = get_marker_array(data_np, frame_id, arrow_direction=arrow_direction)
    rospy.sleep(1)
    publisher.publish(marker_array)


def load_sdf_space(processed_file_name):
    step_size = np.loadtxt(processed_file_name + '.step', delimiter=',')
    mins = np.loadtxt(processed_file_name + '.mins', delimiter=',')
    dims = np.loadtxt(processed_file_name + '.dims', delimiter=',', dtype=int)

    data_ND_sdf = np.fromfile(processed_file_name + '.sdf', dtype=float)
    data_ND_sdf = data_ND_sdf.reshape(dims)

    sdf_data = []
    for i in range(data_ND_sdf.size):
        idx = np.unravel_index(i, data_ND_sdf.shape)
        val = data_ND_sdf[idx]

        # position = mins+ idx*step_size
        f = lambda idx_, min_, step_: min_ + idx_ * step_
        position = map(f, idx, mins, step_size)
        entry = [i] + position + [val]  # [i, position, val]
        sdf_data.append(entry)

    return sdf_data


def display_sdf_space_old(processed_file_name, marker_topic="marker_topic"):
    sdf_data = load_sdf_space(processed_file_name)
    sdf_data = np.array(sdf_data)
    min_sdf = np.min(sdf_data[:, -1])
    max_sdf = np.max(sdf_data[:, -1])

    publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.MarkerArray, queue_size=100000)
    frame_id = "object_0"

    ma = visualization_msgs.msg.MarkerArray()
    scale = (.03, .03, .03)
    m_type = visualization_msgs.msg.Marker.CUBE

    for count, data in enumerate(sdf_data):
        p = Pose(Point(data[1], data[2], data[3]), Quaternion(0, 0, 0, 1))
        r = (data[-1] - min_sdf) / (max_sdf - min_sdf)
        color = (r, 0, 1 - r, 1)

        marker = make_marker(m_id=count, pose=p, frame_id=frame_id,
                             color=color, m_type=m_type, scale=scale)
        ma.markers.append(marker)

    rospy.sleep(1)
    publisher.publish(ma)


def display_sdf_space_data(sdf_data, marker_topic="marker_topic"):
    sdf_data = np.array(sdf_data)
    min_sdf = np.min(sdf_data[:, -1])
    max_sdf = np.max(sdf_data[:, -1])

    publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.MarkerArray, queue_size=100000)
    frame_id = "object_0"

    ma = visualization_msgs.msg.MarkerArray()
    scale = (.03, .03, .03)
    m_type = visualization_msgs.msg.Marker.CUBE

    is_6d_space = len(sdf_data[0]) > 6
    if is_6d_space:
        scale = (.1, .01, .01)
        m_type = visualization_msgs.msg.Marker.ARROW

    for count, data in enumerate(sdf_data):
        if is_6d_space and not count % 25 == 0:
            continue

        if is_6d_space:
            rot = tf_conversions.Rotation.RPY(data[4], data[5], data[6])
            quarternion = rot.GetQuaternion()
        else:
            quarternion = (0, 0, 0, 1)

        p = Pose(Point(data[1], data[2], data[3]), Quaternion(*quarternion))
        r = (data[-1] - min_sdf) / (max_sdf - min_sdf)
        color = (r, 0, 1 - r, 1)

        marker = make_marker(m_id=count, pose=p, frame_id=frame_id,
                             color=color, m_type=m_type, scale=scale)
        ma.markers.append(marker)

    rospy.sleep(1)
    publisher.publish(ma)


def display_sdf_space(processed_file_name, marker_topic="marker_topic"):
    sdf_data = load_sdf_space(processed_file_name)
    display_sdf_space_data(sdf_data, marker_topic)


def display_grasps_approach(grasps, energies=None, marker_topic="marker_topic", frame_id="object_0"):
    publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.MarkerArray, queue_size=100000)

    ma = visualization_msgs.msg.MarkerArray()

    for count, grasp in enumerate(grasps):

        p = grasp.pose
        if energies is None:
            color = (1, 1, 0, 1)
        else:
            b = (energies[count] - np.min(energies)) / (np.max(energies) - np.min(energies))
            color = (1 - b, 0, b, 1)

        # # fix to align Barrett hand approach_tran to line up with x axis
        # if grasp.approach_direction.vector.z == 1.0:
        #     q_array_func = lambda p: np.array([p.x, p.y, p.z, p.w])
        #     q_orig = q_array_func(grasp.pose.orientation)
        #     rot = tf_conversions.Rotation.Quaternion(*q_orig)
        #     rot.DoRotY(-math.pi / 2)
        #     rot.DoRotX(math.pi)
        #     grasp.pose.orientation = Quaternion(*rot.GetQuaternion())
        #
        # marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color)
        if grasp.approach_direction.vector.z == 1.0:
            # marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color, arrow_direction='z')
            ma.markers.append(make_marker(m_id=count, pose=p, frame_id=frame_id, color=color, arrow_direction='x'))
            ma.markers.append(make_marker(m_id=count+1000, pose=p, frame_id=frame_id, color=color, arrow_direction='y'))
            ma.markers.append(make_marker(m_id=count+2000, pose=p, frame_id=frame_id, color=color, arrow_direction='z'))
        else:
            marker = make_marker(m_id=count, pose=p, frame_id=frame_id, color=color)
        # ma.markers.append(marker)

    rospy.sleep(1)
    publisher.publish(ma)


# # https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
# # https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector


if __name__ == '__main__':
    pass
