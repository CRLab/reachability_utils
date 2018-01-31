#!/usr/bin/python
import numpy as np
import tf_conversions

from fractions import Fraction
from collections import defaultdict

# http://www.skylinesoft.com/SkylineGlobe/TerraExplorer/v6.5.0/APIReferenceGuide/Yaw_Pitch_and_Roll_Angles.htm

step_size = 2   # should be odd to avoid the singularity points which occur when pitch is -pi/2

# rolls = np.arange(-np.pi, np.pi, np.pi / step_size)
rolls = np.arange(-np.pi, 0, np.pi / step_size)
pitchs = np.arange(-np.pi, np.pi, np.pi / step_size)
yaws = np.arange(0, 2 * np.pi, np.pi / step_size)

as_fraction_of_pi = lambda num: str(Fraction(num / np.pi).limit_denominator()) + '*pi'
list_as_fraction_of_pi = lambda vals: map(as_fraction_of_pi, vals)
print "rolls: \t{}".format(list_as_fraction_of_pi(rolls))
print "pitchs: \t{}".format(list_as_fraction_of_pi(pitchs))
print "yaws: \t{}".format(list_as_fraction_of_pi(yaws))

rpys = []

for roll in rolls:
    for pitch in pitchs:
        for yaw in yaws:
            rpys.append([roll, pitch, yaw])

print "Total number of rpys: \t{} \n\n\n".format(len(rpys))


def get_angular_difference(rpy1, rpy2):
    arrow1 = tf_conversions.Rotation.RPY(*rpy1)
    arrow2 = tf_conversions.Rotation.RPY(*rpy2)
    c = arrow1 * arrow2.Inverse()

    # c.GetQuaternion()  # get quaternion result
    # c.GetRPY()  # get roll pitch yaw result

    return c.GetRotAngle()[0]


# Fixing the singularities of rpy
# http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4104344
def fix_rpy(rpy):
    rpy_new = [val for val in rpy]

    if rpy_new[0] >= 0 or np.isclose(rpy_new[0], 0):
        # rpy_new = np.array(rpy_new) + np.array([-np.pi, np.pi, np.pi])
        rpy_new = [rpy_new[0] - np.pi, np.pi - rpy_new[1], rpy_new[2] - np.pi]
        if rpy_new[1] < -np.pi:
            rpy_new[1] += 2 * np.pi
        if rpy_new[1] >= np.pi or np.isclose(rpy_new[1], np.pi):
            rpy_new[1] -= 2 * np.pi
        if rpy_new[2] < 0:
            rpy_new[2] += 2 * np.pi
    return rpy_new


unique_rpys = []
unique_rpys_dict = defaultdict(list)
for i in range(len(rpys)):
    is_unique = True
    for k in unique_rpys_dict:
        angle_diff = get_angular_difference(rpys[i], unique_rpys_dict[k][0])

        if np.isclose(angle_diff, 0):
            is_unique = False
            unique_rpys_dict[k].append(rpys[i])  # not unique, add to existing key
            break
    if is_unique:
        unique_rpys.append(rpys[i])
        unique_rpys_dict[i].append(rpys[i])  # unique, adding new key
    else:
        # check that fix for duplicate rpys works
        if not np.isclose(np.sum(np.array(fix_rpy(rpys[i])) - np.array(unique_rpys_dict[k][0])), 0):
            print "duplicate found: \t{},\t {}".format(list_as_fraction_of_pi(rpys[i]),
                                                       list_as_fraction_of_pi(unique_rpys_dict[k][0]))
            print "Fix:\t{}".format(list_as_fraction_of_pi(fix_rpy(rpys[i])))

            diff = np.array(rpys[i]) - np.array(unique_rpys_dict[k][0])
            print "diff: {}".format(list_as_fraction_of_pi(diff))
            # import IPython
            # IPython.embed()
            # assert np.sum(np.array(fix_rpy(rpys[i])) - np.array(fix_rpy(unique_rpys_dict[k]))) == 0

print "\nNumber of unique rpys: \t{} \n\n".format(len(unique_rpys_dict.keys()))

for k, unique_rpys in unique_rpys_dict.iteritems():
    if len(unique_rpys) > 1:
        print "Equivalent rpys ({}): \t{}".format(len(unique_rpys), map(list_as_fraction_of_pi, unique_rpys))


print "\n\n\nTotal number of rpys: \t{} ".format(len(rpys))
print "Number of unique rpys: \t{} \n".format(len(unique_rpys_dict.keys()))
print "Fraction of rpys that are unique: \t{} \n\n\n".format(Fraction((len(unique_rpys_dict.keys())*1.0/len(rpys))))

import IPython
IPython.embed()
