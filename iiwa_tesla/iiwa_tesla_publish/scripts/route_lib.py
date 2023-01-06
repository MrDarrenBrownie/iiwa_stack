#!/usr/bin/python3

import copy
import geometry_msgs.msg

from moveit_commander.conversions import pose_to_list
from math import pi, dist, fabs, cos

def home_pose_singularity_safe():
    return [0.1 for x in range(7)]

def start_position_middle():
    joint_pose = [0.0 for x in range(7)]
    joint_pose[3] = -75.0 *(pi/180)
    joint_pose[5] = 38.0 *(pi/180)
    return joint_pose

def relative_movement_from_start(last_start_pose, routes):
    
    start_pose = copy.deepcopy(last_start_pose)

    rel_x = 0.1
    rel_y = 0.1
    
    init_1_x = -0.1
    init_1_y = 0.1
    init_2_x = 0.05
    init_2_y = init_1_y
    init_3_x = init_2_x
    init_3_y = - (init_1_y + rel_y)
    init_4_x = init_1_x
    init_4_y = - (init_1_y + rel_y)

    init_x = [init_1_x, init_2_x, init_3_x, init_4_x]
    init_y = [init_1_y, init_2_y, init_3_y, init_4_y]

    for pos, (ix,iy) in enumerate(zip(init_x, init_y)):
        route_n = []

        position_init = copy.deepcopy(start_pose)
        position_init.position.x += ix
        position_init.position.y += iy
        route_n.append(position_init)

        position2 = copy.deepcopy(position_init)
        position2.position.x += rel_x
        route_n.append(position2)
   
        position3 = copy.deepcopy(position2)
        position3.position.y += rel_y
        route_n.append(position3)

        position4 = copy.deepcopy(position3)
        position4.position.x -= rel_x
        route_n.append(position4)
        routes['route'+str(pos)] = route_n

    return routes


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def is_goal_reached(move_group, goal, tolerance):
    if type(goal) is list:
        print('wait for movement to be finished')
        while not all_close(goal, move_group.get_current_joint_values(), tolerance):
            pass
            # print('wait for movement to be finished')
    elif type(goal) is geometry_msgs.msg.Pose:
        print('wait for movement to be finished')
        while not all_close(goal, move_group.get_current_pose().pose, tolerance):
            pass
            # print('wait for movement to be finished')