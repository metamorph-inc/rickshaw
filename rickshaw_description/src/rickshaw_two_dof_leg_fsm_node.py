#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: rickshaw_two_dof_leg_fsm_node_v2.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/12/2017
# Edit Date: 12/12/2017
#
# Description:
#   Finite state machine controlling the position and movements of a
#   2-dof 'knee-up' leg with a z-axis hip joint and a z-axis knee joint
#
#                    z
#                     \
#     *     <-   -   - * -- y
#      \               |                     z
#       \              x                    /
#        x     <-   -   -   -   -   - y -- x
#        |                                 |
#        |                                 x
*        O
'''

import sys
import argparse
import rospy
from smach import StateMachine
import smach_ros
from rickshaw_two_dof_leg_states.get_master_cmd import GetMasterCmd
from rickshaw_two_dof_leg_states.generate_joint_trajectory import GenerateJointTrajectory
from rickshaw_two_dof_leg_states.execute_joint_trajectory import ExecuteJointTrajectory


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-side', type=str, choices=['left','right'], required=True, help="type=str, Description='side the leg is mounted on'")
    parser.add_argument('-thigh_len', type=float, required=True, help="type=float, Description='length of thigh section in [m]'")
    parser.add_argument('-shin_len', type=float, required=True, help="type=float, Description='length of shin section in [m]'")
    parser.add_argument('-num_traj_pts', type=int, default=10, help="type=int, Description='number of trajectory points generated'")
    parser.add_argument('-from_hip', type=str, required=True, help="type=str, Description='hip joint ROS topic to subscribe to'")
    parser.add_argument('-to_hip', type=str, required=True, help="type=str, Description='hip joint ROS topic to publish to'")
    parser.add_argument('-from_knee', type=str, required=True, help="type=str, Description='knee joint ROS topic to subscribe to'")
    parser.add_argument('-to_knee', type=str, required=True, help="type=str, Description='knee joint ROS topic to publish to'")
    parser.add_argument('-from_master', type=str, required=True, help="type=str, Description='master ROS topic to subscribe to'")
    parser.add_argument('-to_master', type=str, required=True, help="type=str, Description='master ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('rickshaw_two_dof_leg_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    side = parser.side
    thigh_len = parser.thigh_len
    shin_len = parser.shin_len
    num_traj_pts = parser.num_traj_pts
    from_hip_topic = parser.from_hip
    to_hip_topic = parser.to_hip
    from_knee_topic = parser.from_knee
    to_knee_topic = parser.to_knee
    from_master_topic = parser.from_master
    to_master_topic = parser.to_master
    ros_rate = parser.ros_rate

    ### CREATE LEG STATE INSTANCES ###
    get_master_cmd = GetMasterCmd(from_master_topic, to_master_topic, 'done', ros_rate)
    gen_joint_traj = GenerateJointTrajectory(side, thigh_len, shin_len, num_traj_pts)
    exec_joint_traj = ExecuteJointTrajectory(from_hip_topic, to_hip_topic,
                        from_knee_topic, to_knee_topic, ros_rate)

    ### CREATE TOP SM ###
    top = StateMachine(outcomes=['success'])

    ### INITIALIZE USERDATA ###
    # Updated by GetMasterCmd
    top.userdata.depth = 0.95*thigh_len + 0.95*shin_len
    top.userdata.offset = 0.0
    top.userdata.curvature = 0.0
    top.userdata.curvature_ref_depth = 0.5
    top.userdata.curvature_ref_offset = 0.0
    top.userdata.duration_execute_s = 0.5
    top.userdata.duration_rest_s = 0.5
    # Updated by GenerateJointTrajectory
    top.userdata.hip_traj = []
    top.userdata.knee_traj = []
    top.userdata.prev_depth = 0.95*thigh_len + 0.95*shin_len
    top.userdata.prev_offset = 0.0
    # Updated by ExecuteJointTrajectory
    #   nothing

    remapping_dict = {'depth':'depth','offset':'offset','curvature':'curvature',
                      'curvature_ref_depth':'curvature_ref_depth',
                      'curvature_ref_offset':'curvature_ref_offset',
                      'duration_execute_s':'duration_execute_s',
                      'duration_rest_s':'duration_rest_s',
                      'hip_traj':'hip_traj','knee_traj':'knee_traj',
                      'prev_depth':'prev_depth','prev_offset':'prev_offset'}

    with top:
        ### ADD LEG STATES TO THE TOP SM ###
        StateMachine.add('GET_MASTER_CMD', get_master_cmd,
            transitions={'exit':'GEN_JOINT_TRAJ','success':'GEN_JOINT_TRAJ'},
            remapping=remapping_dict)

        StateMachine.add('GEN_JOINT_TRAJ', gen_joint_traj,
            transitions={'exit':'EXEC_JOINT_TRAJ', 'success':'EXEC_JOINT_TRAJ'},
            remapping=remapping_dict)

        StateMachine.add('EXEC_JOINT_TRAJ', exec_joint_traj,
            transitions={'exit':'GET_MASTER_CMD','success':'GET_MASTER_CMD'},
            remapping=remapping_dict)

    sis = smach_ros.IntrospectionServer('rickshaw_two_dof_leg_fsm_node'
        + str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()


    user_input = raw_input("Please press the 'Return/Enter' key to start executing \
        - pkg: rickshaw_two_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: rickshaw_two_dof_leg_fsm_node.py | node: "
        + str(rospy.get_name()) + "\n")

    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
