#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: rickshaw_gait_fsm_node_v2.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/14/2017
# Edit Date: 12/14/2017
#
# Description:
#   #TODO
'''

import sys
import argparse
import math as m
import rospy
from smach import StateMachine, Concurrence
import smach_ros
from rickshaw_gait_states.gait import Gait
#from rickshaw_gait_states.transition import Transition


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-thigh_len', type=float, required=True, help="type=float, Description='length of thigh section in [m]'")
    parser.add_argument('-shin_len', type=float, required=True, help="type=float, Description='length of shin section in [m]'")
    parser.add_argument('-from_master', type=str, required=True, help="type=str, Description='master ROS topic to subscribe to'")
    parser.add_argument('-gait_params', type=str, required=True, help="type=str, Description='ROS topic to publish gait params to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('rickshaw_gait_fsm_node.py')

    parser = parse_args(sys.argv[1:])
    thigh_len = parser.thigh_len
    shin_len = parser.shin_len
    from_master = parser.from_master
    gait_params = parser.gait_params
    ros_rate = parser.ros_rate

    ### CREATE LEG STATE INSTANCES ###
    lift_dict = {}
    lift_dict['depth'] = thigh_len
    lift_dict['offset'] = 0.0
    lift_dict['curvature'] = 0.0
    lift_dict['curvature_ref_depth'] = 10.0
    lift_dict['curvature_ref_offset'] = 0.0
    plant_dict = {}
    plant_dict['depth'] = (thigh_len+shin_len)/m.sqrt(2.0) - 0.01
    plant_dict['offset'] = m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0) - 0.25
    plant_dict['curvature'] = 0.0
    plant_dict['curvature_ref_depth'] = 0.0
    plant_dict['curvature_ref_offset'] = -10.0
    push_dict = {}
    push_dict['depth'] = (thigh_len+shin_len)/m.sqrt(2.0) - 0.01
    push_dict['offset'] = -m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0) + 0.2  # TODO: Mistake - fortunate?
    push_dict['curvature'] = 0.0
    push_dict['curvature_ref_depth'] = -10.0
    push_dict['curvature_ref_offset'] = 0.0
    params = [lift_dict, plant_dict, push_dict, 2.0, 0.0, 0.0]

    state_a = Gait(from_master, gait_params, params, ros_rate)

    params[3] = 0.3

    state_b = Gait(from_master, gait_params, params, ros_rate)

    params[1]['offset'] = m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0)
    params[2]['offset'] = 0.0

    state_c = Gait(from_master, gait_params, params, ros_rate)

    params[1]['offset'] = 0.0
    params[2]['offset'] = -m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0) + 0.2
    params[3] = 1.0
    state_d = Gait(from_master, gait_params, params, ros_rate)

    params[3] = 0.1
    state_e = Gait(from_master, gait_params, params, ros_rate)

    ### CREATE CONCURRENCE INSTANCES ###
    input_keys = ['lift_params', 'plant_params', 'push_params',
                  'duration_execute_s', 'duration_rest_s', 'phase_shift_s']
    remapping_dict = {key:key for key in input_keys}

    ### CREATE TOP SM ###
    top = StateMachine(outcomes=['success'])

    ### INITIALIZE USERDATA ###
    lift_dict = {}
    lift_dict['depth'] = thigh_len
    lift_dict['offset'] = 0.0
    lift_dict['curvature'] = 0.0
    lift_dict['curvature_ref_depth'] = 10.0
    lift_dict['curvature_ref_offset'] = 0.0
    top.userdata.lift_params = lift_dict
    plant_dict = {}
    plant_dict['depth'] = (thigh_len+shin_len)/m.sqrt(2.0) - 0.01
    plant_dict['offset'] = m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0) - 0.25
    plant_dict['curvature'] = 0.0
    plant_dict['curvature_ref_depth'] = 0.0
    plant_dict['curvature_ref_offset'] = -10.0
    top.userdata.plant_params = plant_dict
    push_dict = {}
    push_dict['depth'] = (thigh_len+shin_len)/m.sqrt(2.0) - 0.01
    push_dict['offset'] = -m.sqrt(thigh_len+shin_len**2.0 - plant_dict['depth']**2.0) + 0.2
    push_dict['curvature'] = 0.0
    push_dict['curvature_ref_depth'] = -10.0
    push_dict['curvature_ref_offset'] = 0.0
    top.userdata.push_params = push_dict
    top.userdata.duration_execute_s = 0.2 #2.0
    top.userdata.duration_rest_s = 0.0
    top.userdata.phase_shift_s = 0.0  # - means right leads left; + means right lags left

    with top:
        ### ADD STATE/CONCURRENCE INSTANCES TO THE TOP SM ###
        StateMachine.add('STATE_A', state_a, transitions={'success':'STATE_B','exit':'success'},
            remapping=remapping_dict)
        StateMachine.add('STATE_B', state_b, transitions={'success':'STATE_C','exit':'success'},
            remapping=remapping_dict)
        StateMachine.add('STATE_C', state_c, transitions={'success':'STATE_D','exit':'success'},
            remapping=remapping_dict)
        StateMachine.add('STATE_D', state_d, transitions={'success':'STATE_E','exit':'success'},
            remapping=remapping_dict)
        StateMachine.add('STATE_E', state_e, transitions={'success':'STATE_A','exit':'success'},
            remapping=remapping_dict)

    sis = smach_ros.IntrospectionServer('rickshaw_gait_fsm_node'
        + str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    user_input = raw_input("Please press the 'Return/Enter' key to start executing \
        - pkg: rickshaw_gait_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: rickshaw_gait_fsm_node.py | node: "
        + str(rospy.get_name()) + "\n")

    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
