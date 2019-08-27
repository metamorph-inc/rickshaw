#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: rickshaw_movement_fsm_node_v2.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/13/2017
# Edit Date: 12/13/2017
#
# Description:
#   Finite state machine controlling two legs.
#   Subscribes to /gait_params and /move_direction topics.
#   Subscribes to /from_leftleg and /from_rightleg topics.
#   Publishes on /to_leftleg and /to_rightleg topics.
'''

import sys
import argparse
import math as m
import rospy
from smach import StateMachine, Concurrence
import smach_ros
from rickshaw_movement_states.send_leg_cmd import SendLegCmd
from rickshaw_movement_states.get_gait_params import GetGaitParams


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
    parser.add_argument('-from_master_gait', type=str, required=True, help="type=str, Description='gait params ROS topic to subscribe to'")
    parser.add_argument('-from_master_direction', type=str, required=True, help="type=str, Description='direction ROS topic to subscribe to'")
    parser.add_argument('-from_leftleg', type=str, required=True, help="type=str, Description='left leg ROS topic to subscribe to'")
    parser.add_argument('-to_leftleg', type=str, required=True, help="type=str, Description='left leg ROS topic to publish to'")
    parser.add_argument('-from_rightleg', type=str, required=True, help="type=str, Description='right leg ROS topic to subscribe to'")
    parser.add_argument('-to_rightleg', type=str, required=True, help="type=str, Description='right leg ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def build_homo_con_container(legs_list, mode, generic_states, input_keys):
    """Create & returns a SMACH Concurrence instance for one mode
    Args:
        legs_list       (list of str)
        modes           (str)
        generic_states     (dict of class instances)
    """
    outcome_map = {'success':{leg.upper()+mode.upper():'success' for leg in legs_list}}
    con = Concurrence(outcomes=['success'], default_outcome='success',
        outcome_map=outcome_map, input_keys=input_keys)
    with con:
        for leg in legs_list:
            Concurrence.add(leg.upper()+mode.upper(), generic_states[leg][mode])
    return con


def main():
    # ROS stuff
    rospy.init_node('rickshaw_movement_fsm_node.py')

    parser = parse_args(sys.argv[1:])
    thigh_len = parser.thigh_len
    shin_len = parser.shin_len
    from_master_gait = parser.from_master_gait
    from_master_direction = parser.from_master_direction
    from_leftleg = parser.from_leftleg
    to_leftleg = parser.to_leftleg
    from_rightleg = parser.from_rightleg
    to_rightleg = parser.to_rightleg
    ros_rate = parser.ros_rate

    ### CREATE LEG STATE INSTANCES ###
    legs = ['left', 'right']
    leg_topics = {'left':{'to':to_leftleg,'from':from_leftleg},  #TODO: Clean up later
                  'right':{'to':to_rightleg,'from':from_rightleg}}
    modes = ['center', 'lift', 'plant', 'push']
    generic_states = {}
    for leg in legs:
        modes_dict = {}
        for mode in modes:
            new_state = SendLegCmd('done', leg, mode, leg_topics[leg]['from'], leg_topics[leg]['to'], ros_rate)
            modes_dict[mode] = new_state
        generic_states[leg] = modes_dict

    forward_start = SendLegCmd('done', 'left', 'lift', leg_topics['left']['from'], leg_topics['left']['to'], ros_rate)
    forward_a = SendLegCmd('done', 'left', 'plant', leg_topics['left']['from'], leg_topics['left']['to'], ros_rate)
    forward_b = SendLegCmd('done', 'right', 'lift', leg_topics['right']['from'], leg_topics['right']['to'], ros_rate)
    forward_c = SendLegCmd('done', 'left', 'push', leg_topics['left']['from'], leg_topics['left']['to'], ros_rate)
    forward_d = SendLegCmd('done', 'right', 'plant', leg_topics['right']['from'], leg_topics['right']['to'], ros_rate)
    forward_e = SendLegCmd('done', 'left', 'lift', leg_topics['left']['from'], leg_topics['left']['to'], ros_rate)
    forward_f = SendLegCmd('done', 'right', 'push', leg_topics['right']['from'], leg_topics['right']['to'], ros_rate)

    get_gait_params = GetGaitParams(from_master_gait, ros_rate)

    ### CREATE CONCURRENCE INSTANCES ###
    input_keys = ['lift_params', 'plant_params', 'push_params',
                  'duration_execute_s', 'duration_rest_s', 'phase_shift_s','msg_consumed_cnt']
    con_start = build_homo_con_container(legs, 'center', generic_states, input_keys)
    con_center = build_homo_con_container(legs, 'center', generic_states, input_keys)
    con_lift = build_homo_con_container(legs, 'lift', generic_states, input_keys)
    con_plant = build_homo_con_container(legs, 'plant', generic_states, input_keys)
    con_push = build_homo_con_container(legs, 'push', generic_states, input_keys)

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
    top.userdata.msg_consumed_cnt = {leg:0 for leg in legs}

    remapping_dict = {key:key for key in input_keys}

    with top:
        ### ADD STATE/CONCURRENCE INSTANCES TO THE TOP SM ###
        StateMachine.add('START', con_start, transitions={'success':'FORWARD_START'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_START', forward_start, transitions={'success':'PRE_FORWARD_A','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_A', get_gait_params, transitions={'success':'FORWARD_A'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_A', forward_a, transitions={'success':'PRE_FORWARD_B','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_B', get_gait_params, transitions={'success':'FORWARD_B'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_B', forward_b, transitions={'success':'PRE_FORWARD_C','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_C', get_gait_params, transitions={'success':'FORWARD_C'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_C', forward_c, transitions={'success':'PRE_FORWARD_D','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_D', get_gait_params, transitions={'success':'FORWARD_D'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_D', forward_d, transitions={'success':'PRE_FORWARD_E','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_E', get_gait_params, transitions={'success':'FORWARD_E'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_E', forward_e, transitions={'success':'PRE_FORWARD_F','exit':'success'},
                            remapping=remapping_dict)
        StateMachine.add('PRE_FORWARD_F', get_gait_params, transitions={'success':'FORWARD_F'},
                            remapping=remapping_dict)
        StateMachine.add('FORWARD_F', forward_f, transitions={'success':'PRE_FORWARD_A','exit':'success'},
                            remapping=remapping_dict)


        #FIXME: This did not work like I thought it might...
        #StateMachine.add('START', con_start, transitions={'success':'FORWARD_A'},
        #                                      remapping=remapping_dict)
        #StateMachine.add('FORWARD_A', con_lift, transitions={'success':'FORWARD_B'},
        #                                        remapping=remapping_dict)
        #StateMachine.add('FORWARD_B', con_plant, transitions={'success':'FORWARD_C'},
        #                                         remapping=remapping_dict)
        #StateMachine.add('FORWARD_C', con_push, transitions={'success':'FORWARD_A'},
        #                                        remapping=remapping_dict)

    sis = smach_ros.IntrospectionServer('rickshaw_movement_fsm_node'
        + str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    user_input = raw_input("Please press the 'Return/Enter' key to start executing \
        - pkg: rickshaw_movement_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: rickshaw_movement_fsm_node.py | node: "
        + str(rospy.get_name()) + "\n")

    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
