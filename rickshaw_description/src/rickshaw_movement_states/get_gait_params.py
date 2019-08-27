'''
# Name: get_gait_params.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/13/2017
# Edit Date: 12/13/2017
#
# Description:
#   Subscribes to from_master_params topic and updates userdata.
'''

import rospy
from smach import State
from std_msgs.msg import String


class GetGaitParams(State):
    #TODO
    """
    """
    def __init__(self, from_master_gait_topic, ros_rate):
        State.__init__(self, outcomes=['success'],
            output_keys=['lift_params', 'plant_params', 'push_params',
                         'duration_execute_s', 'duration_rest_s', 'phase_shift_s'])

        # ROS stuff
        self._from_master_gait = rospy.Subscriber(from_master_gait_topic, String, self._master_gait_callback)
        self._rate = rospy.Rate(ros_rate)
        self._msg_from_master = None

    def execute(self, userdata):
        if self._msg_from_master is not None:
            # process msg from _master
            self._update_userdata(userdata)
            self._msg_from_master = None
        return 'success'

    def _master_gait_callback(self, msg):
        msg_list = msg.data.split(',')  #TODO: Replace with custom msg type
        self._msg_from_master = msg_list

    def _update_userdata(self, userdata):
        lift_dict = {}
        lift_dict['depth'] = float(self._msg_from_master[0])
        lift_dict['offset'] = float(self._msg_from_master[1])
        lift_dict['curvature'] = float(self._msg_from_master[2])
        lift_dict['curvature_ref_depth'] = float(self._msg_from_master[3])
        lift_dict['curvature_ref_offset'] = float(self._msg_from_master[4])
        userdata.lift_params = lift_dict
        plant_dict = {}
        plant_dict['depth'] = float(self._msg_from_master[5])
        plant_dict['offset'] = float(self._msg_from_master[6])
        plant_dict['curvature'] = float(self._msg_from_master[7])
        plant_dict['curvature_ref_depth'] = float(self._msg_from_master[8])
        plant_dict['curvature_ref_offset'] = float(self._msg_from_master[9])
        userdata.plant_params = plant_dict
        push_dict = {}
        push_dict['depth'] = float(self._msg_from_master[10])
        push_dict['offset'] = float(self._msg_from_master[11])
        push_dict['curvature'] = float(self._msg_from_master[12])
        push_dict['curvature_ref_depth'] = float(self._msg_from_master[13])
        push_dict['curvature_ref_offset'] = float(self._msg_from_master[14])
        userdata.push_params = push_dict
        userdata.duration_execute_s = float(self._msg_from_master[15])
        userdata.duration_rest_s = float(self._msg_from_master[16])
        userdata.phase_shift_s = float(self._msg_from_master[17])
