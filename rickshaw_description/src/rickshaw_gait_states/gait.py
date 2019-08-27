'''
# Name: gait.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/14/2017
# Edit Date: 12/14/2017
#
# Description:
#   #TODO
'''

import rospy
from smach import State
from std_msgs.msg import String


class Gait(State):
    #TODO
    """
    """
    def __init__(self, from_master_topic, gait_params_topic, params, ros_rate):
        State.__init__(self, outcomes=['success','exit'],
            output_keys=['lift_params', 'plant_params', 'push_params',
                         'duration_execute_s', 'duration_rest_s', 'phase_shift_s'])

        # ROS stuff
        self._from_master = rospy.Subscriber(from_master_topic, String, self._from_master)
        self._gait_params_pub = rospy.Publisher(gait_params_topic, String, queue_size=1)
        self._rate = rospy.Rate(ros_rate)
        self._active_flag = False
        self._msg_from_master = None

        self._lift_params = params[0]
        self._plant_params = params[1]
        self._push_params = params[2]
        self._duration_execute_s = params[3]
        self._duration_rest_s = params[4]
        self._phase_shift_s = params[5]

    def execute(self, userdata):
        self._active_flag = True
        while not rospy.is_shutdown():
            if self._msg_from_master is not None:
                self._msg_from_master = None
                self._update_userdata(userdata)
                self._clean_up()
                return 'success'
            else:
                self._publish_gait_params(userdata)
            self._rate.sleep()
        return 'exit'  # if rospy.is_shutdown()

    def _from_master(self, msg):
        if self._active_flag:
            self._msg_from_master = msg.data

    def _publish_gait_params(self, userdata):
        data = []
        data.append(self._lift_params['depth'])
        data.append(self._lift_params['offset'])
        data.append(self._lift_params['curvature'])
        data.append(self._lift_params['curvature_ref_depth'])
        data.append(self._lift_params['curvature_ref_offset'])
        data.append(self._plant_params['depth'])
        data.append(self._plant_params['offset'])
        data.append(self._plant_params['curvature'])
        data.append(self._plant_params['curvature_ref_depth'])
        data.append(self._plant_params['curvature_ref_offset'])
        data.append(self._push_params['depth'])
        data.append(self._push_params['offset'])
        data.append(self._push_params['curvature'])
        data.append(self._push_params['curvature_ref_depth'])
        data.append(self._push_params['curvature_ref_offset'])
        data.append(self._duration_execute_s)
        data.append(self._duration_rest_s)
        data.append(self._phase_shift_s)
        data_str = ','.join([str(val) for val in data])
        self._gait_params_pub.publish(data_str)

    def _update_userdata(self, userdata):
        userdata.lift_params = self._lift_params
        userdata.plant_params = self._plant_params
        userdata.push_params = self._push_params
        userdata.duration_execute_s = self._duration_execute_s
        userdata.duration_rest_s = self._duration_rest_s
        userdata.phase_shift_s = self._phase_shift_s

    def _clean_up(self):
        self._active_flag = False
