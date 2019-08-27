'''
# Name: get_master_cmd.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/12/2017
# Edit Date: 12/12/2017
#
# Description:
#   Subscribes to from_master_topic and updates SMACH userdata.
#   Publishes on to_master_topic to notify master of cycle completion.
'''

import rospy
from smach import State
from std_msgs.msg import String


class GetMasterCmd(State):
    """SMACH state: Talks to master node

    Attributes:
        from_master_topic   (str): subscribe to master node
        to_master_topic     (str): publish to master node
        completion_msg      (str): message published on to_master_topic on cycle
                                    completion
        ros_rate            (float)
    """
    def __init__(self, from_master_topic, to_master_topic, completion_msg, ros_rate):
        State.__init__(self, outcomes=['exit','success'],
            output_keys=['depth','offset',
                         'curvature','curvature_ref_depth','curvature_ref_offset',
                         'duration_execute_s','duration_rest_s'])

        # ROS stuff
        self._from_master = rospy.Subscriber(from_master_topic, String, self._master_callback)  #TODO: replace with custom msg type
        self._to_master = rospy.Publisher(to_master_topic, String, queue_size=1)
        self._completion_msg = completion_msg
        self._rate = rospy.Rate(ros_rate)
        self._msg_from_master = None

    def execute(self, userdata):
        self._to_master.publish(self._completion_msg)
        while not rospy.is_shutdown():
            if self._msg_from_master is not None:
                # process msg from master
                self._update_userdata(userdata)
                self._msg_from_master = None
                return 'success'
            self._rate.sleep()
        return 'exit'  # if rospy.is_shutdown()

    def _master_callback(self, msg):
        msg_list = msg.data.split(',')  #TODO: Replace with custom msg type
        self._msg_from_master = msg_list

    def _update_userdata(self, userdata):
        userdata.depth = float(self._msg_from_master[0])
        userdata.offset = float(self._msg_from_master[1])
        userdata.curvature = float(self._msg_from_master[2])
        userdata.curvature_ref_depth = float(self._msg_from_master[3])
        userdata.curvature_ref_offset = float(self._msg_from_master[4])
        userdata.duration_execute_s = float(self._msg_from_master[5])
        userdata.duration_rest_s = float(self._msg_from_master[6])
