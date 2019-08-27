#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smach import StateMachine, State
from std_msgs.msg import String


### DEFINE STATE CLASSES ###
class UpdateName(State):
    def __init__(self, my_name_topic):
        State.__init__(self, outcomes=['success'], output_keys=['my_name'])

        self._my_name_sub = rospy.Subscriber(my_name_topic, String, self._my_name_callback)
        self._msg_data = None

    def execute(self, userdata):
        userdata.my_name = self._msg_data
        return 'success'

    def _my_name_callback(self, msg):
        print('received msg.data={}'.format(msg.data))
        self._msg_data = msg.data

class PrintName(State):
    def __init__(self, duration):
        State.__init__(self, outcomes=['success'], input_keys=['my_name'])

        self._duration = duration

    def execute(self, userdata):
        time_start = rospy.Time.now().to_sec()
        time_end = time_start + self._duration
        print("My name is {}!".format(userdata.my_name))
        while not rospy.is_shutdown():
            time_now = rospy.Time.now().to_sec()
            if time_now > time_end:
                return 'success'


def main():
    rospy.init_node('update_userdata_from_old_state')

    state_a = UpdateName('name_topic')
    state_b = PrintName(duration=4.0)
    state_c = PrintName(duration=2.0)

    top = StateMachine(outcomes=['success'])

    top.userdata.my_name = "Joseph"

    with top:
        StateMachine.add('A', state_a, transitions={'success':'B'}, remapping={'my_name':'my_name'})
        StateMachine.add('B', state_b, transitions={'success':'C'}, remapping={'my_name':'my_name'})
        StateMachine.add('C', state_c, transitions={'success':'A'}, remapping={'my_name':'my_name'})

    top.execute()

if __name__ == '__main__':
    main()
