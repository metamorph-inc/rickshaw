'''
# Name: send_leg_cmd.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/13/2017
# Edit Date: 12/13/2017
#
# Description:
#   Sends a cmd to leg
'''

import rospy
from smach import State
from std_msgs.msg import String


class SendLegCmd(State):
    """SMACH state: Waits for leg to complete previous action, then send new cmd
        done_msg        (str):  expected leg completion message
        side            (str):  left, right
        mode            (str):  lift, plant, push, center
        from_leg_topic  (str):  subscribe to leg node
        to_leg_topic    (str):  publish to leg node
        ros_rate        (float)
    """
    def __init__(self, done_msg, side, mode, from_leg_topic, to_leg_topic, ros_rate):
        State.__init__(self, outcomes=['exit','success'],
            input_keys=['lift_params','plant_params','push_params',
                        'duration_execute_s','duration_rest_s','phase_shift_s',
                        'msg_consumed_cnt'],
            output_keys=['msg_consumed_cnt'])

        # ROS stuff
        self._done_msg = done_msg
        self._side = side
        self._mode = mode
        self._from_leg = rospy.Subscriber(from_leg_topic, String, self._leg_callback)
        self._to_leg = rospy.Publisher(to_leg_topic, String, queue_size=1)
        self._rate = rospy.Rate(ros_rate)
        self._msg_from_leg = None
        self._done_msg_received = False
        self._msg_received_cnt = 0
        self._msg_consumed_cnt = 0
        
    def execute(self, userdata):
        self._msg_consumed_cnt = userdata.msg_consumed_cnt[self._side]
        if self._msg_from_leg is not None and self._msg_received_cnt > self._msg_consumed_cnt:
            pass  # Unhandled msg
        else:
            self._msg_from_leg = None
        time_start = rospy.Time.now().to_sec()
        delay = 0.0
        if userdata.phase_shift_s < 0.0 and self._side == 'left':
            delay = -userdata.phase_shift_s
        elif userdata.phase_shift_s > 0.0 and self._side == 'right':
            delay = userdata.phase_shift_s
        leg_done = False
        if self._done_msg is None:
            leg_done = True
        delay_done = False

        while not rospy.is_shutdown():
            time_now = rospy.Time.now().to_sec()
            if not delay_done and time_now > time_start + delay:
                delay_done = True
            if self._msg_from_leg is not None:
                self._msg_consumed_cnt += 1
                if self._msg_from_leg == self._done_msg:
                    self._done_msg_received = True
                    leg_done = True
                self._msg_from_leg = None
            if delay_done and leg_done:
                self._send_data_to_leg(userdata)
                userdata.msg_consumed_cnt[self._side] = self._msg_consumed_cnt
                self._clean_up()
                return 'success'
            self._rate.sleep()
        return 'exit'  # if rospy.is_shutdown()

    def _send_data_to_leg(self, userdata):
        mode = self._mode
        data = []
        if mode == 'lift':
            data.append(userdata.lift_params['depth'])
            data.append(userdata.lift_params['offset'])
            data.append(userdata.lift_params['curvature'])
            data.append(userdata.lift_params['curvature_ref_depth'])
            data.append(userdata.lift_params['curvature_ref_offset'])
            data.append(userdata.duration_execute_s)
            data.append(userdata.duration_rest_s)
        elif mode == 'plant':
            data.append(userdata.plant_params['depth'])
            data.append(userdata.plant_params['offset'])
            data.append(userdata.plant_params['curvature'])
            data.append(userdata.plant_params['curvature_ref_depth'])
            data.append(userdata.plant_params['curvature_ref_offset'])
            data.append(userdata.duration_execute_s)
            data.append(userdata.duration_rest_s)
        elif mode == 'push':
            data.append(userdata.push_params['depth'])
            data.append(userdata.push_params['offset'])
            data.append(userdata.push_params['curvature'])
            data.append(userdata.push_params['curvature_ref_depth'])
            data.append(userdata.push_params['curvature_ref_offset'])
            data.append(userdata.duration_execute_s)
            data.append(userdata.duration_rest_s)
        elif mode == 'center':
            data.append((userdata.plant_params['depth']+userdata.push_params['depth']) / 2.0)
            data.append(0.0)
            data.append(0.0)
            data.append(0.0)
            data.append(0.0)
            data.append(userdata.duration_execute_s)
            data.append(userdata.duration_rest_s)
        else:
            print('Unrecognized mode={}!!!'.format(mode))
        data_str = ','.join([str(val) for val in data])
        self._to_leg.publish(data_str)

    def _leg_callback(self, msg):
        self._msg_received_cnt += 1
        if not self._done_msg_received:
            self._msg_from_leg = msg.data

    def _clean_up(self):
        self._done_msg_received = False
