"""
# Name: execute_joint_trajectory.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/12/2017
# Edit Date: 12/12/2017
#
# Description:
#   Executes hip and knee joint trajectories.
#   Publishes joint positions on to_hip_topic and to_knee_topic.
#   Subscribe to from_hip_topic and from_knee_topic
"""

import rospy
from smach import State
from std_msgs.msg import Float32


class ExecuteJointTrajectory(State):
    """SMACH state: Executes hip and knee joint trajectories

    Attributes:
        from_hip_topic  (str): subscribes to hip node
        to_hip_topic    (str): publishes to hip node
        from_knee_topic (str): subscibes to knee node
        to_knee_topic   (str): publishes to knee node
        ros_rate        (float)
    """
    def __init__(self, from_hip_topic, to_hip_topic,
                 from_knee_topic, to_knee_topic, ros_rate):
        State.__init__(self, outcomes=['exit','success'],
            input_keys=['duration_execute_s','duration_rest_s',
                        'hip_traj','knee_traj'])

        self._from_hip = rospy.Subscriber(from_hip_topic, Float32, self._hip_callback)
        self._to_hip = rospy.Publisher(to_hip_topic, Float32, queue_size=1)
        self._from_knee = rospy.Subscriber(from_knee_topic, Float32, self._knee_callback)
        self._to_knee = rospy.Publisher(to_knee_topic, Float32, queue_size=1)
        self._rate = rospy.Rate(ros_rate)
        self._hip_angle = None
        self._knee_angle = None

    def execute(self, userdata):
        # execute joint trajectories
        hip_traj = userdata.hip_traj
        print(" hip_traj={}".format(hip_traj))
        knee_traj = userdata.knee_traj
        num_pts = len(hip_traj)
        time_start = rospy.Time.now().to_sec()  # works with simulation time too
        time_end = time_start + userdata.duration_execute_s
        time_incr = (time_end - time_start) / float(num_pts)
        index = 0
        rest = False
        done = False
        while not rospy.is_shutdown():
            if done:
                return 'success'
            time_now = rospy.Time.now().to_sec()
            if time_now > time_end:
                if not rest:
                    rest = True
                    time_end = time_now + userdata.duration_rest_s
                else:
                    done = True
            elif (time_now > time_start + index*time_incr):
                if index < num_pts-1:
                    index += 1
            else:
                self._to_hip.publish(hip_traj[index])
                self._to_knee.publish(knee_traj[index])
            self._rate.sleep()
        return 'exit'  # if rospy.is_shutdown()

    def _hip_callback(self, msg):
        self._hip_angle = msg.data

    def _knee_callback(self, msg):
        self._knee_angle = msg.data
