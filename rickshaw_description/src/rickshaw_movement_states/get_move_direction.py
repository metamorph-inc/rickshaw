'''
# Name: get_move_direction.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/13/2017
# Edit Date: 12/13/2017
#
# Description:
#   Subscribes to from_master_direction topic and
#   returns a state transition based on received trigger messages.
'''

import rospy
from smach import State
from std_msgs.msg import String


class GetMoveDirection(State):
    #TODO
