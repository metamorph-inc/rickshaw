'''
# Name: generate_joint_trajectory.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/12/2017
# Edit Date: 12/12/2017
#
# Description:
#   Generate hip and knee joint trajectories from SMACH userdata.
#   Test version here: https://repl.it/@jcoombe/Rick-2-dof-reverse-kinematics
'''

import math as m
import rospy
from smach import State


class GenerateJointTrajectory(State):
    """SMACH state: Generates hip and knee joint trajectories from SMACH userdata

    Attributes:
        side            (str): left or right
        thigh_len       (float): length of thigh segment [m]
        shin_len        (float): length of thigh segment [m]
        num_traj_pts    (int): number of trajectory pts generated per joint
    """
    def __init__(self, side, thigh_len, shin_len, num_traj_pts):
        State.__init__(self, outcomes=['exit','success'],
            input_keys=['depth','offset','curvature','curvature_ref_depth',
                        'curvature_ref_offset','prev_depth','prev_offset'],
            output_keys=['hip_traj','knee_traj','prev_depth','prev_offset'])

        self._side = side
        self._thigh_len = thigh_len
        self._shin_len = shin_len
        self._num_traj_pts = num_traj_pts

        self._hip_traj = []
        self._knee_traj = []
        self._prev_depth = None
        self._prev_offset = None

    def execute(self, userdata):
        end_pt_traj = self._gen_end_pt_traj(userdata, self._num_traj_pts)
        print("end_pt_traj={}".format(end_pt_traj))
        for pt in end_pt_traj:
            x_val = pt[0]
            y_val = pt[1]
            joint_angles = self._ik(0.0, 0.0, x_val, y_val, self._thigh_len, self._shin_len)
            print("hip_angles={},knee_angles={}".format(joint_angles[0],joint_angles[1]))
            if self._side == 'left':
                joint_angles = [-angle for angle in joint_angles]
            self._hip_traj.append(joint_angles[0])
            self._knee_traj.append(joint_angles[1])
        self._update_userdata(userdata)
        self._on_exit()
        return 'success'

    def _gen_end_pt_traj(self, userdata, num_pts):
        y_s = userdata.prev_offset
        x_s = userdata.prev_depth
        y_f = userdata.offset
        x_f = userdata.depth
        curvature = userdata.curvature
        center_coord = (userdata.curvature_ref_depth, userdata.curvature_ref_offset)

        if curvature < 0:
            raise ValueError("curvature={}! curvature parameter must be >= 0.".format(curvature))

        elif curvature == 0:  # straight line
            x_dif = x_f - x_s
            y_dif = y_f - y_s
            x_delta = x_dif / float(num_pts)
            y_delta = y_dif / float(num_pts)
            trajectory = []
            for i in range(num_pts):
                pt = (x_s + (i+1)*x_delta, y_s + (i+1)*y_delta)
                trajectory.append(pt)
            return trajectory

        else:  # arc
            # Ax + By + C = D
            a = y_s - y_f
            b = x_f - x_s
            c = x_s*y_f - x_f*y_s
            d = a*center_coord[0] + b*center_coord[1] + c
            sign_center = int(d >= 0.0)

            radius = 1.0/curvature

            # http://mathforum.org/library/drmath/view/53027.html
            line_length = m.sqrt((x_f-x_s)**2.0 + (y_f-y_s)**2.0)
            x_mid = (x_f+x_s)/2.0
            y_mid = (y_f+y_s)/2.0
            x_perp_dir = (y_s-y_f) / line_length
            y_perp_dir = (x_f-x_s) / line_length
            dist = m.sqrt(radius**2.0 - (line_length/2.0)**2.0)
            option1_x = x_mid + dist*x_perp_dir
            option1_y = y_mid + dist*y_perp_dir
            option2_x = x_mid - dist*x_perp_dir
            option2_y = y_mid - dist*y_perp_dir

            # determine curve's center coordinates
            x_curve_center = None
            y_curve_center = None
            sign_option1 = int(a*option1_x + b*option1_y + c >= 0.0)
            if sign_option1 == sign_center:
                x_curve_center = option1_x
                y_curve_center = option1_y
            else:
                x_curve_center = option2_x
                y_curve_center = option2_y

            # determine start and end angles
            theta_start = m.atan2(y_s-y_curve_center, x_s-x_curve_center)
            theta_finish = m.atan2(y_f-y_curve_center, x_f-x_curve_center)

            # determine dif (sign)
            theta_dif_sign = None
            theta_dif = None
            if theta_start >= 0.0:
                if theta_finish >= theta_start or theta_finish < theta_start - m.pi:
                    theta_dif_sign = +1.0
                    if theta_finish >= 0.0:
                        theta_dif = theta_finish - theta_start
                    else:
                        theta_dif = (m.pi + theta_finish) + (m.pi - theta_start)
                else:
                    theta_dif_sign = -1.0
                    if theta_finish >= 0:
                        theta_dif = theta_start - theta_finish
                    else:
                        theta_dif = -theta_finish + theta_start
            else:  # theta_start < 0.0
                if theta_finish <= theta_start or theta_finish > theta_start + m.pi:
                    theta_dif_sign = -1.0
                    if theta_finish <= 0:
                        theta_dif = theta_start - theta_finish
                    else:
                        theta_dif = (m.pi - theta_finish) + (m.pi + theta_start)
                else:
                    theta_dif_sign = +1.0
                    if theta_finish <= 0:
                        theta_dif = theta_finish - theta_start
                    else:
                        theta_dif = theta_finish - theta_start
            theta_delta = theta_dif*theta_dif_sign/float(num_pts)

            # generate trajectory via parametric circle expression
            trajectory = []
            for i in range(num_pts):
                theta_add = theta_delta*(i+1)
                theta = (theta_start + theta_add)%(2*m.pi)
                x = x_curve_center + radius*m.cos(theta)
                y = y_curve_center + radius*m.sin(theta)
                trajectory.append((x, y))
            return trajectory

    def _ik(self, x0, y0, xf, yf, thigh_len, shin_len):
        rho1 = m.acos((thigh_len**2.0 + shin_len**2.0 - (xf-x0)**2.0 - (yf-y0)**2.0) / (2.0*thigh_len*shin_len))
        theta1 = m.pi - rho1
        phi0 = m.pi/2 + m.atan2(xf, yf) - m.atan2(shin_len*m.sin(theta1), thigh_len + shin_len*m.cos(theta1))
        theta0 = None
        if phi0 >= 0:
            theta0 = m.pi - phi0
        else:  # phi0 < 0
            theta0 = -m.pi - phi0
        return theta0, theta1

    def _update_userdata(self, userdata):
        userdata.hip_traj = self._hip_traj
        userdata.knee_traj = self._knee_traj
        userdata.prev_depth = userdata.depth
        userdata.prev_offset = userdata.offset

    def _on_exit(self):
        self._hip_traj = []
        self._knee_traj = []
