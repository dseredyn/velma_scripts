#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('barrett_hand_controller')

import rospy
import tf

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from threading import Lock

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *
from controller_manager_msgs.srv import *

import geometry_msgs.msg

import PyKDL
import math
from numpy import *
import numpy as np

import copy

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

import velma_fk_ik

# reference frames:
# B - robot's base
# W - wrist
# E - gripper
# F - finger distal link
# T - tool

class Velma:
    """
Class for velma robot.
"""

    def getPressureSensorsInfoClient(self):
        try:
            if not hasattr(self, 'get_pressure_sensors_info') or self.get_pressure_sensors_info == None:
                rospy.wait_for_service('/' + self.prefix + '_hand/get_pressure_info')
                self.get_pressure_sensors_info = rospy.ServiceProxy('/' + self.prefix + '_hand/get_pressure_info', BHGetPressureInfo)
            resp = self.get_pressure_sensors_info()
            return resp.info
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def calibrateTactileSensors(self):
        try:
            if not hasattr(self, 'calibrate_tactile_sensors') or self.calibrate_tactile_sensors == None:
                rospy.wait_for_service('/' + self.prefix + '_hand/calibrate_tactile_sensors')
                self.calibrate_tactile_sensors = rospy.ServiceProxy('/' + self.prefix + '_hand/calibrate_tactile_sensors', Empty)
            resp = self.calibrate_tactile_sensors()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def resetFingers(self):
        try:
            if not hasattr(self, 'reset_fingers') or self.reset_fingers == None:
                rospy.wait_for_service('/' + self.prefix + '_hand/reset_fingers')
                self.reset_fingers = rospy.ServiceProxy('/' + self.prefix + '_hand/reset_fingers', Empty)
            resp = self.reset_fingers()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setMedianFilter(self, samples):
        try:
            if not hasattr(self, 'set_median_filter') or self.set_median_filter == None:
                rospy.wait_for_service('/' + self.prefix + '_hand/set_median_filter')
                self.set_median_filter = rospy.ServiceProxy('/' + self.prefix + '_hand/set_median_filter', BHSetMedianFilter)
            resp1 = self.set_median_filter(samples)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def jointStatesCallback(self, data):
        joint_idx = 0
        for joint_name in data.name:
            self.js_pos[joint_name] = data.position[joint_idx]
            joint_idx += 1

#                if self.abort_on_q5_q6_self_collision and self.getQ5Q6SpaceSector(self.qar[5], self.qar[6], margin=10.0/180.0*math.pi) < 0 and not self.aborted_on_q5_q6_self_collision:    # self-collision
#                    try:
#                        self.aborted_on_q5_q6_self_collision = True
#                        self.action_right_cart_traj_client.cancel_goal()
#                    except:
#                        pass

    def setHeadLookAtPoint(self, pt):
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(pt)))

    def tactileCallback(self, data, hand_name):
#        max_tactile_value = -1.0
#        max_tactile_index = 0
#        max_tactile_finger = 0
#        fingers = [data.finger1_tip, data.finger2_tip, data.finger3_tip]
#        val_sum = 0.0
#        for f in range(0,3):
#            for i in range(0, 24):
#                if fingers[f][i] > max_tactile_value:
#                    max_tactile_value = fingers[f][i]
#                    max_tactile_index = i
#                    max_tactile_finger = f
        self.tactile_lock[hand_name].acquire()
        self.tactile_data_index[hand_name] += 1
        if self.tactile_data_index[hand_name] >= self.tactile_data_len:
            self.tactile_data_index[hand_name] = 0
        tact_buf_idx = self.tactile_data_index[hand_name]

#        self.max_tactile_value = copy.copy(max_tactile_value)
        # time, finger, max value, contact center
        self.tactile_data[hand_name][tact_buf_idx][0] = copy.copy(data.header.stamp)
        self.tactile_data[hand_name][tact_buf_idx][1] = copy.copy(data.finger1_tip)
        self.tactile_data[hand_name][tact_buf_idx][2] = copy.copy(data.finger2_tip)
        self.tactile_data[hand_name][tact_buf_idx][3] = copy.copy(data.finger3_tip)
        self.tactile_data[hand_name][tact_buf_idx][4] = copy.copy(data.palm_tip)
        self.tactile_lock[hand_name].release()

    def tactileCallbackRight(self, data):
        self.tactileCallback(data, "right")

    def tactileCallbackLeft(self, data):
        self.tactileCallback(data, "left")

    def getMaxWrench(self):
        wrench = Wrench()
        for i in range(0,self.wrench_tab_len):
            if abs(self.wrench_tab[i].force.x) > wrench.force.x:
                wrench.force.x = abs(self.wrench_tab[i].force.x)
            if abs(self.wrench_tab[i].force.y) > wrench.force.y:
                wrench.force.y = abs(self.wrench_tab[i].force.y)
            if abs(self.wrench_tab[i].force.z) > wrench.force.z:
                wrench.force.z = abs(self.wrench_tab[i].force.z)
            if abs(self.wrench_tab[i].torque.x) > wrench.torque.x:
                wrench.torque.x = abs(self.wrench_tab[i].torque.x)
            if abs(self.wrench_tab[i].torque.y) > wrench.torque.y:
                wrench.torque.y = abs(self.wrench_tab[i].torque.y)
            if abs(self.wrench_tab[i].torque.z) > wrench.torque.z:
                wrench.torque.z = abs(self.wrench_tab[i].torque.z)
        return wrench

    def wrenchCallback(self, wrench):
        self.wrench_tab[self.wrench_tab_index] = wrench
        self.wrench_tab_index += 1
        if self.wrench_tab_index >= self.wrench_tab_len:
            self.wrench_tab_index = 0
        wfx = abs(wrench.force.x)
        wfy = abs(wrench.force.y)
        wfz = abs(wrench.force.z)
        wtx = abs(wrench.torque.x)
        wty = abs(wrench.torque.y)
        wtz = abs(wrench.torque.z)
        if not self.joint_traj_active:
            if (wfx>self.current_max_wrench.force.x*2.0) or (wfy>self.current_max_wrench.force.y*2.0) or (wfz>self.current_max_wrench.force.z*2.0) or (wtx>self.current_max_wrench.torque.x*2.0) or (wty>self.current_max_wrench.torque.y*2.0) or (wtz>self.current_max_wrench.torque.z*2.0):
                self.wrench_emergency_stop = True

    def __init__(self):

        self.js_pos = {}

        self.joint_impedance_active = False
        self.cartesian_impedance_active = False
        self.joint_traj_active = False

        self.abort_on_q5_q6_self_collision = False
        self.aborted_on_q5_q6_self_collision = False

        self.T_B_L = [PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame()]

        # parameters
        self.prefix="right"
        self.k_error = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.T_B_W = None
        self.T_W_T = None #PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
        self.T_W_E = None
        self.T_E_W = None
        self.current_max_wrench = Wrench(Vector3(20, 20, 20), Vector3(20, 20, 20))
        self.wrench_emergency_stop = False
        self.exit_on_emergency_stop = True

        self.last_contact_time = rospy.Time.now()

        self.tactile_lock = {"left":Lock(), "right":Lock()}

        # for tactile sync
        self.tactile_data = {"left":[], "right":[]}
        self.tactile_data_len = 120
        self.tactile_data_index = {"left":0, "right":0}
        for gripper_name in self.tactile_data:
            for i in range(0, self.tactile_data_len):
                self.tactile_data[gripper_name].append( [rospy.Time.now(), [], [], [], []] )
                for j in range(24):
                    self.tactile_data[gripper_name][-1][1].append(0)
                    self.tactile_data[gripper_name][-1][2].append(0)
                    self.tactile_data[gripper_name][-1][3].append(0)
                    self.tactile_data[gripper_name][-1][4].append(0)

        # for score function
        self.failure_reason = "unknown"

        self.emergency_stop_active = False

        # cartesian wrist trajectory for right arm
        self.action_right_cart_traj_client = None

        # joint trajectory for right arm
        self.action_right_joint_traj_client = None

        # cartesian tool trajectory for right arm in wrist frame
        self.action_tool_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/tool_trajectory", CartesianTrajectoryAction)
        self.action_tool_client.wait_for_server()

        # cartesian impedance trajectory for right arm
        self.action_impedance_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_impedance", CartesianImpedanceAction)
        self.action_impedance_client.wait_for_server()

        self.action_move_hand_client = actionlib.SimpleActionClient("/right_hand/move_hand", BHMoveAction)
        self.action_move_hand_client.wait_for_server()

#        self.pub_wrench = rospy.Publisher("/"+self.prefix+"_arm/wrench_stamped", WrenchStamped)

        self.listener = tf.TransformListener();
        self.br = tf.TransformBroadcaster()

        rospy.sleep(1.0)
        
#        self.max_tactile_value = 0

        self.pressure_info = self.getPressureSensorsInfoClient()
        self.pressure_frames = []
        for i in range(0, 24):
            center = PyKDL.Vector(self.pressure_info.sensor[0].center[i].x, self.pressure_info.sensor[0].center[i].y, self.pressure_info.sensor[0].center[i].z)
            halfside1 = PyKDL.Vector(self.pressure_info.sensor[0].halfside1[i].x, self.pressure_info.sensor[0].halfside1[i].y, self.pressure_info.sensor[0].halfside1[i].z)
            halfside2 = PyKDL.Vector(self.pressure_info.sensor[0].halfside2[i].x, self.pressure_info.sensor[0].halfside2[i].y, self.pressure_info.sensor[0].halfside2[i].z)
            halfside1.Normalize()
            halfside2.Normalize()
            norm = halfside1*halfside2
            norm.Normalize()
            self.pressure_frames.append( PyKDL.Frame(PyKDL.Rotation(halfside1, halfside2, norm), center) )

        self.palm_pressure_frames = []
        for i in range(0, 24):
            center = PyKDL.Vector(self.pressure_info.sensor[3].center[i].x, self.pressure_info.sensor[3].center[i].y, self.pressure_info.sensor[3].center[i].z)
            halfside1 = PyKDL.Vector(self.pressure_info.sensor[3].halfside1[i].x, self.pressure_info.sensor[3].halfside1[i].y, self.pressure_info.sensor[3].halfside1[i].z)
            halfside2 = PyKDL.Vector(self.pressure_info.sensor[3].halfside2[i].x, self.pressure_info.sensor[3].halfside2[i].y, self.pressure_info.sensor[3].halfside2[i].z)
            halfside1.Normalize()
            halfside2.Normalize()
            norm = halfside1*halfside2
            norm.Normalize()
            self.palm_pressure_frames.append( PyKDL.Frame(PyKDL.Rotation(halfside1, halfside2, norm), center) )

        self.wrench_tab = []
        self.wrench_tab_index = 0
        self.wrench_tab_len = 4000
        for i in range(0,self.wrench_tab_len):
            self.wrench_tab.append( Wrench(Vector3(), Vector3()) )

        rospy.Subscriber('/right_hand/BHPressureState', BHPressureState, self.tactileCallbackRight)
        rospy.Subscriber('/left_hand/BHPressureState', BHPressureState, self.tactileCallbackLeft)
        rospy.Subscriber('/'+self.prefix+'_arm/wrench', Wrench, self.wrenchCallback)
        joint_states_listener = rospy.Subscriber('/joint_states', JointState, self.jointStatesCallback)

        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)

    def action_right_cart_traj_feedback_cb(self, feedback):
        self.action_right_cart_traj_feedback = copy.deepcopy(feedback)

    def waitForCartEnd(self):
        self.action_right_cart_traj_client.wait_for_result()
        return self.action_right_cart_traj_client.get_result(), self.action_right_cart_traj_feedback

    def moveWrist2(self, wrist_frame):
        wrist_pose = pm.toMsg(wrist_frame)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")

    def moveWrist(self, wrist_frame, t, max_wrench, start_time=0.01, stamp=None, path_tol=None):
        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
            print "FATAL ERROR: moveWrist"
            exit(0)

        self.joint_traj_active = False
        # we are moving the tool, so: T_B_Wd*T_W_T
        wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), "dest", "torso_base")
        wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)

        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)
        action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        action_trajectory_goal.wrench_constraint = max_wrench
        if path_tol != None:
            action_trajectory_goal.path_tolerance.position = geometry_msgs.msg.Vector3( path_tol[0].x(), path_tol[0].y(), path_tol[0].z() )
            action_trajectory_goal.path_tolerance.rotation = geometry_msgs.msg.Vector3( path_tol[1].x(), path_tol[1].y(), path_tol[1].z() )
        self.current_max_wrench = max_wrench
        self.action_right_cart_traj_client.send_goal(action_trajectory_goal, feedback_cb = self.action_right_cart_traj_feedback_cb)

    def moveWristTraj(self, wrist_frames, times, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
            print "FATAL ERROR: moveWristTraj"
            exit(0)
        self.abort_on_q5_q6_self_collision = abort_on_q5_q6_self_collision
        self.aborted_on_q5_q6_self_collision = False

        self.joint_traj_active = False

        # we are moving the tool, so: T_B_Wd*T_W_T
        action_trajectory_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_trajectory_goal.trajectory.header.stamp = stamp
        else:
            action_trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_time)

        i = 0
        for wrist_frame in wrist_frames:
            wrist_pose = pm.toMsg(wrist_frame*self.T_W_T)
            action_trajectory_goal.trajectory.points.append(CartesianTrajectoryPoint(
            rospy.Duration(times[i]),
            wrist_pose,
            Twist()))
            i += 1

        action_trajectory_goal.wrench_constraint = max_wrench
        self.current_max_wrench = max_wrench
        self.action_right_cart_traj_client.send_goal(action_trajectory_goal)

    def moveTool(self, wrist_frame, t, stamp=None):
        wrist_pose = pm.toMsg(wrist_frame)

        action_tool_goal = CartesianTrajectoryGoal()
        if stamp != None:
            action_tool_goal.trajectory.header.stamp = stamp
        else:
            action_tool_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.01)
#        action_tool_goal.trajectory.header.stamp = rospy.Time.now()
        action_tool_goal.trajectory.points.append(CartesianTrajectoryPoint(
        rospy.Duration(t),
        wrist_pose,
        Twist()))
        self.action_tool_client.send_goal(action_tool_goal)

    def moveImpedance(self, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            action_impedance_goal.trajectory.header.stamp = stamp
        else:
            action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
        rospy.Duration(t),
        CartesianImpedance(k,damping)))
        self.action_impedance_client.send_goal(action_impedance_goal)

    def moveImpedanceTraj(self, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        action_impedance_goal = CartesianImpedanceGoal()
        if stamp != None:
            action_impedance_goal.trajectory.header.stamp = stamp
        else:
            action_impedance_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        i = 0
        for k in k_n:
            action_impedance_goal.trajectory.points.append(CartesianImpedanceTrajectoryPoint(
            rospy.Duration(t_n[i]),
            CartesianImpedance(k,damping)))

        self.action_impedance_client.send_goal(action_impedance_goal)

    def moveWristJoint(self, q_dest, time, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveWristJoint"
            exit(0)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['torso_0_joint', 'torso_1_joint',
        'right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint',
        'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']

        move_joint_name_idx = {'right_arm_0_joint':0, 'right_arm_1_joint':1, 'right_arm_2_joint':2, 'right_arm_3_joint':3, 'right_arm_4_joint':4, 'right_arm_5_joint':5, 'right_arm_6_joint':6}
        q_dest_all = []
        for joint_name in goal.trajectory.joint_names:
            if joint_name in move_joint_name_idx:
                q = q_dest[move_joint_name_idx[joint_name]]
            else:
                q = self.js_pos[joint_name]
            q_dest_all.append(q)
        goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(time)))
        position_tol = 1.0/180.0 * math.pi
        velocity_tol = 1.0/180.0 * math.pi
        acceleration_tol = 1.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        self.joint_traj_active = True
        self.action_right_joint_traj_client.send_goal(goal)

    def moveWristTrajJoint(self, traj, time_mult, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveWristTrajJoint"
            exit(0)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['torso_0_joint', 'torso_1_joint',
        'right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint', 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint', 'right_arm_6_joint',
        'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint', 'left_arm_6_joint']

        move_joint_name_idx = {'right_arm_0_joint':0, 'right_arm_1_joint':1, 'right_arm_2_joint':2, 'right_arm_3_joint':3, 'right_arm_4_joint':4, 'right_arm_5_joint':5, 'right_arm_6_joint':6}

        pos = traj[0]
        vel = traj[1]
        acc = traj[2]
        dti = traj[3]
        time = 0.0
        for i in range(0, len(pos)):
            time += dti[i] * time_mult
            if vel != None:
                v = [0.0, 0.0, vel[i][0]/time_mult, vel[i][1]/time_mult, vel[i][2]/time_mult, vel[i][3]/time_mult, vel[i][4]/time_mult, vel[i][5]/time_mult, vel[i][6]/time_mult, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                v = []
            if acc != None:
                a = [0.0, 0.0] + acc[i] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                a = []

            q_dest_all = []
            for joint_name in goal.trajectory.joint_names:
                if joint_name in move_joint_name_idx:
                    q = pos[i][move_joint_name_idx[joint_name]]
                else:
                    q = self.js_pos[joint_name]
                q_dest_all.append(q)

            goal.trajectory.points.append(JointTrajectoryPoint(q_dest_all, v, a, [], rospy.Duration(time)))

        position_tol = 7.0/180.0 * math.pi
        velocity_tol = 7.0/180.0 * math.pi
        acceleration_tol = 7.0/180.0 * math.pi
        for joint_name in goal.trajectory.joint_names:
            goal.path_tolerance.append(JointTolerance(joint_name, position_tol, velocity_tol, acceleration_tol))
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.joint_traj_active = True
        self.action_right_joint_traj_client.send_goal(goal)

    def stopArm(self):
#        if self.action_right_cart_traj_client.gh:
#            self.action_right_cart_traj_client.cancel_all_goals()
#        if self.action_tool_client.gh:
#            self.action_tool_client.cancel_all_goals()

        try:
            self.action_right_cart_traj_client.cancel_goal()
        except:
            pass
        try:
            self.action_tool_client.cancel_goal()
        except:
            pass

    def emergencyStop(self):
        self.moveImpedance(self.k_error, 0.5)
        self.stopArm()
        self.emergency_stop_active = True
        print "emergency stop"

    def checkStopCondition(self, t=0.0):

        end_t = rospy.Time.now()+rospy.Duration(t+0.0001)
        while rospy.Time.now()<end_t:
            if rospy.is_shutdown():
                self.emergencyStop()
                print "emergency stop: interrupted  %s  %s"%(self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "user_interrupt"
                rospy.sleep(1.0)
            if self.wrench_emergency_stop:
                self.emergencyStop()
                print "too big wrench"
                self.failure_reason = "too_big_wrench"
                rospy.sleep(1.0)

            if (self.action_right_cart_traj_client != None) and (self.action_right_cart_traj_client.gh) and ((self.action_right_cart_traj_client.get_state()==GoalStatus.REJECTED) or (self.action_right_cart_traj_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_right_cart_traj_client.get_state()
                result = self.action_right_cart_traj_client.get_result()
                self.emergencyStop()
                print "emergency stop: traj_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_trajectory"
                rospy.sleep(1.0)

            if (self.action_tool_client.gh) and ((self.action_tool_client.get_state()==GoalStatus.REJECTED) or (self.action_tool_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_tool_client.get_state()
                result = self.action_tool_client.get_result()
                self.emergencyStop()
                print "emergency stop: tool_err: %s ; %s ; max_wrench: %s   %s"%(state, result, self.getMaxWrench(), self.wrench_tab_index)
                self.failure_reason = "too_big_wrench_tool"
                rospy.sleep(1.0)
            rospy.sleep(0.01)
        return self.emergency_stop_active

    # ex.
    # q = (120.0/180.0*numpy.pi, 120.0/180.0*numpy.pi, 40.0/180.0*numpy.pi, 180.0/180.0*numpy.pi)
    # robot.move_hand_client("right", q)
#    def move_hand_client(self, q, v=(1.2, 1.2, 1.2, 1.2), t=(2000.0, 2000.0, 2000.0, 2000.0)):
#        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
#        try:
#            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
#            resp1 = move_hand(q[0], q[1], q[2], q[3], v[0], v[1], v[2], v[3], t[0], t[1], t[2], t[3])
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

    def moveHand(self, q, v, t, maxPressure, hold=False):
        action_goal = BHMoveGoal()
        action_goal.name = ["right_HandFingerOneKnuckleTwoJoint", "right_HandFingerTwoKnuckleTwoJoint", "right_HandFingerThreeKnuckleTwoJoint", "right_HandFingerOneKnuckleOneJoint"]
        action_goal.q = q
        action_goal.v = v
        action_goal.t = t
        action_goal.maxPressure = maxPressure
        if hold == True:
            action_goal.hold = 1
        else:
            action_goal.hold = 0
        self.action_move_hand_client.send_goal(action_goal)

    def waitForHand(self):
        self.action_move_hand_client.wait_for_result()        

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def updateTransformations(self):
        pose = self.listener.lookupTransform('torso_base', 'torso_link2', rospy.Time(0))
        self.T_B_T2 = pm.fromTf(pose)
        self.T_T2_B = self.T_B_T2.Inverse()

        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
        self.T_B_W = pm.fromTf(pose)

        for i in range(0,7):
            pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_' + str(i+1) + '_link', rospy.Time(0))
            self.T_B_L[i] = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_2_link', rospy.Time(0))
#        self.T_B_L2 = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_5_link', rospy.Time(0))
#        self.T_B_L5 = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_6_link', rospy.Time(0))
#        self.T_B_L6 = pm.fromTf(pose)

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_7_link', rospy.Time(0))
#        self.T_B_L7 = pm.fromTf(pose)

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F = pm.fromTf(pose)
        self.T_F_E = self.T_E_F.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', rospy.Time(0))
        self.T_E_F13 = pm.fromTf(pose)
        self.T_F13_E = self.T_E_F13.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', rospy.Time(0))
        self.T_E_F23 = pm.fromTf(pose)
        self.T_F23_E = self.T_E_F23.Inverse()

        pose = self.listener.lookupTransform('/'+self.prefix+'_HandPalmLink', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', rospy.Time(0))
        self.T_E_F33 = pm.fromTf(pose)
        self.T_F33_E = self.T_E_F33.Inverse()

        if self.T_W_E == None:
            pose = self.listener.lookupTransform(self.prefix+'_arm_7_link', self.prefix+'_HandPalmLink', rospy.Time(0))
            self.T_W_E = pm.fromTf(pose)
            self.T_E_W = self.T_W_E.Inverse()

        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_cmd', rospy.Time(0))
        self.T_B_T_cmd = pm.fromTf(pose)

        pose = self.listener.lookupTransform('right_HandPalmLink', 'right_HandFingerOneKnuckleOneLink', rospy.Time(0))
        self.T_E_F11 = pm.fromTf(pose)

        pose = self.listener.lookupTransform('right_HandPalmLink', 'right_HandFingerTwoKnuckleOneLink', rospy.Time(0))
        self.T_E_F21 = pm.fromTf(pose)

        self.T_E_F31 = PyKDL.Frame()

#        pose = self.listener.lookupTransform('torso_base', 'camera', rospy.Time(0))
#        self.T_B_C = pm.fromTf(pose)

    def getContactPointsInFrame(self, threshold, frame_name, hand_prefix):
        self.tactile_lock[hand_prefix].acquire()
        latest_index = copy.copy(self.tactile_data_index[hand_prefix])
        self.tactile_lock[hand_prefix].release()

        tactile_frames_names = [
        '/'+hand_prefix+'_HandFingerOneKnuckleThreeLink',
        '/'+hand_prefix+'_HandFingerTwoKnuckleThreeLink',
        '/'+hand_prefix+'_HandFingerThreeKnuckleThreeLink',
        '/'+hand_prefix+'_HandPalmLink']
        contacts = []
        forces = []

        pressure_frames = [self.pressure_frames, self.pressure_frames, self.pressure_frames, self.palm_pressure_frames]
        for tact_idx in range(len(tactile_frames_names)):
            tact_name = tactile_frames_names[tact_idx]
            for buf_prev_idx in range(20, self.tactile_data_len-2):
                buf_idx = latest_index - buf_prev_idx
#                buf_idx_prev = buf_idx - 1
                if buf_idx < 0:
                    buf_idx += self.tactile_data_len
#                if buf_idx_prev < 0:
#                    buf_idx_prev += self.tactile_data_len

                time = self.tactile_data[hand_prefix][buf_idx][0]
                tactile_data = self.tactile_data[hand_prefix][buf_idx][tact_idx+1]

                if self.listener.canTransform('torso_base', tact_name, time) and self.listener.canTransform('torso_base', frame_name, time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', tact_name, time))
                    T_B_R = pm.fromTf(self.listener.lookupTransform('torso_base', frame_name, time))
                    T_R_B = T_B_R.Inverse()
                    for i in range(0, len(pressure_frames[tact_idx])):
#                        print "i"
                        neighbourhood_ok = True
                        # check the time neighbourhood
                        for buf_neigh in range(-19, 19):
                            buf_neigh_idx = buf_idx+buf_neigh
                            if buf_neigh_idx < 0:
                                buf_neigh_idx += self.tactile_data_len
                            elif buf_neigh_idx >= self.tactile_data_len:
                                buf_neigh_idx -= self.tactile_data_len
#                            print buf_neigh_idx
#                            print self.tactile_data[hand_prefix][0][1]
                            if self.tactile_data[hand_prefix][buf_neigh_idx][tact_idx+1][i] < threshold:
#                            if self.tactile_data[hand_prefix][0][1][0] < threshold:
                                neighbourhood_ok = False
                                break
                        if neighbourhood_ok:#tactile_data[i] > threshold:
#                            contacts.append( T_R_B * T_B_F * pressure_frames[tact_idx][i] * PyKDL.Vector() )
                            h1 = self.pressure_info.sensor[tact_idx].halfside1[i]
                            h2 = self.pressure_info.sensor[tact_idx].halfside2[i]
                            contacts.append( (T_R_B * T_B_F * pressure_frames[tact_idx][i], PyKDL.Vector(h1.x, h1.y, h1.z).Norm(), PyKDL.Vector(h2.x, h2.y, h2.z).Norm()) )
                    break

        return contacts

    # there is a lot of bugs in this function!
    def getContactPoints(self, threshold, f1=True, f2=True, f3=True, palm=True):
        self.tactile_lock["right"].acquire()
        index = copy.copy(self.tactile_data_index)
        max_value = copy.copy(self.max_tactile_value)
        self.tactile_lock["right"].release()

        contacts = []
        forces = []
        if f1:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerOneKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][1][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if f2:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerTwoKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][2][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if f3:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandFingerThreeKnuckleThreeLink', time))
                    for i in range(0, len(self.pressure_frames)):
                        if self.tactile_data[index][3][i] > threshold:
                            contacts.append( T_B_F * self.pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        if palm:
            for i in range(0, self.tactile_data_len-2):
                time = self.tactile_data[index][0]
                if self.listener.canTransform('torso_base', '/'+self.prefix+'_HandPalmLink', time):
                    T_B_F = pm.fromTf(self.listener.lookupTransform('torso_base', '/'+self.prefix+'_HandPalmLink', time))
                    for i in range(0, len(self.palm_pressure_frames)):
                        if self.tactile_data[index][4][i] > threshold:
                            contacts.append( T_B_F * self.palm_pressure_frames[i] * PyKDL.Vector() )
                            forces.append(self.tactile_data[index][4][i])
                    break
                index -= 1
                if index < 0:
                    index = copy.copy(self.tactile_data_len)-1

        return contacts, forces

    def handleEmergencyStop(self):
        if self.emergency_stop_active:
            ch = '_'
            while (ch != 'e') and (ch != 'n') and (ch != 'r'):
                ch = raw_input("Emergency stop active... (e)xit, (n)ext case, (r)epeat case: ")
            if ch == 'e':
                exit(0)
            if ch == 'n':
                self.action = "next"
                self.index += 1
            if ch == 'r':
                self.action = "repeat"
            self.updateTransformations()
            print "moving desired pose to current pose"
            self.emergency_stop_active = False
            self.moveWrist(self.T_B_W, 2.0, Wrench(Vector3(25,25,25), Vector3(5,5,5)))
            self.checkStopCondition(2.0)
            return True
        return False

    def updateAndMoveTool(self, tool, duration):
        self.T_W_T = copy.deepcopy(tool)    # tool transformation
        self.updateTransformations()

        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
        # move both tool position and wrist position - the gripper holds its position
        print "moving wrist"
        # we assume that during the initialization there are no contact forces, so we limit the wrench
        stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.moveWrist( self.T_B_W, duration, Wrench(Vector3(10, 10, 10), Vector3(2, 2, 2)), stamp=stamp)
#        print "moving tool"
        self.moveTool( self.T_W_T, duration, stamp=stamp )

    def updateAndMoveToolOnly(self, tool, duration):
        self.T_W_T = copy.deepcopy(tool)    # tool transformation
        self.updateTransformations()

        print "setting the tool to %s relative to wrist frame"%(self.T_W_T)
        # move both tool position and wrist position - the gripper holds its position
        print "moving tool"
        self.moveTool( self.T_W_T, duration )

    def waitForFirstContact(self, threshold, duration, emergency_stop=True, f1=True, f2=True, f3=True, palm=True):
        contacts = []
        contact_found = False
        end_t = rospy.Time.now()+rospy.Duration(duration)
        while rospy.Time.now()<end_t:
            if self.checkStopCondition(0.02):
                    break
            contacts = self.getContactPoints(threshold, f1, f2, f3, palm)
            if len(contacts) > 0:
                contact_found = True
                self.stopArm()
                break
        if emergency_stop and not contact_found:
            print "could not find contact point"
            self.emergencyStop()
            rospy.sleep(1.0)
        return contacts

    def getMovementTime(self, T_B_Wd, max_v_l = 0.1, max_v_r = 0.2):
        self.updateTransformations()
        twist = PyKDL.diff(self.T_B_W, T_B_Wd, 1.0)
        v_l = twist.vel.Norm()
        v_r = twist.rot.Norm()
#        print "v_l: %s   v_r: %s"%(v_l, v_r)
        f_v_l = v_l/max_v_l
        f_v_r = v_r/max_v_r
        if f_v_l > f_v_r:
            duration = f_v_l
        else:
            duration = f_v_r
        if duration < 0.2:
            duration = 0.2
        return duration

    def getMovementTime2(self, T_B_Wd1, T_B_Wd2, max_v_l = 0.1, max_v_r = 0.2):
        twist = PyKDL.diff(T_B_Wd1, T_B_Wd2, 1.0)
        v_l = twist.vel.Norm()
        v_r = twist.rot.Norm()
        print "v_l: %s   v_r: %s"%(v_l, v_r)
        f_v_l = v_l/max_v_l
        f_v_r = v_r/max_v_r
        if f_v_l > f_v_r:
            duration = f_v_l
        else:
            duration = f_v_r
        if duration < 0.5:
            duration = 0.5
        return duration

    def switchToJoint(self):
        self.cartesian_impedance_active = False
        result = False
        try:
            if not hasattr(self, 'conmanSwitch') or self.conmanSwitch == None:
                rospy.wait_for_service('/controller_manager/switch_controller')
                self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            # '2' is for STRICT
            if self.conmanSwitch(['JntImp', 'TrajectoryGeneratorJoint'], ['CImp', 'PoseIntLeft', 'PoseIntRight'], 2):
                # joint trajectory for right arm
                if self.action_right_joint_traj_client == None:
                    self.action_right_joint_traj_client = actionlib.SimpleActionClient('/spline_trajectory_action_joint', FollowJointTrajectoryAction)
                    self.action_right_joint_traj_client.wait_for_server()
                result = True
        except rospy.ROSInterruptException:
            print "rospy.ROSInterruptException"
        except IOError:
            print "IOError"
        except KeyError:
            print "KeyError"
        if not result:
            print "FATAL ERROR: switchToJoint"
            exit(0)
        self.joint_impedance_active = True
        return result

    def switchToCart(self):
        self.joint_impedance_active = False
        result = False
        try:
            if not hasattr(self, 'conmanSwitch') or self.conmanSwitch == None:
                rospy.wait_for_service('/controller_manager/switch_controller')
                self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            # '2' is for STRICT
            if self.conmanSwitch(['CImp', 'PoseIntLeft', 'PoseIntRight'], ['JntImp', 'TrajectoryGeneratorJoint'], 2):
                # cartesian wrist trajectory for right arm
                if self.action_right_cart_traj_client == None:
                    self.action_right_cart_traj_client = actionlib.SimpleActionClient("/" + self.prefix + "_arm/cartesian_trajectory", CartesianTrajectoryAction)
                    self.action_right_cart_traj_client.wait_for_server()
                result = True

        except rospy.ROSInterruptException:
            print "rospy.ROSInterruptException"
        except IOError:
            print "IOError"
        except KeyError:
            print "KeyError"
        if not result:
            print "FATAL ERROR: switchToCart"
            exit(0)
        self.cartesian_impedance_active = True
        return result

    def isJointImpedanceActive(self):
        if self.joint_impedance_active and not self.cartesian_impedance_active:
            return True
        return False

    def isCartesianImpedanceActive(self):
        if not self.joint_impedance_active and self.cartesian_impedance_active:
            return True
        return False


