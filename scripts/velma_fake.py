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
roslib.load_manifest('velma_common')

import rospy

import sensor_msgs.msg
import geometry_msgs.msg
import actionlib
import actionlib_msgs.msg
import cartesian_trajectory_msgs.msg
import barrett_hand_controller_msgs.msg
import controller_manager_msgs.srv
import std_srvs.srv
import control_msgs.msg

import velma_fk_ik
import headkinematics

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

import copy

import urdf_parser_py.urdf

class MoveHandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = barrett_hand_controller_msgs.msg.BHMoveFeedback()
    _result   = barrett_hand_controller_msgs.msg.BHMoveResult()

    def __init__(self, name, robot_state):
        self.robot_state = robot_state
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, barrett_hand_controller_msgs.msg.BHMoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        while True:
            js_pos = self.robot_state.getJsPos()

            allIdle = True
            for dof_idx in range(len(goal.q)):
                joint_name = goal.name[dof_idx]
                diff = goal.q[dof_idx] - js_pos[joint_name]
                if abs(diff) > 0.0001:
                    allIdle = False
                if abs(diff) > abs(goal.v[dof_idx] * 0.01):
                    if diff > 0:
                        diff = goal.v[dof_idx] * 0.01
                    else:
                        diff = -goal.v[dof_idx] * 0.01
                self.robot_state.js.position[ self.robot_state.joint_name_idx_map[joint_name] ] += diff

            self.robot_state.updateJointLimits(self.robot_state.js)
            self.robot_state.updateMimicJoints(self.robot_state.js)

            if allIdle:
                break
            rospy.sleep(0.01)

        self._result.error_code = 0
        self._as.set_succeeded(self._result)

class MoveImpAction(object):
    # create messages that are used to publish feedback/result
    _feedback = cartesian_trajectory_msgs.msg.CartesianImpedanceFeedback()
    _result   = cartesian_trajectory_msgs.msg.CartesianImpedanceResult()

    def __init__(self, name, robot_state):
        self.robot_state = robot_state
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cartesian_trajectory_msgs.msg.CartesianImpedanceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        success = True

        self._result.error_code = 0
        self._as.set_succeeded(self._result)

        return

class MoveToolAction(object):
    # create messages that are used to publish feedback/result
    _feedback = cartesian_trajectory_msgs.msg.CartesianTrajectoryFeedback()
    _result   = cartesian_trajectory_msgs.msg.CartesianTrajectoryResult()

    def __init__(self, name, robot_state, wrist_name):
        self.robot_state = robot_state
        self.wrist_name = wrist_name
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cartesian_trajectory_msgs.msg.CartesianTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        success = True

        print "MoveCartesianTrajectory ", self._action_name, " points: ", len(goal.trajectory.points)

        init_T_B_W = self.robot_state.fk_solver.calculateFk(self.wrist_name, self.robot_state.getJsPos())
        init_T_W_T = self.robot_state.T_W_T[self.wrist_name]

        current_dest_point_idx = 0
        while True:

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                return

            time_now = rospy.Time.now()
            time_from_start = (time_now - goal.trajectory.header.stamp).to_sec()
            if time_from_start <= 0:
                rospy.sleep(0.01)
                continue
            while time_from_start > goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec():
                current_dest_point_idx += 1
                if current_dest_point_idx >= len(goal.trajectory.points):
                    break

            if current_dest_point_idx >= len(goal.trajectory.points):
                break

            dest_time = goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec()
            dest_T_W_T = pm.fromMsg(goal.trajectory.points[current_dest_point_idx].pose)

            if current_dest_point_idx > 0:
                prev_time = goal.trajectory.points[current_dest_point_idx-1].time_from_start.to_sec()
                prev_T_W_T = pm.fromMsg(goal.trajectory.points[current_dest_point_idx-1].pose)
            else:
                prev_time = 0.0
                prev_T_W_T = init_T_W_T

            f = (time_from_start-prev_time) / (dest_time - prev_time)
            if f < 0 or f > 1:
                print "error: MoveCartesianTrajectory f: ", str(f)

            diff_T_W_T = PyKDL.diff(prev_T_W_T, dest_T_W_T, 1.0)

            T_W_Ti = PyKDL.addDelta(prev_T_W_T, diff_T_W_T, f)

            T_B_Tcmd = self.robot_state.arm_cmd[self.wrist_name]
            T_B_Wi = T_B_Tcmd * T_W_Ti.Inverse()

            self.robot_state.T_W_T[self.wrist_name] = T_W_Ti

            if not self.robot_state.joint_impedance_active:
                q_out = self.robot_state.fk_solver.simulateTrajectory(self.wrist_name, self.robot_state.getJsPos(), T_B_Wi)
                if q_out == None:
                    self._result.error_code = -3    # PATH_TOLERANCE_VIOLATED
                    self._as.set_aborted(self._result)
                    return

                for i in range(7):
                    joint_name = self.robot_state.fk_solver.ik_joint_state_name[self.wrist_name][i]
                    self.robot_state.js.position[ self.robot_state.joint_name_idx_map[joint_name] ] = q_out[i]

            # publish the feedback
            self._feedback.header.stamp = time_now
            self._feedback.desired = pm.toMsg(T_W_Ti)
            self._feedback.actual = pm.toMsg(T_W_Ti)
            self._feedback.error =  pm.toMsg(PyKDL.Frame())
            self._as.publish_feedback(self._feedback)

            rospy.sleep(0.01)

        self.robot_state.T_W_T[self.wrist_name] = dest_T_W_T

        self._result.error_code = 0
        self._as.set_succeeded(self._result)

class MoveJointTrajectory(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result   = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name, robot_state):
        self.robot_state = robot_state
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        success = True

        print "MoveJointTrajectory ", self._action_name, " points: ", len(goal.trajectory.points)

        if not self.robot_state.joint_impedance_active:
            self._result.error_code = 0
            self._as.set_succeeded(self._result)
            print "ERROR MoveJointTrajectory wrong state"

        # verify the trajectory
        if len(goal.trajectory.points) == 0:
            self._result.error_code = -1    # INVALID_GOAL
            self._as.set_aborted(self._result)
            return

        init_pos = []
        for joint_name in goal.trajectory.joint_names:
            joint_idx = self.robot_state.joint_name_idx_map[joint_name]
            init_pos.append(self.robot_state.js.position[joint_idx])

        current_dest_point_idx = 0
        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                return

            time_now = rospy.Time.now()
            time_from_start = (time_now - goal.trajectory.header.stamp).to_sec()
            if time_from_start <= 0:
                rospy.sleep(0.001)
                continue
            while time_from_start > goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec():
                current_dest_point_idx += 1
                if current_dest_point_idx >= len(goal.trajectory.points):
                    break

            if current_dest_point_idx >= len(goal.trajectory.points):
                break

            dest_time = goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec()
            dest_pos = goal.trajectory.points[current_dest_point_idx].positions

            if current_dest_point_idx > 0:
                prev_time = goal.trajectory.points[current_dest_point_idx-1].time_from_start.to_sec()
                prev_pos = goal.trajectory.points[current_dest_point_idx-1].positions
            else:
                prev_time = 0.0
                prev_pos = init_pos

            f = (time_from_start-prev_time) / (dest_time - prev_time)
            if f < 0 or f > 1:
                print "error: MoveJointTrajectory f: ", str(f)

            idx = 0
            for joint_name in goal.trajectory.joint_names:
                self.robot_state.js.position[ self.robot_state.joint_name_idx_map[joint_name] ] = prev_pos[idx] * (1.0-f) + dest_pos[idx] * f
                idx += 1
                self.robot_state.updateMimicJoints(self.robot_state.js)

            # publish the feedback
            self._feedback.header.stamp = time_now
#            self._feedback.desired = pm.toMsg(T_B_Ti)
#            self._feedback.actual = pm.toMsg(T_B_Ti)
#            self._feedback.error =  pm.toMsg(PyKDL.Frame())
            self._as.publish_feedback(self._feedback)

            rospy.sleep(0.001)

        self._result.error_code = 0
        self._as.set_succeeded(self._result)


class MoveCartesianTrajectory(object):
    # create messages that are used to publish feedback/result
    _feedback = cartesian_trajectory_msgs.msg.CartesianTrajectoryFeedback()
    _result   = cartesian_trajectory_msgs.msg.CartesianTrajectoryResult()

    def __init__(self, name, robot_state, wrist_name):
        self.robot_state = robot_state
        self.wrist_name = wrist_name
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, cartesian_trajectory_msgs.msg.CartesianTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        success = True

        if self.robot_state.joint_impedance_active:
            self._result.error_code = 0
            self._as.set_succeeded(self._result)

        print "MoveCartesianTrajectory ", self._action_name, " points: ", len(goal.trajectory.points)

        init_T_B_W = self.robot_state.fk_solver.calculateFk(self.wrist_name, self.robot_state.getJsPos())
        init_T_B_T = init_T_B_W * self.robot_state.T_W_T[self.wrist_name]

        current_dest_point_idx = 0
        while True:

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                return

            time_now = rospy.Time.now()
            time_from_start = (time_now - goal.trajectory.header.stamp).to_sec()
            if time_from_start <= 0:
                rospy.sleep(0.01)
                continue
            while time_from_start > goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec():
                current_dest_point_idx += 1
                if current_dest_point_idx >= len(goal.trajectory.points):
                    break

            if current_dest_point_idx >= len(goal.trajectory.points):
                break

            dest_time = goal.trajectory.points[current_dest_point_idx].time_from_start.to_sec()
            dest_T_B_T = pm.fromMsg(goal.trajectory.points[current_dest_point_idx].pose)

            if current_dest_point_idx > 0:
                prev_time = goal.trajectory.points[current_dest_point_idx-1].time_from_start.to_sec()
                prev_T_B_T = pm.fromMsg(goal.trajectory.points[current_dest_point_idx-1].pose)
            else:
                prev_time = 0.0
                prev_T_B_T = init_T_B_T

            f = (time_from_start-prev_time) / (dest_time - prev_time)
            if f < 0 or f > 1:
                print "error: MoveCartesianTrajectory f: ", str(f)

            diff_T_B_T = PyKDL.diff(prev_T_B_T, dest_T_B_T, 1.0)

            T_B_Ti = PyKDL.addDelta(prev_T_B_T, diff_T_B_T, f)

            T_B_Wi = T_B_Ti * self.robot_state.T_W_T[self.wrist_name].Inverse()
            q_out = self.robot_state.fk_solver.simulateTrajectory(self.wrist_name, self.robot_state.getJsPos(), T_B_Wi)
            if q_out == None:
                self._result.error_code = -3    # PATH_TOLERANCE_VIOLATED
                self._as.set_aborted(self._result)
                return

            for i in range(7):
                joint_name = self.robot_state.fk_solver.ik_joint_state_name[self.wrist_name][i]
                self.robot_state.js.position[ self.robot_state.joint_name_idx_map[joint_name] ] = q_out[i]

            self.robot_state.arm_cmd[self.wrist_name] = T_B_Ti

            # publish the feedback
            self._feedback.header.stamp = time_now
            self._feedback.desired = pm.toMsg(T_B_Ti)
            self._feedback.actual = pm.toMsg(T_B_Ti)
            self._feedback.error =  pm.toMsg(PyKDL.Frame())
            self._as.publish_feedback(self._feedback)

            rospy.sleep(0.01)

        self._result.error_code = 0
        self._as.set_succeeded(self._result)

class VelmaFake:

    #
    # joint state
    #
    def updateMimicJoints(self, js):
        position_map = {}
        for i in range(len(js.name)):
            position_map[js.name[i]] = js.position[i]

        for i in range(len(js.name)):
            joint_name = js.name[i]
            if joint_name in self.mimic_joints_map:
                js.position[i] = position_map[self.mimic_joints_map[joint_name].joint] * self.mimic_joints_map[joint_name].multiplier
                if self.mimic_joints_map[joint_name].offset != None:
                    js.position[i] += self.mimic_joints_map[joint_name].offset

    def updateJointLimits(self, js):
        for i in range(len(js.name)):
            joint_name = js.name[i]
            if joint_name in self.mimic_joints_map:
                continue
            if js.position[i] < self.joint_name_limit_map[joint_name].lower:
                js.position[i] = self.joint_name_limit_map[joint_name].lower
            elif js.position[i] > self.joint_name_limit_map[joint_name].upper:
                js.position[i] = self.joint_name_limit_map[joint_name].upper

    def initJointStatePublisher(self):
        self.pub_js = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState, queue_size=100)
        self.robot = urdf_parser_py.urdf.URDF.from_parameter_server()
        self.mimic_joints_map = {}
        self.joint_name_limit_map = {}
        self.joint_name_idx_map = {}
        self.js = sensor_msgs.msg.JointState()

        joint_idx_ros = 0
        for i in range(len(self.robot.joints)):
            joint = self.robot.joints[i]
#            print joint.name

            if joint.joint_type == "fixed":
                continue
            if joint.mimic != None:
                self.mimic_joints_map[joint.name] = joint.mimic

            self.joint_name_limit_map[joint.name] = joint.limit
            self.joint_name_idx_map[joint.name] = joint_idx_ros

            self.js.name.append(joint.name)
            self.js.position.append(0)
            joint_idx_ros += 1

#        print "self.js.name"
#        print self.js.name
        # the URDF does not contain mimic joint information for the grippers
        # the following code adds mimic joints for grippers
        hand_mimic = [
        ["right_HandFingerThreeKnuckleThreeJoint", "right_HandFingerThreeKnuckleTwoJoint", 0.33333333, 0.0],
        ["right_HandFingerOneKnuckleThreeJoint", "right_HandFingerOneKnuckleTwoJoint", 0.33333333, 0.0],
        ["right_HandFingerTwoKnuckleOneJoint", "right_HandFingerOneKnuckleOneJoint", 1.0, 0.0],
        ["right_HandFingerTwoKnuckleThreeJoint", "right_HandFingerTwoKnuckleTwoJoint", 0.33333333, 0.0],
        ["left_HandFingerThreeKnuckleThreeJoint", "left_HandFingerThreeKnuckleTwoJoint", 0.33333333, 0.0],
        ["left_HandFingerOneKnuckleThreeJoint", "left_HandFingerOneKnuckleTwoJoint", 0.33333333, 0.0],
        ["left_HandFingerTwoKnuckleOneJoint", "left_HandFingerOneKnuckleOneJoint", 1.0, 0.0],
        ["left_HandFingerTwoKnuckleThreeJoint", "left_HandFingerTwoKnuckleTwoJoint", 0.33333333, 0.0]
        ]

        for mimic in hand_mimic:
            if not mimic[0] in self.mimic_joints_map:
                self.mimic_joints_map[mimic[0]] = urdf_parser_py.urdf.JointMimic()
                self.mimic_joints_map[mimic[0]].joint = mimic[1]
                self.mimic_joints_map[mimic[0]].multiplier = mimic[2]
                self.mimic_joints_map[mimic[0]].offset = mimic[3]

        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)

    def publishJointStates(self):
        self.js.header.stamp = rospy.Time.now()
        self.pub_js.publish(self.js)

    def simSetJointPosition(self, joint_name, joint_position):
        self.js.position[self.joint_name_idx_map[joint_name]] = joint_position

    def setInitialJointPosition(self):
#        self.simSetJointPosition("torso_1_joint", -80.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_0_joint", -20.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_1_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_2_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_3_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_4_joint", 0.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_5_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_6_joint", 0.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_0_joint", 20.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_1_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_2_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_3_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_4_joint", 0.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_5_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_6_joint", 0.0/180.0*math.pi)
        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)

    def getJsPos(self):
        js_pos = {}
        j_idx = 0
        for joint_name in self.js.name:
            js_pos[joint_name] = self.js.position[j_idx]
            j_idx += 1
        return js_pos

    def initTactilePublisher(self):
        self.pub_tact = {
        "left":rospy.Publisher('/left_hand/BHPressureState', barrett_hand_controller_msgs.msg.BHPressureState, queue_size=100),
        "right":rospy.Publisher('/right_hand/BHPressureState', barrett_hand_controller_msgs.msg.BHPressureState, queue_size=100)}
        self.tact = {"left":barrett_hand_controller_msgs.msg.BHPressureState(), "right":barrett_hand_controller_msgs.msg.BHPressureState()}

        for gripper_name in self.tact:
            for i in range(24):
                self.tact[gripper_name].finger1_tip.append(0)
                self.tact[gripper_name].finger2_tip.append(0)
                self.tact[gripper_name].finger3_tip.append(0)
                self.tact[gripper_name].palm_tip.append(0)

    def publishTactile(self):
        self.pub_tact["left"].publish(self.tact["left"])
        self.pub_tact["right"].publish(self.tact["right"])

    def initTactileInfoPublisher(self):
        self.pub_tact_info = {
        "left":rospy.Publisher('/left_hand/tactile_info_out', barrett_hand_controller_msgs.msg.BHPressureInfo, queue_size=100),
        "right":rospy.Publisher('/right_hand/tactile_info_out', barrett_hand_controller_msgs.msg.BHPressureInfo, queue_size=100)}
        self.tact_info = {"left":barrett_hand_controller_msgs.msg.BHPressureInfo(), "right":barrett_hand_controller_msgs.msg.BHPressureInfo()}

        pc, p1, p2, fc, f1, f2 = self.getTactileGeometry()
        sensor_center = [fc, fc, fc, pc]
        sensor_h1 = [f1, f1, f1, p1]
        sensor_h2 = [f2, f2, f2, p2]

        for gripper_name in self.tact_info:
            for sensor_id in range(4):
                elem = barrett_hand_controller_msgs.msg.BHPressureInfoElement()
#               TODO:
#                elem.frame_id = ""
                for i in range(24):
                    elem.center.append( geometry_msgs.msg.Vector3( sensor_center[sensor_id][i][0]*0.001, sensor_center[sensor_id][i][1]*0.001, sensor_center[sensor_id][i][2]*0.001 ) )
                    elem.halfside1.append( geometry_msgs.msg.Vector3( sensor_h1[sensor_id][i][0]*0.001, sensor_h1[sensor_id][i][1]*0.001, sensor_h1[sensor_id][i][2]*0.001 ) )
                    elem.halfside2.append( geometry_msgs.msg.Vector3( sensor_h2[sensor_id][i][0]*0.001, sensor_h2[sensor_id][i][1]*0.001, sensor_h2[sensor_id][i][2]*0.001 ) )
                    elem.force_per_unit.append( 1.0/256.0 )
                self.tact_info[gripper_name].sensor.append(elem)

    def publishTactileInfo(self):
        self.pub_tact_info["left"].publish(self.tact_info["left"])
        self.pub_tact_info["right"].publish(self.tact_info["right"])
    
    def headLookAtCallback(self, msg):
        T_B_Pla = pm.fromMsg(msg)
        self.head_look_at = T_B_Pla.p

    def rightHandResetCallback(self, msg):
        self.js.position[ self.joint_name_idx_map["right_HandFingerOneKnuckleOneJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["right_HandFingerOneKnuckleTwoJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["right_HandFingerTwoKnuckleTwoJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["right_HandFingerThreeKnuckleTwoJoint"] ] = 0.0
        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)

    def leftHandResetCallback(self, msg):
        self.js.position[ self.joint_name_idx_map["left_HandFingerOneKnuckleOneJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["left_HandFingerOneKnuckleTwoJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["left_HandFingerTwoKnuckleTwoJoint"] ] = 0.0
        self.js.position[ self.joint_name_idx_map["left_HandFingerThreeKnuckleTwoJoint"] ] = 0.0
        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)

    def __init__(self):
        self.initTactilePublisher()
        self.initTactileInfoPublisher()
        self.initJointStatePublisher()
        self.setInitialJointPosition()
        self.fk_solver = velma_fk_ik.VelmaFkIkSolver([], None)
        self.arm_cmd = {}
        self.arm_cmd["right_arm_7_link"] = self.fk_solver.calculateFk("right_arm_7_link", self.getJsPos())
        self.arm_cmd["left_arm_7_link"] = self.fk_solver.calculateFk("left_arm_7_link", self.getJsPos())
        self.T_W_T = {}
        self.T_W_T["right_arm_7_link"] = PyKDL.Frame()
        self.T_W_T["left_arm_7_link"] = PyKDL.Frame()
        self.joint_impedance_active = False

        self.head_look_at = PyKDL.Vector(1,0,1.8)

        v_rot = 0.800
        v_lean = 0.375
        v_head = 0.392
        h_cam = 0.0
        v_cam = 0.225
        self.head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)

        rospy.Subscriber('/head_lookat_pose', geometry_msgs.msg.Pose, self.headLookAtCallback)
        rospy.Subscriber('/right_hand/reset_fingers', std_msgs.msg.Empty, self.rightHandResetCallback)
        rospy.Subscriber('/left_hand/reset_fingers', std_msgs.msg.Empty, self.leftHandResetCallback)

#stream("Hand.calibrate_tactile_sensors", ros.comm.topic("calibrate_tactile_sensors"))
#stream("Hand.set_median_filter", ros.comm.topic("set_median_filter"))

    # conman switch fake service callback
    def handle_switch_controller(self, req):
        if len(req.start_controllers) == 2 and 'JntImp' in req.start_controllers and 'TrajectoryGeneratorJoint' in req.start_controllers and len(req.stop_controllers) == 3 and 'CImp' in req.stop_controllers and 'PoseIntLeft' in req.stop_controllers and 'PoseIntRight' in req.stop_controllers:
            self.joint_impedance_active = True
            print "switched to joint impedance"
            return controller_manager_msgs.srv.SwitchControllerResponse(True)
        elif len(req.stop_controllers) == 2 and 'JntImp' in req.stop_controllers and 'TrajectoryGeneratorJoint' in req.stop_controllers and len(req.start_controllers) == 3 and 'CImp' in req.start_controllers and 'PoseIntLeft' in req.start_controllers and 'PoseIntRight' in req.start_controllers:
            self.joint_impedance_active = False
            print "switched to cartesian impedance"
            return controller_manager_msgs.srv.SwitchControllerResponse(True)

        print "ERROR: handle_switch_controller: ", req.start_controllers, req.stop_controllers, req.strictness

        return controller_manager_msgs.srv.SwitchControllerResponse(False)

    def getTactileGeometry(self):
        palm_sensor_center = [
        [ 22, 15.9, 77.5 ], [ 11, 15.9, 77.5 ], [ 0, 15.9, 77.5 ], [ -11, 15.9, 77.5 ], [ -22, 15.9, 77.5 ],
        [ 33, 5.3, 77.5 ], [ 22, 5.3, 77.5 ], [ 11, 5.3, 77.5 ], [ 0, 5.3, 77.5 ], [ -11, 5.3, 77.5 ], [ -22, 5.3, 77.5 ], [ -33, 5.3, 77.5 ],
        [ 33, -5.3, 77.5 ], [ 22, -5.3, 77.5 ], [ 11, -5.3, 77.5 ], [ 0, -5.3, 77.5 ], [ -11, -5.3, 77.5 ], [ -22, -5.3, 77.5 ], [ -33, -5.3, 77.5 ],
        [ 22, -15.9, 77.5 ], [ 11, -15.9, 77.5 ], [ 0, -15.9, 77.5 ], [ -11, -15.9, 77.5 ], [ -22, -15.9, 77.5 ] ]
        palm_sensor_halfside1 = [
        [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ],
        [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ],
        [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ],
        [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ], [ 5, 0, 0 ] ]
        palm_sensor_halfside2 = [
        [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ],
        [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ],
        [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ],
        [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ], [ 0, 4.85, 0 ] ]
        finger_sensor_center = [
        [ 22.25, -9.5, 5.2 ], [22.25, -9.5, 0 ], [ 22.25, -9.5, -5.2 ],
        [ 28.25, -9.5, 5.2 ], [ 28.25, -9.5, 0 ], [ 28.25, -9.5, -5.2 ],
        [ 34.2484, -9.41371, 5.2 ], [ 34.2484, -9.41371, 0 ], [ 34.2484, -9.41371, -5.2 ],
        [ 40.2349, -9.05695, 5.2 ], [ 40.2349, -9.05695, 0 ], [ 40.2349, -9.05695, -5.2 ],
        [ 46.1912, -8.35887, 5.2 ], [ 46.1912, -8.35887, 0 ], [ 46.1912, -8.35887, -5.2 ],
        [ 51.0813, -7.1884, 5.2 ], [ 51.0813, -7.1884, 0 ], [ 51.0813, -7.1884, -5.2 ],
        [ 53.8108, -5.14222, 5.2 ], [ 53.8108, -5.14222, 0 ], [ 53.8108, -5.14222, -5.2 ],
        [ 55.4163, -2.13234, 5.2 ], [ 55.4163, -2.13234, 0 ], [ 55.4163, -2.13234, -5.2 ] ]
        finger_sensor_halfside1 = [
        [ 2.75, 0, 0 ], [ 2.75, 0, 0 ], [ 2.75, 0, 0 ],
        [ 2.75, 0, 0 ], [ 2.75, 0, 0 ], [ 2.75, 0, 0 ],
        [ 2.74837, 0.085096, 0 ], [ 2.74837, 0.085096, 0 ], [ 2.74837, 0.085096, 0 ],
        [ 2.73902, 0.241919, 0 ], [ 2.73902, 0.241919, 0 ], [ 2.73902, 0.241919, 0 ],
        [ 2.72073, 0.397956, 0 ], [ 2.72073, 0.397956, 0 ], [ 2.72073, 0.397956, 0 ],
        [ 1.35885, 0.614231, 0 ], [ 1.35885, 0.614231, 0 ], [ 1.35885, 0.614231, 0 ],
        [ 0.970635, 1.13209, 0 ], [ 0.970635, 1.13209, 0 ], [ 0.970635, 1.13209, 0 ],
        [ 0.399575, 1.4367, 0 ], [ 0.399575, 1.4367, 0 ], [ 0.399575, 1.4367, 0 ] ]
        finger_sensor_halfside2 = [
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ],
        [ 0, 0, 2.35 ], [ 0, 0, 2.35 ], [ 0, 0, 2.35 ] ]
        return palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2

    def sendToolTransform(self, wrist_name):
        q = self.T_W_T[wrist_name+"_arm_7_link"].M.GetQuaternion()
        p = self.T_W_T[wrist_name+"_arm_7_link"].p
        self.br.sendTransform([p[0], p[1], p[2]], [q[0], q[1], q[2], q[3]], rospy.Time.now(), wrist_name+"_arm_tool", wrist_name+"_arm_7_link")

    def spin(self):

        self.br = tf.TransformBroadcaster()

        # conman switch fake service
        rospy.Service('/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController, self.handle_switch_controller)

        move_effector_right = MoveCartesianTrajectory("/right_arm/cartesian_trajectory", self, "right_arm_7_link")
        move_tool_right = MoveToolAction("/right_arm/tool_trajectory", self,  "right_arm_7_link")
        move_imp_right = MoveImpAction("/right_arm/cartesian_impedance", self)

        move_effector_left = MoveCartesianTrajectory("/left_arm/cartesian_trajectory", self, "left_arm_7_link")
        move_tool_left = MoveToolAction("/left_arm/tool_trajectory", self,  "left_arm_7_link")
        move_imp_left = MoveImpAction("/left_arm/cartesian_impedance", self)

        move_joint = MoveJointTrajectory("/spline_trajectory_action_joint", self)

        move_hand = MoveHandAction("/right_hand/move_hand", self)
        move_hand = MoveHandAction("/left_hand/move_hand", self)

        time_diff = 0.05
        print "fake Velma interface is running"
        while not rospy.is_shutdown():
            self.publishJointStates()
            right_arm_cmd = pm.toMsg(self.arm_cmd["right_arm_7_link"])
            left_arm_cmd = pm.toMsg(self.arm_cmd["left_arm_7_link"])
            self.br.sendTransform([right_arm_cmd.position.x, right_arm_cmd.position.y, right_arm_cmd.position.z], [right_arm_cmd.orientation.x, right_arm_cmd.orientation.y, right_arm_cmd.orientation.z, right_arm_cmd.orientation.w], rospy.Time.now(), "right_arm_cmd", "torso_base")
            self.br.sendTransform([left_arm_cmd.position.x, left_arm_cmd.position.y, left_arm_cmd.position.z], [left_arm_cmd.orientation.x, left_arm_cmd.orientation.y, left_arm_cmd.orientation.z, left_arm_cmd.orientation.w], rospy.Time.now(), "left_arm_cmd", "torso_base")

            self.head_kin.UpdateTargetPosition(self.head_look_at.x(), self.head_look_at.y(), self.head_look_at.z())
            self.head_kin.TransformTargetToHeadFrame()
            joint_pan, joint_tilt = self.head_kin.CalculateHeadPose()
            current_head_pan = self.getJsPos()["head_pan_joint"]
            current_head_tilt = self.getJsPos()["head_tilt_joint"]
            head_pan_diff = joint_pan - current_head_pan
            head_tilt_diff = joint_tilt - current_head_tilt
            if head_pan_diff > 45.0/180.0*math.pi*time_diff:
                head_pan_diff = 45.0/180.0*math.pi*time_diff
            elif head_pan_diff < -45.0/180.0*math.pi*time_diff:
                head_pan_diff = -45.0/180.0*math.pi*time_diff
            if head_tilt_diff > 45.0/180.0*math.pi*time_diff:
                head_tilt_diff = 45.0/180.0*math.pi*time_diff
            elif head_tilt_diff < -45.0/180.0*math.pi*time_diff:
                head_tilt_diff = -45.0/180.0*math.pi*time_diff
            self.simSetJointPosition("head_pan_joint", current_head_pan + head_pan_diff)
            self.simSetJointPosition("head_tilt_joint", current_head_tilt + head_tilt_diff)

            self.sendToolTransform("right")
            self.sendToolTransform("left")

            self.publishTactile()
            self.publishTactileInfo()

            rospy.sleep(time_diff)

if __name__ == '__main__':
    rospy.init_node('velma_fake')
    v = VelmaFake()
    v.spin()

