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
from threading import Lock

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
from numpy import *
import numpy as np

import copy

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

import velmautils

import velma_fk_ik

# reference frames:
# B - robot's base
# W - wrist
# E - gripper
# F - finger distal link
# T - tool

class VelmaSim:
    """
Class for velma robot.
"""
    def calibrateTactileSensors(self):
        pass

    def resetFingers(self):
        # TODO
        pass

    def setMedianFilter(self, samples):
        pass

    def getQ5Q6SpaceSector(self, q5, q6, margin=0.0):
        i = 0
        # x1,x2,y1,y2
        for r in self.q5_q6_restricted_area:
            if q5 > r[0]+margin and q5 < r[1]-margin and q6 > r[2]+margin and q6 < r[3]-margin:
                return i
            i += 1
        return -1

    def getClosestQ5Q6SpaceSector(self, q5, q6):
        dist = []
        sect = self.getQ5Q6SpaceSector(q5, q6)
        if sect >= 0:
            return sect
        # x1,x2,y1,y2
        for r in self.q5_q6_restricted_area:
            d5 = 1000000.0
            d6 = 1000000.0
            if q5 < r[0]:
                d5 = r[0] - q5
            elif q5 > r[1]:
                d5 = q5 - r[1]
            if q6 < r[2]:
                d6 = r[2] - q6
            elif q6 > r[3]:
                d6 = q6 - r[3]
            dist.append( min( d5, d6 ) )

        i = 0
        min_dist = 1000000.0
        min_index = -1
        for d in dist:
            if d < min_dist:
                min_dist = d
                min_index = i
            i += 1
        return min_index

#1.92521262169
#-1.44746339321
#-0.428265035152
#-2.89507389069

#-2.16006875038
#-1.44746339321
#0.273337930441
#2.22040915489

#2.07619023323
#0.932657182217
#-0.819031119347
#2.86872577667

    def initRightQ5Q6SelfCollisionDetection(self):
        self.right_q5_q6_collision_polygon = [
        [2.12191557884,-2.90171504021],
        [1.90023589134,-1.52496743202],
        [0.465151429176,-1.25661826134],
        [0.319309473038,-0.218223929405],
        [1.30520093441,1.01851558685],
        [2.11024808884,1.01851534843],
        [2.12191557884,2.89695906639],
        [-0.812423944473,2.8736243248],
        [-1.2324488163,2.25525474548],
        [-2.18333816528,2.27858948708],
        [-2.22417378426,-1.30328762531],
        [-0.41573381424,-1.85165309906],
        [-0.386565506458,-2.89588117599],
        ]

        self.r1_r2_point = [0.00486671971157, -1.43977892399]
        self.r2_r3_point = [-0.202122956514, 1.60210323334]
        self.q5_q6_restricted_area = [
        [-0.428265035152,1.92521262169,-2.89507389069,-1.38213706017],
        [-2.11473441124,0.435783565044,-1.52231526375,2.22040915489],
        [-0.819031119347,2.07619023323,0.932657182217,2.86872577667],
        ]

    def tactileCallback(self, data):
        max_tactile_value = -1.0
        max_tactile_index = 0
        max_tactile_finger = 0
        fingers = [data.finger1_tip, data.finger2_tip, data.finger3_tip]
        val_sum = 0.0
        for f in range(0,3):
            for i in range(0, 24):
                if fingers[f][i] > max_tactile_value:
                    max_tactile_value = fingers[f][i]
                    max_tactile_index = i
                    max_tactile_finger = f
        self.tactile_lock.acquire()
        self.tactile_data_index += 1
        if self.tactile_data_index >= self.tactile_data_len:
            self.tactile_data_index = 0

        self.max_tactile_value = copy.copy(max_tactile_value)
        # time, finger, max value, contact center
        self.tactile_data[self.tactile_data_index][0] = copy.copy(data.header.stamp)
        self.tactile_data[self.tactile_data_index][1] = copy.copy(data.finger1_tip)
        self.tactile_data[self.tactile_data_index][2] = copy.copy(data.finger2_tip)
        self.tactile_data[self.tactile_data_index][3] = copy.copy(data.finger3_tip)
        self.tactile_data[self.tactile_data_index][4] = copy.copy(data.palm_tip)
        self.tactile_lock.release()

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
        if (wfx>self.current_max_wrench.force.x*2.0) or (wfy>self.current_max_wrench.force.y*2.0) or (wfz>self.current_max_wrench.force.z*2.0) or (wtx>self.current_max_wrench.torque.x*2.0) or (wty>self.current_max_wrench.torque.y*2.0) or (wtz>self.current_max_wrench.torque.z*2.0):
            self.wrench_emergency_stop = True

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
        self.pub_js = rospy.Publisher("/joint_states", JointState)
        self.robot = URDF.from_parameter_server()
        self.mimic_joints_map = {}
        self.joint_name_limit_map = {}
        self.joint_name_idx_map = {}
        self.js = JointState()

        joint_idx_ros = 0
        for i in range(len(self.robot.joints)):
            joint = self.robot.joints[i]
            
            if joint.joint_type == "fixed":
                continue
            if joint.mimic != None:
                self.mimic_joints_map[joint.name] = joint.mimic

            self.joint_name_limit_map[joint.name] = joint.limit
            self.joint_name_idx_map[joint.name] = joint_idx_ros

            self.js.name.append(joint.name)
            print joint.name
            self.js.position.append(0)
            joint_idx_ros += 1

        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)
        print self.js.name
        print self.joint_name_idx_map

    def publishJointStates(self):
        self.js.header.stamp = rospy.Time.now()
        self.pub_js.publish(self.js)

    def simSetJointPosition(self, joint_name, joint_position):
        self.js.position[self.joint_name_idx_map[joint_name]] = joint_position

    def setInitialJointPosition(self):
        self.simSetJointPosition("torso_1_joint", -80.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_0_joint", 120.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_1_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_2_joint", 110.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_3_joint", 100.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_4_joint", 20.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_5_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("right_arm_6_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_0_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_1_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_2_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_3_joint", -90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_4_joint", 0.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_5_joint", 90.0/180.0*math.pi)
        self.simSetJointPosition("left_arm_6_joint", -90.0/180.0*math.pi)
        self.updateJointLimits(self.js)
        self.updateMimicJoints(self.js)

    #
    # init
    #
    def __init__(self, openraveinstance, velma_ikr):

        self.velma_ikr = velma_ikr
        self.openrave = openraveinstance

        self.joint_impedance_active = False
        self.cartesian_impedance_active = False
        self.qar = [-90.0/180.0*math.pi, 90.0/180.0*math.pi, -90.0/180.0*math.pi, 90.0/180.0*math.pi, 0.0/180.0*math.pi, -90.0/180.0*math.pi, 0]
        self.qal = [-90.0/180.0*math.pi, -90.0/180.0*math.pi, 90.0/180.0*math.pi, -90.0/180.0*math.pi, 0, 90.0/180.0*math.pi, 0]
        self.qhr = [0.0, 0.0, 0.0, 0.0]
        self.qhl = [0.0, 0.0, 0.0, 0.0]
        self.qt = [0.0, -1.5707963267948966]

        self.abort_on_q5_singularity = False
        self.abort_on_q5_singularity_angle = 20.0/180.0*math.pi
        self.aborted_on_q5_singularity = False
        self.abort_on_q5_q6_self_collision = False
        self.aborted_on_q5_q6_self_collision = False

        self.T_B_L = [PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame(),PyKDL.Frame()]

#        self.initRightQ5Q6SelfCollisionDetection()

        # parameters
        self.prefix="right"
        self.T_B_W = None
        self.T_W_T = PyKDL.Frame(PyKDL.Vector(0.2,-0.05,0))    # tool transformation
        self.T_W_E = None
        self.T_E_W = None
        self.wrench_emergency_stop = False
        self.exit_on_emergency_stop = True
        self.k_error = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))

        self.last_contact_time = rospy.Time.now()

        self.tactile_lock = Lock()

        # for tactile sync
        self.tactile_data = []
        self.tactile_data_len = 40
        self.tactile_data_index = 0
        for i in range(0, self.tactile_data_len):
            self.tactile_data.append( [rospy.Time.now(), [], [], [], []] )

        # for score function
        self.failure_reason = "unknown"

        self.emergency_stop_active = False

        self.max_tactile_value = 0

        self.initJointStatePublisher()
        self.setInitialJointPosition()

        self.fk_solver = velma_fk_ik.VelmaFkIkSolver()
#


    def moveWrist2(self, T_B_W):
        pass

    def getCartImpWristTraj(self, js, goal_T_B_W):
        init_js = copy.deepcopy(js)
        init_T_B_E = self.fk_solver.calculateFk("right_HandPalmLink", init_js)
        T_B_Ed = goal_T_B_W * self.T_W_E
        T_B_E_diff = PyKDL.diff(init_T_B_E, T_B_Ed, 1.0)
        js_map = {}
        for j_idx in range(len(init_js.name)):
            js_map[init_js.name[j_idx]] = j_idx

        self.updateJointLimits(init_js)
        self.updateMimicJoints(init_js)
        q_list = []
        for f in np.linspace(0.0, 1.0, 50):
            T_B_Ei = PyKDL.addDelta(init_T_B_E, T_B_E_diff, f)
            q_out = self.fk_solver.simulateTrajectory("right_HandPalmLink", init_js, T_B_Ei)
            if q_out == None:
                return None
            q_list.append(q_out)

#            print "len(q_out) ", len(q_out)
            for i in range(7):
                joint_name = self.fk_solver.ik_joint_state_name["right_HandPalmLink"][i]
                init_js.position[ js_map[joint_name] ] = q_out[i]

        return q_list

    def moveWrist(self, goal_T_B_W, t, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        init_js = copy.deepcopy(self.js)
        init_T_B_E = self.fk_solver.calculateFk("right_HandPalmLink", init_js)
        T_B_Ed = goal_T_B_W * self.T_W_E
        T_B_E_diff = PyKDL.diff(init_T_B_E, T_B_Ed, 1.0)
        js_map = {}
        for j_idx in range(len(init_js.name)):
            js_map[init_js.name[j_idx]] = j_idx

        q_out_list = []
        for f in np.linspace(0.0, 1.0, 50):
            T_B_Ei = PyKDL.addDelta(init_T_B_E, T_B_E_diff, f)
            q_out = self.fk_solver.simulateTrajectory("right_HandPalmLink", init_js, T_B_Ei)
            if q_out == None :
                print "moveWrist: could not reach the desired pose"
                return
            q_out_list.append(q_out)

            for i in range(7):
                joint_name = self.fk_solver.ik_joint_state_name["right_HandPalmLink"][i]
                init_js.position[ js_map[joint_name] ] = q_out[i]

        for q in q_out_list:
            for i in range(7):
                joint_name = self.fk_solver.ik_joint_state_name["right_HandPalmLink"][i]
                self.simSetJointPosition(joint_name, q[i])
            rospy.sleep(t/50.0)

#    def moveWrist(self, T_B_W, t, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
#        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
#            print "FATAL ERROR: moveWrist"
#            exit(0)
#        T_B_W_current = self.openrave.getLinkPose("right_arm_7_link", qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        T_B_T2_current = self.openrave.getLinkPose("torso_link2", qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        T_T2_B_current = T_B_T2_current.Inverse()
#        qar = copy.deepcopy(self.qar)
#        for f in np.linspace(0.0, 1.0, 50):
#            q_out, T_B_Ei = self.velma_ikr.simulateTrajectory(T_B_W_current * self.T_W_E, T_B_W * self.T_W_E, f, qar, T_T2_B_current)
#            if q_out == None :
#                print "moveWrist: could not reach the desired pose"
#                break
#            self.qar = q_out
#            rospy.sleep(t/50.0)

    def moveWristTraj(self, wrist_frames, times, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (self.cartesian_impedance_active and not self.joint_impedance_active):
            print "FATAL ERROR: moveWristTraj"
            exit(0)
        i = 0
        for wrist_frame in wrist_frames:
            T_B_W_current = self.openrave.getLinkPose("right_arm_7_link", qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
            T_B_T2_current = self.openrave.getLinkPose("torso_link2", qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
            T_T2_B_current = T_B_T2_current.Inverse()
            if i > 0:
                time_diff = times[i] - times[i-1]
            else:
                time_diff = times[i]
            for f in np.linspace(0.0, 1.0, 10):
                q_out, T_B_Ei = self.velma_ikr.simulateTrajectory(T_B_W_current * self.T_W_E, wrist_frame * self.T_W_E, f, self.qar, T_T2_B_current)
                if q_out == None :
                    print "moveWristTraj: could not reach the desired pose"
                    return
#                self.dofs_lock.acquire()
                self.qar = q_out
#                self.dofs_lock.release()
                rospy.sleep(time_diff/10.0)
            i += 1

    def moveWristJoint(self, q_dest, time, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveWristJoint"
            exit(0)
        self.qar = q_dest
        rospy.sleep(time)

    def moveWristTrajJoint(self, traj, time_mult, max_wrench, start_time=0.01, stamp=None, abort_on_q5_singularity = False, abort_on_q5_q6_self_collision=False):
        if not (not self.cartesian_impedance_active and self.joint_impedance_active):
            print "FATAL ERROR: moveWristTrajJoint"
            exit(0)
        for i in range(0, len(traj[0])):
            rospy.sleep(traj[3][i]*time_mult)
            self.qar = traj[0][i]

    def moveTool(self, wrist_frame, t):
        pass

    def moveImpedance(self, k, t, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        pass

    def moveImpedanceTraj(self, k_n, t_n, stamp=None, damping=Wrench(Vector3(0.7, 0.7, 0.7),Vector3(0.7, 0.7, 0.7))):
        pass

    def stopArm(self):
        pass

    def emergencyStop(self):
        self.moveImpedance(self.k_error, 0.5)
        self.stopArm()
        self.emergency_stop_active = True
        print "emergency stop"

    def checkStopCondition(self, t=0.0):
        if rospy.is_shutdown():
            self.emergencyStop()
            print "emergency stop: interrupted  %s  %s"%(self.getMaxWrench(), self.wrench_tab_index)
            self.failure_reason = "user_interrupt"
            rospy.sleep(1.0)
        # TODO
        return
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
            if self.aborted_on_q5_singularity:
#                print "aborted_on_q5_singularity"
                break

            if (self.action_right_trajectory_client.gh) and ((self.action_right_trajectory_client.get_state()==GoalStatus.REJECTED) or (self.action_right_trajectory_client.get_state()==GoalStatus.ABORTED)):
                state = self.action_right_trajectory_client.get_state()
                result = self.action_right_trajectory_client.get_result()
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
    def move_hand_client(self, q, v=(1.2, 1.2, 1.2, 1.2), t=(2000.0, 2000.0, 2000.0, 2000.0)):
        steps = 100
        q_traj = [np.linspace(self.qhr[0], q[3], steps), np.linspace(self.qhr[1], q[0], steps), np.linspace(self.qhr[2], q[2], steps), np.linspace(self.qhr[3], q[1], steps)]
        for i in range(0, steps):
            self.qhr[0] = q_traj[0][i]
            self.qhr[1] = q_traj[1][i]
            self.qhr[2] = q_traj[2][i]
            self.qhr[3] = q_traj[3][i]
            rospy.sleep(0.02)
        # TODO
        return
        rospy.wait_for_service('/' + self.prefix + '_hand/move_hand')
        try:
            move_hand = rospy.ServiceProxy('/' + self.prefix + '_hand/move_hand', BHMoveHand)
            resp1 = move_hand(q[0], q[1], q[2], q[3], v[0], v[1], v[2], v[3], t[0], t[1], t[2], t[3])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def hasContact(self, threshold, print_on_false=False):
        if self.T_F_C != None:
            return True
        return False

    def updateTransformations(self):

#        self.T_B_T2 = self.openrave.getLinkPose('torso_link2', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
        self.T_B_T2 = self.fk_solver.calculateFk("torso_link2", self.js)
        self.T_T2_B = self.T_B_T2.Inverse()

#        self.T_B_W = self.openrave.getLinkPose(self.prefix+'_arm_7_link', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
        self.T_B_W = self.fk_solver.calculateFk(self.prefix+'_arm_7_link', self.js)
#        self.T_B_E = self.openrave.getLinkPose(self.prefix+'_HandPalmLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
        self.T_B_E = self.fk_solver.calculateFk(self.prefix+'_HandPalmLink', self.js)
        self.T_W_E = self.T_B_W.Inverse() * self.T_B_E
        self.T_E_W = self.T_W_E.Inverse()

#        for i in range(0,7):
#            self.T_B_L[i] = self.openrave.getLinkPose(self.prefix+'_arm_' + str(i+1) + '_link', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)

#        self.T_B_F = self.openrave.getLinkPose(self.prefix+'_HandFingerThreeKnuckleThreeLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F = self.T_B_E.Inverse() * self.T_B_F
#        self.T_F_E = self.T_E_F.Inverse()

#        self.T_B_F13 = self.openrave.getLinkPose(self.prefix+'_HandFingerOneKnuckleThreeLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F13 = self.T_B_E.Inverse() * self.T_B_F13
#        self.T_F13_E = self.T_E_F13.Inverse()

#        self.T_B_F23 = self.openrave.getLinkPose(self.prefix+'_HandFingerTwoKnuckleThreeLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F23 = self.T_B_E.Inverse() * self.T_B_F23
#        self.T_F23_E = self.T_E_F23.Inverse()

#        self.T_B_F33 = self.openrave.getLinkPose(self.prefix+'_HandFingerThreeKnuckleThreeLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F33 = self.T_B_E.Inverse() * self.T_B_F33
#        self.T_F33_E = self.T_E_F33.Inverse()

        # camera pose
        self.T_B_C = PyKDL.Frame(PyKDL.Rotation.RotY(135.0/180.0*math.pi), PyKDL.Vector(0.4, 0, 1.4)) * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi))

#        pose = self.listener.lookupTransform('torso_base', self.prefix+'_arm_cmd', rospy.Time(0))
#        self.T_B_T_cmd = pm.fromTf(pose)

#        self.T_B_F11 = self.openrave.getLinkPose(self.prefix+'_HandFingerOneKnuckleOneLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F11 = self.T_B_E.Inverse() * self.T_B_F11
#        self.T_F11_E = self.T_E_F11.Inverse()

#        self.T_B_F21 = self.openrave.getLinkPose(self.prefix+'_HandFingerTwoKnuckleOneLink', qt=self.qt, qar=self.qar, qal=self.qal, qhr=self.qhr, qhl=self.qhl)
#        self.T_E_F21 = self.T_B_E.Inverse() * self.T_B_F21
#        self.T_F21_E = self.T_E_F21.Inverse()

#        self.T_E_F31 = PyKDL.Frame()

    def getContactPoints(self, threshold, f1=True, f2=True, f3=True, palm=True):
        return [], []
        self.tactile_lock.acquire()
        index = copy.copy(self.tactile_data_index)
        max_value = copy.copy(self.max_tactile_value)
        self.tactile_lock.release()

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

    def calculateMoveGripperPointToPose(self, P_E_p, rot_B_E, pos_B_E_p):
        P_B_N = PyKDL.Frame( copy.deepcopy(rot_B_E) ) * P_E_p
        T_B_Ed = PyKDL.Frame( copy.deepcopy(rot_B_E), pos_B_E_p - P_B_N )
        T_B_Wd = T_B_Ed * self.T_E_W
        return T_B_Wd

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

    def updateAndMoveToolOnly(self, tool, duration):
        self.T_W_T = copy.deepcopy(tool)    # tool transformation
        self.updateTransformations()

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

    def getFingersKinematics(self):
        return

    def getClosestRotations(self, rot_set, rot):
        buf = [[100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0], [100000.0,0]]
        lbuf = len(buf)
        def putToBuffer(data):
            for i in range(0, lbuf):
                if data[0] < buf[lbuf-1-i][0]:
                    for j in range(0, lbuf-1-i):
                        buf[j] = buf[j+1]
                    buf[-1-i] = data
                    break
        i = 0
        for r in rot_set:
            twist = PyKDL.diff(PyKDL.Frame(r), PyKDL.Frame(rot), 1.0)
            twist_v = (twist.rot.x()*twist.rot.x() + twist.rot.y()*twist.rot.y() + twist.rot.z()*twist.rot.z())
            putToBuffer([twist_v,i])
            i += 1
        print "%s   %s   %s   %s   %s   %s   %s   %s   %s"%(buf[8][0], buf[7][0], buf[6][0], buf[5][0], buf[4][0], buf[3][0], buf[2][0], buf[1][0], buf[0][0])
#        print "%s"%(buf[2][0])
#        print "%s"%(buf[1][0])
#        print "%s"%(buf[0][0])
        return [buf[0][1], buf[1][1], buf[2][1]]

    def getClosestRotation(self, rot_set, rot):
        min_twist = 10000.0
        i = 0
        index = 0
        for r in rot_set:
            twist = PyKDL.diff(PyKDL.Frame(r), PyKDL.Frame(rot), 1.0)
            twist_v = (twist.rot.x()*twist.rot.x() + twist.rot.y()*twist.rot.y() + twist.rot.z()*twist.rot.z())
            if min_twist > twist_v:
                min_twist = twist_v
                index = i
            i += 1
        return [min_twist, index]

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
        if duration < 0.02:
            duration = 0.02
        return duration

    def generateTrajectoryInJoint(self, i, rel_angle, om):
        if rel_angle > 0.0:
            omega = math.fabs(om)
        elif rel_angle < 0.0:
            omega = -math.fabs(om)
        else:
            return None

        self.updateTransformations()
        T_Li_L7 = self.T_B_L[i].Inverse() * self.T_B_L[6]

        time_d = 0.01
        stop = False
        angle = 0.0
        times = []
        time = 0.5
        tab_T_B_Wd = []
        while not stop:
            angle += omega * time_d
            time += time_d
            if rel_angle > 0.0 and angle > rel_angle:
                angle = rel_angle
                stop = True
            if rel_angle < 0.0 and angle < rel_angle:
                angle = rel_angle
                stop = True
            if i == 0:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotZ(angle))
            elif i == 1:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotY(angle))
            elif i == 2:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotZ(angle))
            elif i == 3:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotY(-angle))
            elif i == 4:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotZ(angle))
            elif i == 5:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotY(angle))
            elif i == 6:
                T_Li_Lid = PyKDL.Frame(PyKDL.Rotation.RotZ(angle))
            T_B_Wd = self.T_B_L[i] * T_Li_Lid * T_Li_L7
            tab_T_B_Wd.append(T_B_Wd)
            times.append(time)
        return [tab_T_B_Wd, times]

    def moveAwayQ5Singularity(self, omega, T_B_Wd=None):
        self.updateTransformations()
        sector0 = self.getQ5Q6SpaceSector(self.q_r[5], self.q_r[6])
        sector0 = self.getQ5Q6SpaceSector(self.q_r[5], self.q_r[6])
        angle = 25.0/180.0*math.pi
        if T_B_Wd == None:
            if self.q_r[5] > 0.0:
                angle = 25.0/180.0*math.pi
            else:
                angle = -25.0/180.0*math.pi
            traj, times = self.generateTrajectoryInJoint(5, -self.q_r[5]+angle, omega)
        else:
            traj1, times1 = self.generateTrajectoryInJoint(5, -self.q_r[5]+angle, omega)
            traj2, times2 = self.generateTrajectoryInJoint(5, -self.q_r[5]-angle, omega)
            diff1 = PyKDL.diff(T_B_Wd,traj1[-1])
            diff2 = PyKDL.diff(T_B_Wd,traj2[-1])
            if diff1.rot.Norm() < diff2.rot.Norm():
                traj = traj1
                times = times1
            else:
                traj = traj2
                times = times2

        self.moveWrist2(traj[-1]*self.T_W_T)
        raw_input("Press Enter to move the robot in " + str(times[-1]) + " s...")
        if self.checkStopCondition():
            exit(0)
        self.moveWristTraj(traj, times, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=False)
        self.checkStopCondition(times[-1]+0.5)

    def generateTrajectoryInQ5Q6(self, q5, q6, omega):
        actual_q5 = copy.copy(self.q_r[5])
        actual_q6 = copy.copy(self.q_r[6])

        rel_q5 = q5 - actual_q5
        rel_q6 = q6 - actual_q6
        if math.fabs(rel_q5) > math.fabs(rel_q6):
            omega_q5 = math.fabs(omega)
            omega_q6 = math.fabs(omega)*math.fabs(rel_q6)/math.fabs(rel_q5)
        else:
            omega_q6 = math.fabs(omega)
            omega_q5 = math.fabs(omega)*math.fabs(rel_q5)/math.fabs(rel_q6)

        if rel_q5 < 0.0:
            omega_q5 = -omega_q5
        if rel_q6 < 0.0:
            omega_q6 = -omega_q6

        self.updateTransformations()
        T_L5_L6 = self.T_B_L[5].Inverse() * self.T_B_L[6]

        time_d = 0.01
        stop = False
        angle_q5 = 0.0
        angle_q6 = 0.0
        times = []
        time = 0.5
        tab_T_B_Wd = []
        while not stop:
            angle_q5 += omega_q5 * time_d
            angle_q6 += omega_q6 * time_d
            time += time_d
            if rel_q5 > 0.0 and angle_q5 > rel_q5:
                angle_q5 = rel_q5
                stop = True
            if rel_q5 < 0.0 and angle_q5 < rel_q5:
                angle_q5 = rel_q5
                stop = True

            if rel_q6 > 0.0 and angle_q6 > rel_q6:
                angle_q6 = rel_q6
                stop = True
            if rel_q6 < 0.0 and angle_q6 < rel_q6:
                angle_q6 = rel_q6
                stop = True

            T_L5_L5d = PyKDL.Frame(PyKDL.Rotation.RotY(angle_q5))
            T_L6_L6d = PyKDL.Frame(PyKDL.Rotation.RotZ(angle_q6))
            T_B_Wd = self.T_B_L[5] * T_L5_L5d * T_L5_L6 * T_L6_L6d
            tab_T_B_Wd.append(T_B_Wd)
            times.append(time)
        return [tab_T_B_Wd, times]

    def getAllDOFs(self):
        dofs = self.qt + self.qal + self.qhl + self.qar + self.qhr
        return dofs

    def switchToJoint(self):
        self.joint_impedance_active = True
        self.cartesian_impedance_active = False

    def switchToCart(self):
        self.joint_impedance_active = False
        self.cartesian_impedance_active = True

    def isJointImpedanceActive(self):
        if self.joint_impedance_active and not self.cartesian_impedance_active:
            return True
        return False

    def isCartesianImpedanceActive(self):
        if not self.joint_impedance_active and self.cartesian_impedance_active:
            return True
        return False


