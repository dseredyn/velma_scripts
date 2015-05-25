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
roslib.load_manifest('velma_scripts')

import rospy
import tf

import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
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
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma import Velma
import random
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import openraveinstance
import itertools
import operator
import rospkg
from scipy import optimize
import scipy

def KDLToOpenrave(T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

def OpenraveToKDL(T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

class GraspingTask:
    """
Class for grasp learning.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();

    def posesCollectorThread(self, poses_list):
        while not rospy.is_shutdown() and self.collect_poses:
            self.velma.updateTransformations()
            poses_list.append( self.velma.T_B_W * self.velma.T_W_T )
            rospy.sleep(0.1)

    def wristFollowerThread(self, poses_list):
        while not rospy.is_shutdown() and self.follow_wrist_pose:
            self.velma.updateTransformations()
            T_B_Wd = self.velma.T_B_W
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            if time < 1.0:
                time = 1.0
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)))
            rospy.sleep(time+0.1)

    def estCommonPoint(self, T_B_T_list):
        def f_2(tool_end_estimate):
            tool_end_T = PyKDL.Vector(tool_end_estimate[0], tool_end_estimate[1], tool_end_estimate[2])
            tool_end_B_list = []
            mean_pt_B = PyKDL.Vector()
            for T_B_T in T_B_T_list:
                tool_end_B = T_B_T * tool_end_T
                tool_end_B_list.append( tool_end_B )
                mean_pt_B += tool_end_B
            mean_pt_B = mean_pt_B/len(T_B_T_list)
            dist_list = []
            for tool_end_B in tool_end_B_list:
                dist_list.append( (tool_end_B - mean_pt_B).Norm() )
            return dist_list
        tool_end_estimate = 0.0, 0.0, 0.0
        tool_end_estimate_ret, ier = optimize.leastsq(f_2, tool_end_estimate, maxfev = 1000)
        return PyKDL.Vector(tool_end_estimate_ret[0], tool_end_estimate_ret[1], tool_end_estimate_ret[2]), sum( f_2(tool_end_estimate_ret) )

    def estCommonPointTest(self):
        key_endpoint_T = PyKDL.Vector(0,0,0.2)

        central_point_B = PyKDL.Vector(1,0,1)
        
        T_B_T_1 = PyKDL.Frame(PyKDL.Rotation.RotX(30.0/180.0*math.pi), central_point_B) * PyKDL.Frame(-key_endpoint_T)
        T_B_T_2 = PyKDL.Frame(PyKDL.Rotation.RotY(30.0/180.0*math.pi), central_point_B) * PyKDL.Frame(-key_endpoint_T)
        T_B_T_3 = PyKDL.Frame(PyKDL.Rotation.RotX(0.0/180.0*math.pi), central_point_B) * PyKDL.Frame(-key_endpoint_T)

        print self.estCommonPoint([T_B_T_1, T_B_T_2, T_B_T_3])

    def generateSpiral(self, max_radius, min_dist):
        points = [(0,0)]
        rounds = max_radius / min_dist
        for angle in np.arange(0.0, rounds*math.pi*2.0, 1.0/180.0*math.pi):
            rx = math.cos(angle)
            ry = math.sin(angle)
            r = min_dist * angle/math.pi/2.0
            nx = r * rx 
            ny = r * ry
            dist = math.sqrt( (points[-1][0]-nx)*(points[-1][0]-nx) + (points[-1][1]-ny)*(points[-1][1]-ny) )
            if dist > min_dist:
                points.append( (nx, ny) )
        return points

    def spin(self):
        simulation_only = False

        key_endpoint_T = PyKDL.Vector(0,0,0.2)
        T_B_P = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(1,0,1))

        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        # key and grasp parameters
        self.T_E_O = PyKDL.Frame()
        self.key_axis_O = PyKDL.Vector(0,0,1)
        self.key_up_O = PyKDL.Vector(1,0,0)
        self.key_side_O = self.key_axis_O * self.key_up_O

        # change the tool - the safe way
        print "switching to joint impedance..."
        if not self.velma.switchToJoint():
            print "ERROR: switchToJoint"
            exit(0)

        rospy.sleep(0.5)

        print "updating tool..."
        self.velma.updateTransformations()
        self.velma.updateAndMoveToolOnly(PyKDL.Frame(self.velma.T_W_E.p+PyKDL.Vector(0.1,0,0)), 0.1)
        rospy.sleep(0.5)
        print "done."

        print "switching to cartesian impedance..."
        if not self.velma.switchToCart():
            print "ERROR: switchToCart"
            exit(0)

        rospy.sleep(0.5)

        raw_input("Press ENTER to continue...")

        # set gripper configuration
        if False:
            q_grasp = [1.4141768883517725, 1.4179811607057289, 1.4377081536379384, 0]
            q_pre = 10.0/180.0*math.pi
            hv = [1.2, 1.2, 1.2, 1.2]
            ht = [3000, 3000, 3000, 3000]
            self.velma.moveHand([0.0, 0.0, 0.0, 0.0/180.0*math.pi], hv, ht, 5000, True)
            rospy.sleep(3)
            self.velma.moveHand([q_grasp[0]-q_pre, q_grasp[1]-q_pre, q_grasp[2]-q_pre, q_grasp[3]], hv, ht, 5000, True)
            rospy.sleep(5)
            self.velma.moveHand([q_grasp[0]+q_pre, q_grasp[1]+q_pre, q_grasp[2]+q_pre, q_grasp[3]], hv, ht, 5000, True)
            rospy.sleep(3)

        if False:
            # start with very low stiffness
            print "setting stiffness to very low value"
            k_low = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
            self.velma.moveImpedance(k_low, 0.5)
            if self.velma.checkStopCondition(0.5):
                exit(0)
            print "done."

            print "identifying the parameters of tool in..."
            wait_time = 20
            for i in range(wait_time):
                print "%s s"%(wait_time-i)
                rospy.sleep(1.0)

            print "collecting points..."

            self.collect_poses = True
            T_B_T_list = []
            thread.start_new_thread(self.posesCollectorThread, (T_B_T_list,))
            collect_time = 10
            for i in range(collect_time):
                print "%s s"%(collect_time-i)
                rospy.sleep(1.0)
            self.collect_poses = False
            rospy.sleep(0.5)

            key_endpoint_T, error = self.estCommonPoint(T_B_T_list)
            print key_endpoint_T, error
            self.key_endpoint_O = self.T_E_O.Inverse() * self.velma.T_W_E.Inverse() * self.velma.T_W_T * key_endpoint_T
            print "self.key_endpoint_O = PyKDL.Vector(%s, %s, %s)"%(self.key_endpoint_O[0], self.key_endpoint_O[1], self.key_endpoint_O[2])
            print "done."
        else:
#            self.key_endpoint_O = PyKDL.Vector(-0.00248664992634, -6.7348683886e-05, 0.232163117525)
            self.key_endpoint_O = PyKDL.Vector(0.000256401261281, -0.000625166847342, 0.232297442735)
 
        k_high = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(150, 150, 150))
        max_force = 10.0
        max_torque = 10.0
        path_tol = (PyKDL.Vector(max_force/k_high.force.x, max_force/k_high.force.y, max_force/k_high.force.z), PyKDL.Vector(max_torque/k_high.torque.x, max_torque/k_high.torque.y, max_torque/k_high.torque.z))
        max_force2 = 20.0
        max_torque2 = 20.0
        path_tol2 = (PyKDL.Vector(max_force2/k_high.force.x, max_force2/k_high.force.y, max_force2/k_high.force.z), PyKDL.Vector(max_torque2/k_high.torque.x, max_torque2/k_high.torque.y, max_torque2/k_high.torque.z))
        path_tol3 = (PyKDL.Vector(max_force/k_high.force.x, max_force/k_high.force.y, max_force2/k_high.force.z), PyKDL.Vector(max_torque/k_high.torque.x, max_torque/k_high.torque.y, max_torque/k_high.torque.z))
        print "path tolerance:", path_tol

#        k_hole_in = Wrench(Vector3(1000.0, 500.0, 500.0), Vector3(200, 200, 200))
        k_hole_in = Wrench(Vector3(100.0, 1000.0, 1000.0), Vector3(50, 5, 5))
        path_tol_in = (PyKDL.Vector(max_force2/k_hole_in.force.x, max_force2/k_hole_in.force.y, max_force2/k_hole_in.force.z), PyKDL.Vector(max_torque2/k_hole_in.torque.x, max_torque2/k_hole_in.torque.y, max_torque2/k_hole_in.torque.z))

        if False:
            k_increase = [
            Wrench(Vector3(10.0, 10.0, 10.0), Vector3(5, 5, 5)),
            Wrench(Vector3(50.0, 50.0, 50.0), Vector3(25, 25, 25)),
            Wrench(Vector3(250.0, 250.0, 250.0), Vector3(100, 100, 100))
            ]

            for k in k_increase:
                raw_input("Press Enter to set bigger stiffness...")
                self.velma.updateTransformations()
                T_B_Wd = self.velma.T_B_W
                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                print "moving the wrist to the current pose in " + str(time) + " s..."
                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)))
                rospy.sleep(time)
                print "done."
                print "setting stiffness to bigger value"
                self.velma.moveImpedance(k, 1.0)
                if self.velma.checkStopCondition(1.1):
                    exit(0)
                print "done."

            print "move the grasped key near the key hole..."

            self.follow_wrist_pose = True
            thread.start_new_thread(self.wristFollowerThread, (None,))
            raw_input("Press ENTER to save the pose...")
            self.follow_wrist_pose = False
            rospy.sleep(0.5)

            self.velma.updateTransformations()
            self.T_B_O_nearHole = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O

            p = self.T_B_O_nearHole.p
            q = self.T_B_O_nearHole.M.GetQuaternion()
            print "self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s, %s, %s, %s), PyKDL.Vector(%s, %s, %s))"%(q[0], q[1], q[2], q[3], p[0], p[1], p[2])

            T_B_Wd = self.velma.T_B_W
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            print "moving the wrist to the current pose in " + str(time) + " s..."
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)))
            status, feedback = self.velma.waitForCartEnd()

            print "setting stiffness to", k_high
            self.velma.moveImpedance(k_high, 1.0)
            if self.velma.checkStopCondition(1.0):
                exit(0)
            print "done."

        else:
#            self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.697525674378, -0.00510212356955, 0.715527762697, 0.0381041038336), PyKDL.Vector(0.528142123375, 0.0071159410235, 1.31300679435))
#            self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.699716675653, -0.0454110538496, 0.709529999372, 0.0700113561626), PyKDL.Vector(0.546491646893, -0.101297899801, 1.30959887442))
            self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.71891504857, -0.0529880479354, 0.691118088949, 0.0520500417212), PyKDL.Vector(0.883081316461, -0.100813768303, 0.95381559114))

        k_increase = [
        Wrench(Vector3(10.0, 10.0, 10.0), Vector3(5, 5, 5)),
        Wrench(Vector3(50.0, 50.0, 50.0), Vector3(25, 25, 25)),
        Wrench(Vector3(250.0, 250.0, 250.0), Vector3(100, 100, 100)),
        k_high
        ]

        for k in k_increase:
            raw_input("Press Enter to set bigger stiffness...")
            self.velma.updateTransformations()
            T_B_Wd = self.velma.T_B_W
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            print "moving the wrist to the current pose in " + str(time) + " s..."
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)))
            rospy.sleep(time)
            print "done."
            print "setting stiffness to bigger value"
            self.velma.moveImpedance(k, 1.0)
            if self.velma.checkStopCondition(1.1):
                exit(0)
            print "done."

        if True:
            print "probing the door..."
            search_radius = 0.02
            min_dist = 0.005
            probed_points = []
            contact_points_B = []

            # move to the center point and push the lock
            x = 0
            y = 0
            T_O_Od = PyKDL.Frame(self.key_up_O*x + self.key_side_O*y)
            T_O_Od2 = PyKDL.Frame(self.key_up_O*x + self.key_side_O*y + self.key_axis_O*0.1)

            self.velma.updateTransformations()
            T_B_Wd = self.T_B_O_nearHole * T_O_Od * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
            status, feedback = self.velma.waitForCartEnd()
            print "status", status
            if status.error_code != 0:
                print "unexpected contact", feedback
                exit(0)

            self.velma.updateTransformations()
            T_B_Wd = self.T_B_O_nearHole * T_O_Od2 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol)
            status, feedback = self.velma.waitForCartEnd()
            print "status", status
            if status.error_code != 0:
                self.velma.updateTransformations()
                contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
                contact_points_B.append(contact_B)
                m_id = self.pub_marker.publishSinglePointMarker(contact_B, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                print feedback
            else:
                print "no contact"
                exit(0)

            T_O_Od3 = PyKDL.Frame(self.key_axis_O * 5.0/k_high.force.z)
            self.velma.updateTransformations()
            T_B_Wref = self.velma.T_B_W
            T_B_Wd = T_B_Wref * self.velma.T_W_E * self.T_E_O * T_O_Od3 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
            status, feedback = self.velma.waitForCartEnd()
            print "status", status
            if status.error_code != 0:
                print "unexpected high contact force", feedback
                exit(0)

            desired_push_dist = 5.0/k_high.force.z

            self.velma.updateTransformations()
            contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
            contact_points_B.append(contact_B)

            # move along the spiral
            hole_found = False
            spiral = self.generateSpiral(search_radius, min_dist)
            for pt in spiral:
                pt_desired_O_in_ref = self.key_endpoint_O
                pt_contact_O_in_ref = self.T_E_O.Inverse() * self.velma.T_W_E.Inverse() * T_B_Wref.Inverse() * contact_B
                key_end_depth = PyKDL.dot(pt_contact_O_in_ref-pt_desired_O_in_ref, self.key_axis_O)
#                print "key_end_depth", key_end_depth
                T_O_Od4 = PyKDL.Frame(self.key_up_O*pt[0] + self.key_side_O*pt[1] + self.key_axis_O * (5.0/k_high.force.z + key_end_depth))
                pt_desired_B = T_B_Wref * self.velma.T_W_E * self.T_E_O * T_O_Od4 * self.key_endpoint_O
                m_id = self.pub_marker.publishSinglePointMarker(pt_desired_B, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

                T_B_Wd = T_B_Wref * self.velma.T_W_E * self.T_E_O * T_O_Od4 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol)
                status, feedback = self.velma.waitForCartEnd()
                print "status", status
                self.velma.updateTransformations()
                contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
                contact_points_B.append(contact_B)
                m_id = self.pub_marker.publishSinglePointMarker(contact_B, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

                # check if the key is in the hole
                if status.error_code != 0:
                    self.velma.updateTransformations()
                    T_O_Od5 = PyKDL.Frame(self.key_axis_O * 5.0/k_high.force.z)
                    T_B_Wd = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * T_O_Od5 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                    time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                    self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
                    status, feedback = self.velma.waitForCartEnd()
                    if status.error_code != 0:
                        print "unexpected high contact force", feedback
                        exit(0)

                    self.velma.updateTransformations()
                    T_O_Od6 = PyKDL.Frame(self.key_up_O*0.02 + self.key_axis_O * 5.0/k_high.force.z)
                    T_B_Wd = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * T_O_Od6 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                    time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                    self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol)
                    status, feedback = self.velma.waitForCartEnd()
                    if status.error_code != 0:
                        print "key is in a hole"
                        hole_found = True
                        break

            print "hole_found", hole_found

            T_O_Od3 = PyKDL.Frame(self.key_axis_O * 5.0/k_high.force.z)
            self.velma.updateTransformations()
            T_B_Wref = self.velma.T_B_W
            print "moving the wrist to the current pose to reduce tension..."
            T_B_Wd = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * T_O_Od3 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
            time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
            raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
            self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
            status, feedback = self.velma.waitForCartEnd()
            print "status", status
            if status.error_code != 0:
                print "unexpected high contact force", feedback
                exit(0)

            print "setting stiffness to bigger value"
            self.velma.moveImpedance(k_hole_in, 1.0)
            if self.velma.checkStopCondition(1.1):
                exit(0)
            print "done."

            self.velma.updateTransformations()

            # calculate the fixed lock frame T_B_L
            # T_B_L the same orientation as the gripper (z axis pointing towards the door)
            # and the origin at the key endpoint at the initial penetration
            T_B_L = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * PyKDL.Frame(self.key_endpoint_O)
            penetration_axis_L = PyKDL.Vector(0, 0, 1)

            # get current contact point
            contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
            contact_L = T_B_L.Inverse() * contact_B
            contact_max_depth_L = PyKDL.dot(contact_L, penetration_axis_L)
            contact_depth_L = contact_max_depth_L
            prev_contact_depth_L = contact_depth_L
            deepest_contact_L = copy.deepcopy(contact_L)
            m_id = self.pub_marker.publishSinglePointMarker(contact_B, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

            central_point_L = PyKDL.Vector()

            force_key_axis = 17.0
            force_key_up = 10.0
            force_key_side = 10.0

            spiral_hole = self.generateSpiral(0.05, 0.005)

            xi = 7
            yi = 7
            explore_mode = "get_new"
            while True:
                # desired position of the key end
                self.velma.updateTransformations()

                T_B_Ocurrent = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * PyKDL.Frame(self.key_axis_O * (force_key_axis/k_hole_in.force.x))
                diff_B = PyKDL.diff(T_B_Ocurrent, self.T_B_O_nearHole)
                rot_diff_Ocurrent = PyKDL.Frame(T_B_Ocurrent.Inverse().M) * diff_B.rot


                T_B_W_shake = []
                for pt in spiral_hole:
                    T_B_Od_shake = T_B_Ocurrent * PyKDL.Frame(PyKDL.Rotation.RotZ(rot_diff_Ocurrent.z()-6.0/180.0*math.pi), self.key_up_O * pt[0] + self.key_side_O * pt[1])
                    T_B_W_shake.append(T_B_Od_shake * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse())
                    T_B_Od_shake = T_B_Ocurrent * PyKDL.Frame(PyKDL.Rotation.RotZ(rot_diff_Ocurrent.z()+6.0/180.0*math.pi), self.key_up_O * pt[0] + self.key_side_O * pt[1])
                    T_B_W_shake.append(T_B_Od_shake * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse())

                print "shaking..."
                for T_B_W in T_B_W_shake:
                    time = self.velma.getMovementTime(T_B_W, max_v_l = 0.4, max_v_r = 0.4)
#                    self.velma.moveWrist2(T_B_W * self.velma.T_W_T)
#                    raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
                    self.velma.moveWrist(T_B_W, 0.5, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol_in)
                    status1, feedback = self.velma.waitForCartEnd()
                    print "status", status1

                    self.velma.updateTransformations()
                    contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
                    m_id = self.pub_marker.publishSinglePointMarker(contact_B, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

                    contact_L = T_B_L.Inverse() * contact_B
                    contact_depth_L = PyKDL.dot(contact_L, penetration_axis_L)

                    if contact_depth_L > contact_max_depth_L+0.0001:
                        deepest_contact_L = copy.deepcopy(contact_L)
                        contact_max_depth_L = contact_depth_L
                        print "contact_depth_L: %s"%(contact_depth_L)
                        explore_mode = "push"
                        break
                    if status1.error_code != 0:
                        break
                print "done."

                score = contact_depth_L - prev_contact_depth_L
                prev_contact_depth_L = contact_depth_L

                if status1.error_code != 0:
                    break

#                m_id = self.pub_marker.publishSinglePointMarker(T_B_L * PyKDL.Vector(0.01*xi+ori_map_vis_offset[0], 0.01*yi+ori_map_vis_offset[1], score), m_id, r=0, g=0, b=1, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
#                m_id = self.pub_marker.publishSinglePointMarker(T_B_L * PyKDL.Vector(0.01*xi+ori_map_vis_offset[0], 0.01*yi+ori_map_vis_offset[1], ori_map[xi][yi][1]), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

            exit(0)

if __name__ == '__main__':

    rospy.init_node('door_key')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


