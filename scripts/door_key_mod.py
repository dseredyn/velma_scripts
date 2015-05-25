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

    def poseUpdaterThread(self, args, *args2):
        return
        while not rospy.is_shutdown():
            self.pub_marker.publishConstantMeshMarker("package://velma_scripts/data/meshes/klucz_gerda_binary.stl", 0, r=1, g=0, b=0, scale=1.0, frame_id='right_HandPalmLink', namespace='key', T=self.T_E_O)
            self.pub_marker.publishSinglePointMarker(self.key_endpoint_O, 1, r=1, g=1, b=1, a=0.5, namespace='key', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=self.T_E_O)
            self.pub_marker.publishSinglePointMarker(self.T_O_H*PyKDL.Vector(), 2, r=1, g=1, b=0, a=0.5, namespace='key', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.015, 0.015, 0.015), T=self.T_E_O)
            self.pub_marker.publishVectorMarker(self.T_E_O*self.key_endpoint_O, self.T_E_O*(self.key_endpoint_O+self.key_up_O*0.05), 3, 1, 0, 0, frame='right_HandPalmLink', namespace='key', scale=0.001)
            self.pub_marker.publishVectorMarker(self.T_E_O*self.key_endpoint_O, self.T_E_O*(self.key_endpoint_O+self.key_axis_O*0.05), 4, 0, 1, 0, frame='right_HandPalmLink', namespace='key', scale=0.001)

            m_id = 0
            for pt in self.points:
#                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, a=1, namespace='hand', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, a=1, namespace='hand', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

            rospy.sleep(0.1)

    def pointsCollectorThread(self, point_list, frame_name, gripper_name):
        m_id = 0
        while not rospy.is_shutdown() and self.collect_points:
            points = self.velma.getContactPointsInFrame(300, frame_name, gripper_name)
            for pt in points:
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=1, b=0, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.CUBE, scale=Vector3(pt[1]*2, pt[2]*2, 0.001), T=pt[0])
            rospy.sleep(0.1)

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

    def setKeyPosSim(self, T_B_T):
        # transform of the table (z axis is the normal)
        T_B_P = PyKDL.Frame(PyKDL.Rotation.RotX(5.0/180.0*math.pi)*PyKDL.Rotation.RotY(-7.0/180.0*math.pi), PyKDL.Vector(1,0,1.05))
        T_P_B = T_B_P.Inverse()

        T_T_B = T_B_T.Inverse()

        key_endpoint_T = PyKDL.Vector(0.01,-0.015,0.18)

        tool_P = T_P_B * T_B_T * PyKDL.Vector()
        key_endpoint_P = T_P_B * T_B_T * key_endpoint_T

        if tool_P.z() < 0:
            return None

        T_B_T_new = copy.deepcopy(T_B_T)

        if key_endpoint_P.z() < 0:
            # calculate force and torque
            f_P = PyKDL.Vector(0,0,1)
            t_P = (key_endpoint_P - tool_P) * f_P

            f_T = PyKDL.Frame((T_T_B * T_B_P).M) * f_P
            t_T = PyKDL.Frame((T_T_B * T_B_P).M) * t_P

            twist = PyKDL.Twist()
            twist.vel = f_T
            twist.rot = t_T*30.0

            while True:
                T_B_T_new = T_B_T_new * PyKDL.addDelta(PyKDL.Frame(), twist, 0.003)
                key_endpoint_P = T_P_B * T_B_T_new * key_endpoint_T
                if key_endpoint_P.z() >= 0:
                    break
        return T_B_T_new

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
#        T_B_T_1 = PyKDL.Frame(PyKDL.Rotation.RotX(30.0/180.0*math.pi), central_point_B) * PyKDL.Frame(-key_endpoint_T+PyKDL.Vector(0.005, 0.001,0))
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

    def randomOrientation(self, angle):
        while True:
            axis = PyKDL.Vector(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
            if axis.Norm() > 0.001:
                break
        axis.Normalize()
        return PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle))

    def randomOrientationXY(self, angle):
        angle_z = random.uniform(0.0, 2.0*math.pi)
        axis = PyKDL.Vector(math.cos(angle_z), math.sin(angle_z), 0)
        return PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle))

    def initOriMap(self, xsteps, ysteps, xmin, xmax, ymin, ymax):
        # generate a map of orientations and their scores
        ori_map = []
        for angle_x in np.linspace(xmin, xmax, xsteps):
                ori_map.append( [] )
                for angle_y in np.linspace(ymin, ymax, ysteps):
                    axis = PyKDL.Vector(angle_x, angle_y, 0)
                    norm = axis.Norm()
                    if norm < 0.001:
                        ori = PyKDL.Frame()
                    else:
                        axis.Normalize()
                        ori = PyKDL.Frame(PyKDL.Rotation.Rot(axis, norm))
                    score = -1
                    ori_map[-1].append( [ori, score, False] )
        return ori_map

    def getCandidateOri(self, ori_map, x_start, y_start):
                def isNeighbour(ori_map, x_idx, y_idx):
                    if x_idx < 0 or x_idx >= len(ori_map) or y_idx < 0 or y_idx >= len(ori_map[0]):
                        print "ERROR: isNeighbour x_idx, y_idx: %s, %s"%(x_idx, y_idx)
                    if ori_map[x_idx][y_idx][2] == True:
                        print "ERROR: isNeighbour ori_map[%s][%s][2]: %s"%(x_idx, y_idx, ori_map[x_idx][y_idx][2])
                    return (x_idx > 0 and ori_map[x_idx-1][y_idx][2] == True) or (x_idx < len(ori_map)-1 and ori_map[x_idx+1][y_idx][2] == True) or (y_idx > 0 and ori_map[x_idx][y_idx-1][2] == True) or (y_idx < len(ori_map[0])-1 and ori_map[x_idx][y_idx+1][2] == True)

                def extrapolate(ori_map, x_idx, y_idx):
                    if x_idx < 0 or x_idx >= len(ori_map) or y_idx < 0 or y_idx >= len(ori_map[0]):
                        print "ERROR: extrapolate x_idx, y_idx: %s, %s"%(x_idx, y_idx)
                    if ori_map[x_idx][y_idx][2] == True:
                        print "ERROR: extrapolate ori_map[%s][%s][2]: %s"%(x_idx, y_idx, ori_map[x_idx][y_idx][2])
                    gr_values = []

                    if x_idx >= 2 and ori_map[x_idx-1][y_idx][2] == True and ori_map[x_idx-2][y_idx][2] == True:
                        gr_values.append(2*ori_map[x_idx-1][y_idx][1] - ori_map[x_idx-2][y_idx][1])
                    elif x_idx >= 1 and ori_map[x_idx-1][y_idx][2] == True:
                        gr_values.append(ori_map[x_idx-1][y_idx][1])

                    if x_idx < len(ori_map)-2 and ori_map[x_idx+1][y_idx][2] == True and ori_map[x_idx+2][y_idx][2] == True:
                        gr_values.append(2*ori_map[x_idx+1][y_idx][1] - ori_map[x_idx+2][y_idx][1])
                    elif x_idx < len(ori_map)-1 and ori_map[x_idx+1][y_idx][2] == True:
                        gr_values.append(ori_map[x_idx+1][y_idx][1])

                    if y_idx >= 2 and ori_map[x_idx][y_idx-1][2] == True and ori_map[x_idx][y_idx-2][2] == True:
                        gr_values.append(2*ori_map[x_idx][y_idx-1][1] - ori_map[x_idx][y_idx-2][1])
                    elif y_idx >= 1 and ori_map[x_idx][y_idx-1][2] == True:
                        gr_values.append(ori_map[x_idx][y_idx-1][1])

                    if y_idx < len(ori_map[0])-2 and ori_map[x_idx][y_idx+1][2] == True and ori_map[x_idx][y_idx+2][2] == True:
                        gr_values.append(2*ori_map[x_idx][y_idx+1][1] - ori_map[x_idx][y_idx+2][1])
                    elif y_idx < len(ori_map[0])-1 and ori_map[x_idx][y_idx+1][2] == True:
                        gr_values.append(ori_map[x_idx][y_idx+1][1])

                    if len(gr_values) == 0:
                        print "ERROR: extrapolate ori_map[%s][%s][2]: %s   gr_values == []"%(x_idx, y_idx, ori_map[x_idx][y_idx][2])

                    return sum(gr_values) / len(gr_values)

                all_visited = True
                min_ex = None
                # get the cell with the smallest extrapolated score or the cell with no extrapolated score
                for x_idx in range(len(ori_map)):
                    for y_idx in range(len(ori_map[x_idx])):
                        if ori_map[x_idx][y_idx][2] == False:
                            all_visited = False
                            if isNeighbour(ori_map, x_idx, y_idx):
                                ex = extrapolate(ori_map, x_idx, y_idx)
                                if min_ex == None or ex < min_ex[2]:
                                    min_ex = (x_idx, y_idx, ex)
                if min_ex == None:
                    min_ex = (x_start, y_start, 0.0)
                if all_visited:
                    return None
                return min_ex

    def getNextOri(self, ori_map, current_x, current_y):
                def getBestOri(ori_map):
                    best = None
                    for x_idx in range(len(ori_map)):
                        for y_idx in range(len(ori_map[x_idx])):
                            if ori_map[x_idx][y_idx][2] == True:
                                if best == None or ori_map[x_idx][y_idx][1] > best[0]:
                                    best = (ori_map[x_idx][y_idx][1], x_idx, y_idx)
                    return best

                candidates = []
                if current_x > 0:
                    if ori_map[current_x-1][current_y][2] == False:
                        candidates.append( (current_x-1, current_y) )
                        candidates.append( (current_x-1, current_y) )
                        candidates.append( (current_x-1, current_y) )
                    candidates.append( (current_x-1, current_y) )
                if current_x < len(ori_map)-1:
                    if ori_map[current_x+1][current_y][2] == False:
                        candidates.append( (current_x+1, current_y) )
                        candidates.append( (current_x+1, current_y) )
                        candidates.append( (current_x+1, current_y) )
                    candidates.append( (current_x+1, current_y) )
                if current_y > 0:
                    if ori_map[current_x][current_y-1][2] == False:
                        candidates.append( (current_x, current_y-1) )
                        candidates.append( (current_x, current_y-1) )
                        candidates.append( (current_x, current_y-1) )
                    candidates.append( (current_x, current_y-1) )
                if current_y < len(ori_map[0])-1:
                    if ori_map[current_x][current_y+1][2] == False:
                        candidates.append( (current_x, current_y+1) )
                        candidates.append( (current_x, current_y+1) )
                        candidates.append( (current_x, current_y+1) )
                    candidates.append( (current_x, current_y+1) )

                best = getBestOri(ori_map)
                if best == None or (best[1] == current_x and best[2] == current_y) or best[0] < ori_map[current_x][current_y][1] + 0.0002:
                    idx = random.randint(0, len(candidates)-1)
                    return candidates[idx]

                dx = best[1]-current_x
                dy = best[2]-current_y
                if abs(dx) > abs(dy):
                    if dx > 0:
                        return (current_x+1, current_y)
                    else:
                        return (current_x-1, current_y)
                else:
                    if dy > 0:
                        return (current_x, current_y+1)
                    else:
                        return (current_x, current_y-1)

                return None

    def getBestOri(self, ori_map):
        min_ori = None
        for x_idx in range(len(ori_map)):
            for y_idx in range(len(ori_map[x_idx])):
                if ori_map[x_idx][y_idx][2] == True:
                    if min_ori == None or min_ori[2] > ori_map[x_idx][y_idx][1]:
                        min_ori = (x_idx, y_idx, ori_map[x_idx][y_idx][1])
        return min_ori

    def oriGradientDescendUnitTest(self):
        ori_map = self.initOriMap(10, 10, -10.0/180.0*math.pi, 10.0/180.0*math.pi, -10.0/180.0*math.pi, 10.0/180.0*math.pi)
        m_id = 0
        while not rospy.is_shutdown():
            ex = self.getCandidateOri(ori_map, 1, 2)
            if ex == None:
                break
            x, y, score = ex
            diff = PyKDL.diff(PyKDL.Frame(), ori_map[x][y][0])
            score_real = diff.rot.Norm()

            ori_map[x][y][2] = True
            ori_map[x][y][1] = score_real

            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(x*0.01, y*0.01, score*0.2), m_id, r=1, g=0, b=0, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)
            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(x*0.01, y*0.01, score_real*0.2), m_id, r=0, g=1, b=0, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)

            print score_real, score, x, y
            raw_input(".")

    def spin(self):
        simulation_only = False

        key_endpoint_T = PyKDL.Vector(0,0,0.2)
        T_B_P = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(1,0,1))

        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

#        self.oriGradientDescendUnitTest()
#        exit(0)

#        self.estCommonPointTest()
#        exit(0)

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

        # start thread for updating key position in rviz
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

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

            # generate orientations map
#            ori_map = self.initOriMap(20, 20, -4.0/180.0*math.pi, 4.0/180.0*math.pi, -4.0/180.0*math.pi, 4.0/180.0*math.pi)

#            ori_map_vis_offset = (0.03, -0.1)
#            for xi in range(len(ori_map)):
#                for yi in range(len(ori_map[xi])):
#                    n = ori_map[xi][yi][0] * PyKDL.Vector(0,0,0.02)
#                    m_id = self.pub_marker.publishVectorMarker(T_B_L * PyKDL.Vector(0.01*xi+ori_map_vis_offset[0], 0.01*yi+ori_map_vis_offset[1], 0), T_B_L * (PyKDL.Vector(0.01*xi+ori_map_vis_offset[0], 0.01*yi+ori_map_vis_offset[1], 0)+n), m_id, 1, 0, 0, frame='torso_base', namespace='default', scale=0.001)
#                    rospy.sleep(0.001)

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




























            self.velma.updateTransformations()
            contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
            central_ori_ref_T_O_O = PyKDL.Frame()
            pt_contact_O_in_ref = self.T_E_O.Inverse() * self.velma.T_W_E.Inverse() * T_B_Wref.Inverse() * contact_B
            deepest_contact = PyKDL.dot(pt_contact_O_in_ref, self.key_axis_O)
            deepest_contact_O_in_ref = pt_contact_O_in_ref
            pt_desired_O_in_ref = self.key_endpoint_O

            T_O_Od = PyKDL.Frame()
            force = 10.0
            tries_counter = 0
            while True:
                prev_T_O_Od = copy.deepcopy(T_O_Od)
                T_O_Od = self.randomOrientationXY( random.uniform(0.0, 7.0/180.0*math.pi) )# * PyKDL.Frame(self.key_up_O*random.uniform(-0.002, 0.002) + self.key_side_O*random.uniform(-0.002, 0.002))

#                tries_counter += 1
#                if tries_counter > 40:
#                    force += 3.0
#                    tries_counter = 0
#                    print "desired force:", force
#                    if force > 15.0:
#                        print "too big desired force"
#                        exit(0)

                pt_desired_O_in_ref = self.key_endpoint_O
                key_end_depth = PyKDL.dot(deepest_contact_O_in_ref-pt_desired_O_in_ref, self.key_axis_O)

#                T_B_Wd = T_B_Wref * self.velma.T_W_E * self.T_E_O * PyKDL.Frame(self.key_endpoint_O) * central_ori_ref_T_O_O * T_O_Od * PyKDL.Frame(-self.key_endpoint_O + self.key_axis_O*(key_end_depth-0.02)) * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
#                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
#                raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
#                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
#                status, feedback = self.velma.waitForCartEnd()
#                print "status", status
#                if status.error_code != 0:
#                    print "unexpected high contact force", feedback
#                    exit(0)

                T_B_Wd = T_B_Wref * self.velma.T_W_E * self.T_E_O * PyKDL.Frame(self.key_endpoint_O) * central_ori_ref_T_O_O * T_O_Od * PyKDL.Frame(-self.key_endpoint_O + self.key_axis_O * (key_end_depth+force/k_high.force.z)) * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.1, max_v_r = 0.1)
                raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol3)
                status, feedback = self.velma.waitForCartEnd()
                print "status", status
                self.velma.updateTransformations()
                contact_B = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * self.key_endpoint_O
                contact_points_B.append(contact_B)
                m_id = self.pub_marker.publishSinglePointMarker(contact_B, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

                pt_contact_O_in_ref = self.T_E_O.Inverse() * self.velma.T_W_E.Inverse() * T_B_Wref.Inverse() * contact_B
                contact_depth = PyKDL.dot(pt_contact_O_in_ref, self.key_axis_O)
                if contact_depth > deepest_contact:
                    central_ori_ref_T_O_O = central_ori_ref_T_O_O * PyKDL.Frame(T_O_Od.M)
                    deepest_contact_O_in_ref = pt_contact_O_in_ref
                    angle = 1.0/180.0*math.pi
                    deepest_contact = contact_depth

                if status.error_code != 0:
                    self.velma.updateTransformations()
                    T_O_Od5 = PyKDL.Frame(self.key_axis_O * 5.0/k_high.force.z)
                    T_B_Wd = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * T_O_Od5 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                    time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
#                    self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
#                    status, feedback = self.velma.waitForCartEnd()
#                    if status.error_code != 0:
#                        print "unexpected high contact force", feedback
#                        exit(0)



            exit(0)


            k_hole = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(10, 10, 10))
            max_force = 10.0
            max_torque = 10.0
            path_tol = (PyKDL.Vector(max_force/k_hole.force.x, max_force/k_hole.force.y, max_force/k_hole.force.z), PyKDL.Vector(max_torque/k_hole.torque.x, max_torque/k_hole.torque.y, max_torque/k_hole.torque.z))
            max_force2 = 20.0
            max_torque2 = 20.0
            path_tol2 = (PyKDL.Vector(max_force2/k_hole.force.x, max_force2/k_hole.force.y, max_force2/k_hole.force.z), PyKDL.Vector(max_torque2/k_hole.torque.x, max_torque2/k_hole.torque.y, max_torque2/k_hole.torque.z))
            print "path tolerance:", path_tol

            print "setting stiffness to", k_hole
            self.velma.moveImpedance(k_hole, 1.0)
            if self.velma.checkStopCondition(1.0):
                exit(0)
            print "done."

            spiral_idx = 0
            while True:
                pt = spiral[spiral_idx]
                spiral_idx += 1
                if spiral_idx >= len(spiral):
                    spiral_idx = 0
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

                if status.error_code != 0:
                    spiral_idx = 0
                    self.velma.updateTransformations()
                    T_O_Od5 = PyKDL.Frame(self.key_axis_O * (key_end_depth+force/k_high.force.z))
                    T_B_Wd = self.velma.T_B_W * self.velma.T_W_E * self.T_E_O * T_O_Od5 * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                    time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                    self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
                    status, feedback = self.velma.waitForCartEnd()
                    if status.error_code != 0:
                        print "unexpected high contact force", feedback
                        exit(0)




            exit(0)


            while not rospy.is_shutdown():
                print probed_points
                for i in range(1000):
                    x = random.uniform(-search_radius, search_radius)
                    y = random.uniform(-search_radius, search_radius)
                    if math.sqrt(x*x + y*y) > search_radius:
                        continue
                    pt_ok = True
                    for pt in probed_points:
                        dist = math.sqrt((pt[0]-x)*(pt[0]-x) + (pt[1]-y)*(pt[1]-y))
                        if dist < min_dist:
                            pt_ok = False
                            break
                    if pt_ok:
                        break

                if pt_ok:
                    probed_points.append( (x,y) )
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
                        print feedback

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

                    self.velma.updateTransformations()
                    T_B_Wd = self.T_B_O_nearHole * T_O_Od * self.T_E_O.Inverse() * self.velma.T_W_E.Inverse()
                    time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.02, max_v_r = 0.04)
                    raw_input("Press ENTER to movie the wrist in " + str(time) + " s...")
                    self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(40,40,40), Vector3(14,14,14)), path_tol=path_tol2)
                    status, feedback = self.velma.waitForCartEnd()
                    print "status", status
                    if status.error_code != 0:
                        print feedback
                else:
                    print "all space searched"
                    exit(0)






#        while not rospy.is_shutdown():
#            rospy.sleep(1)

        exit(0)

        if True:
            print "collecting contact points with the door..."
            door_points = []

            # move the tool wrt the current pose
            self.velma.updateTransformations()

            T_B_T = self.velma.T_B_W * self.velma.T_W_T

            door_probe_poses_T = [
            [PyKDL.Frame(), PyKDL.Frame(PyKDL.Vector(0,0,0.1)) ],
            [PyKDL.Frame(PyKDL.Vector(0.1,0,0)), PyKDL.Frame(PyKDL.Vector(0.1,0,0.1)) ],
            [PyKDL.Frame(PyKDL.Vector(0,0.1,0)), PyKDL.Frame(PyKDL.Vector(0,0.1,0.1)) ]
            ]

            path_tol = (PyKDL.Vector(), PyKDL.Vector())
            for door_probe_T in door_probe_poses_T:
                T_B_Wd = T_B_T * door_probe_T[0]
                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.1, max_v_r = 0.2)
                raw_input("Press ENTER to move the hand in " + str(time) + " s...")
                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if self.velma.checkStopCondition(time + 0.1):
                    exit(0)

                T_B_Wd = T_B_T * door_probe_T[1]
                time = self.velma.getMovementTime(T_B_Wd, max_v_l = 0.1, max_v_r = 0.2)
                raw_input("Press ENTER to move the hand in " + str(time) + " s...")
                self.velma.moveWrist(T_B_Wd, time, Wrench(Vector3(20,20,20), Vector3(4,4,4)), path_tol=path_tol)
                if self.velma.checkStopCondition(time + 0.1):
                    exit(0)

                # TODO: get and save contact point
#                door_points.append()



            # estimate the door plane
#            T_B_D = velmautils.estPlane(door_points)
            # visualise the door plane
#            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(1.0, 1.0, 0.003), T=T_B_D)

        exit(0)



        print "collecting contact points with the lock.."
        lock_points = []
        if simulation_only:
            sim_lock_hole = PyKDL.Vector(0.9-0.0036,0.2, 1.3)
            for i in range(400):
                    pt = PyKDL.Vector(random.gauss(0.0, 0.001), random.gauss(0.0, 0.01), random.gauss(0.0, 0.01))
                    if pt.Norm() > 0.008:
                        lock_points.append( sim_lock_hole + pt )
        else:
            self.collect_points = True
            thread.start_new_thread(self.pointsCollectorThread, (lock_points,'torso_link2', 'left'))
            raw_input("Press ENTER to stop collecting contacts...")
            self.collect_points = False
            rospy.sleep(0.5)
        print "done."
        for pt in lock_points:
            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=0, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
        rospy.sleep(0.1)

        print "collecting contact points with the key end..."
        key_points_E = []
        key_points_B = []
        if simulation_only:
            sim_key_error_O = PyKDL.Vector(-0.002, 0.001, 0.001)
            for i in range(200):
                    pt = sim_key_error_O + PyKDL.Vector(random.gauss(0.0, 0.001), random.gauss(0.0, 0.002), random.gauss(0.0, 0.002))
                    key_points_E.append( self.T_E_O * (self.key_endpoint_O + pt) )
        else:
            self.collect_points = True
#            thread.start_new_thread(self.pointsCollectorThread, (key_points_E,'right_HandPalmLink', 'left'))
            thread.start_new_thread(self.pointsCollectorThread, (key_points_B,'torso_link2', 'left'))
            raw_input("Press ENTER to stop collecting contacts...")
            self.collect_points = False
            rospy.sleep(0.5)
        for pt in key_points_B:
            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=1, b=1, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.SPHERE, scale=Vector3(0.001, 0.001, 0.001), T=None)
        print "done."
        rospy.sleep(0.1)
#        self.points = key_points_E

#        print len(contacts)
#        for pt in contacts:
#            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=0, b=1, a=1, namespace='default', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)

        while not rospy.is_shutdown():
            rospy.sleep(1)
        exit(0)



        # door normal
        n_door_B = PyKDL.Vector(-1,0,0)

        T_B_W_in_hole = None
#        T_B_W_in_hole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.361231179791, 0.0198304562193, 0.486979840032, 0.794965045241), PyKDL.Vector(0.60853551459, -0.220618900285, 1.30990416702))

        if T_B_W_in_hole == None:
            print "provide the pose of the key hole..."
            raw_input("Put the grasped key deep into key hole and press Enter to continue...")
            self.velma.updateTransformations()
            T_B_W_in_hole = self.velma.T_B_W
            print "T_B_W_in_hole"
            q = T_B_W_in_hole.M.GetQuaternion()
            p = T_B_W_in_hole.p
            print "T_B_W_in_hole = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s, %s, %s, %s), PyKDL.Vector(%s, %s, %s))"%(q[0], q[1], q[2], q[3], p[0], p[1], p[2])

        raw_input("put the gripper in the safe place near the key hole and press Enter to continue...")
        self.velma.updateTransformations()

        print "moving the desired pose to the current pose..."
        self.velma.moveWrist(self.velma.T_B_W, 1.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(1.0):
            exit(0)
        print "done"

        print "setting stiffness to bigger value"
        k_low_2 = Wrench(Vector3(10.0, 10.0, 10.0), Vector3(5, 5, 5))
        self.velma.moveImpedance(k_low_2, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")

        print "setting stiffness to bigger value"
        k_low_3 = Wrench(Vector3(50.0, 50.0, 50.0), Vector3(25, 25, 25))
        self.velma.moveImpedance(k_low_3, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")
        print "setting stiffness to bigger value"
        k_low_4 = Wrench(Vector3(500.0, 500.0, 500.0), Vector3(250, 250, 250))
        self.velma.moveImpedance(k_low_4, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")
        print "setting stiffness to bigger value"
        k_big = Wrench(Vector3(2000.0, 2000.0, 2000.0), Vector3(300, 300, 300))
        self.velma.moveImpedance(k_big, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to move the wrist...")
        T_B_Wd = PyKDL.Frame(n_door_B*0.1) * T_B_W_in_hole
        print "moving the wrist..."
        self.velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(10.0):
            exit(0)
        print "done"

        raw_input("Press Enter to move the wrist...")
        T_B_Wd = PyKDL.Frame(n_door_B*0.0) * T_B_W_in_hole
        print "moving the wrist..."
        self.velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(10.0):
            exit(0)
        print "done"

        # start with very low stiffness
        print "setting stiffness to very low value"
        k_low = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.velma.moveImpedance(k_low, 0.5)
        if self.velma.checkStopCondition(0.5):
            exit(0)
        print "done."

        self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 1.7038538203360971-5.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)
        self.velma.moveHand([2.0177062895374993-5.0/180.0*math.pi, 2.3174461354903535, 1.7038538203360971-5.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)
        self.velma.moveHand([2.0177062895374993-10.0/180.0*math.pi, 2.3174461354903535, 1.7038538203360971-10.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)

        exit(0)

if __name__ == '__main__':

    rospy.init_node('door_key')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


