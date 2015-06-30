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

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

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
import random
import openravepy
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import itertools
import operator
import rospkg

import velmautils
from velma import Velma
import openraveinstance
from prolatehyperspheroid import ProlateHyperspheroid
import headkinematics

import rosparam

from multiprocessing import Process, Queue

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

def uniformInBall2(r, limits, q_start):
                tries = 0
                while True:
                    tries += 1
                    n = len(limits)
                    vec = np.empty(n)
                    for i in range(n):
                        lo_limit = max(limits[i][0], q_start[i]-r)
                        up_limit = min(limits[i][1], q_start[i]+r)
                        vec[i] = random.uniform(lo_limit, up_limit)
                    if np.linalg.norm(q_start - vec) <= r:
#                        print "uniformInBall2", tries
                        return vec

class LooAtTaskRRT:
    def __init__(self, openrave):
        self.openrave = openrave

        v_rot = 0.800
        v_lean = 0.375
        v_head = 0.392
        h_cam = 0.0
        v_cam = 0.225
        self.head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)

        self.kinect_fov = 30.0/180.0*math.pi

        # target: key pocket
        self.vis_targets = [
        ("vis_target_0", 0.1, PyKDL.Vector(0, -0.4, 1.0)),
        ("vis_target_1", 0.1, PyKDL.Vector(0.1, -0.4, 1.0)),
        ("vis_target_2", 0.1, PyKDL.Vector(0.1, -0.5, 1.0)),
        ("vis_target_3", 0.1, PyKDL.Vector(0, -0.5, 1.0)),
#        ("vis_target_4", 0.1, PyKDL.Vector(0.05, -0.45, 1.0)),
        ]
        self.head_target_B = PyKDL.Vector()
        for target in self.vis_targets:
            self.head_target_B += target[2]
        self.head_target_B = self.head_target_B / len(self.vis_targets)

        self.vis_bodies = []
        # target: test (vertical axis at the door plane)
#        self.vis_targets = [
#        ("vis_target_0", 0.1, PyKDL.Vector(1, 0.0, 1.2)),
#        ("vis_target_1", 0.1, PyKDL.Vector(1, 0.0, 1.3)),
#        ("vis_target_2", 0.1, PyKDL.Vector(1, 0.0, 1.4)),
#        ("vis_target_3", 0.1, PyKDL.Vector(1, 0.0, 1.5)),
#        ("vis_target_4", 0.1, PyKDL.Vector(1, 0.0, 1.6)),
#        ]

        for (name, diam, pos) in self.vis_targets:
            body = self.openrave.addSphere(name, diam)
            body.SetTransform(KDLToOpenrave(PyKDL.Frame(pos)))
            self.vis_bodies.append( body )
            self.openrave.env.Remove( body )

        self.dof_names = [
        "head_pan_joint",
        "head_tilt_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "torso_0_joint",
        ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
            self.openrave.switchCollisionModel("velmasimplified0")
            rays_hit = 0
            m_id = 0

            self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
#            openrave.env.UpdatePublishedBodies()

            for body in self.vis_bodies:
                self.openrave.env.Add( body )
            T_W_C = OpenraveToKDL(self.openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()
            cam_W = T_W_C * PyKDL.Vector()
            cam_dir_W = PyKDL.Frame(T_W_C.M) * PyKDL.Vector(0,0,0.5)

            # create rays connecting the optical frame and the target objects
            for (name, diam, pos_W) in self.vis_targets:
                pos_C = T_C_W * pos_W
                dir_W = pos_W - cam_W
                if pos_C.z() < 0.1:
                    continue
                if velmautils.getAngle(PyKDL.Vector(0,0,1), pos_C) > self.kinect_fov:
                    continue

                report = CollisionReport()
                ret = self.openrave.env.CheckCollision(Ray((cam_W[0], cam_W[1], cam_W[2]), (dir_W[0], dir_W[1], dir_W[2])), report)
                if ret and report.plink1 != None and report.plink1.GetParent().GetName().find("vis_target_") == 0:
                    rays_hit += 1
                else:
                    pass
            for body in self.vis_bodies:
                self.openrave.env.Remove( body )

            return rays_hit == 4

    def SampleGoal(self, start_q, shortest_path_len):
            q_goal = np.empty(len(self.dof_names))
            for tries in range(200):
                if shortest_path_len == None:
                    for i in range(len(self.dof_names)):
                        q_goal[i] = random.uniform(self.dof_limits[i][0]+0.01, self.dof_limits[i][1]-0.01)
                else:
                    q_goal = uniformInBall2(shortest_path_len, self.dof_limits, start_q)

                self.head_kin.UpdateTorsoPose(q_goal[self.dof_indices_map["torso_0_joint"]], self.openrave.robot_rave.GetJoint("torso_1_joint").GetValue(0))
                self.head_kin.UpdateTargetPosition(self.head_target_B.x(), self.head_target_B.y(), self.head_target_B.z())
                self.head_kin.TransformTargetToHeadFrame()
                joint_pan, joint_tilt = self.head_kin.CalculateHeadPose()
                if joint_pan == None:
                    continue
                joint_pan = max(joint_pan, self.dof_limits[self.dof_indices_map["head_pan_joint"]][0])
                joint_pan = min(joint_pan, self.dof_limits[self.dof_indices_map["head_pan_joint"]][1])
                joint_tilt = max(joint_tilt, self.dof_limits[self.dof_indices_map["head_tilt_joint"]][0])
                joint_tilt = min(joint_tilt, self.dof_limits[self.dof_indices_map["head_tilt_joint"]][1])
                q_goal[self.dof_indices_map["head_pan_joint"]] = joint_pan
                q_goal[self.dof_indices_map["head_tilt_joint"]] = joint_tilt

#                if shortest_path_len != None and CostLine(start_q, q_goal) > shortest_path_len:
#                    continue

#                if isStateValid(q_goal, dof_indices) and checkGoal(q_goal, dof_indices):
#                    return q_goal
                if self.checkGoal(q_goal):
                    return q_goal
            return None

class KeyRotTaskRRT:
    def __init__(self, openrave):
        self.openrave = openrave

        self.T_E_O = PyKDL.Frame()
        self.T_O_E = self.T_E_O.Inverse()
        self.key_axis_O = PyKDL.Vector(0,0,1)
        self.key_up_O = PyKDL.Vector(1,0,0)
        self.key_side_O = self.key_axis_O * self.key_up_O

        self.key_endpoint_O = PyKDL.Vector(0.000256401261281, -0.000625166847342, 0.232297442735)
        self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.71891504857, -0.0529880479354, 0.691118088949, 0.0520500417212), PyKDL.Vector(0.883081316461, -0.100813768303, 0.95381559114))

        # get the transformation from wrist to palm
        link_E = self.openrave.robot_rave.GetLink("right_HandPalmLink")
        link_W = self.openrave.robot_rave.GetLink("right_arm_7_link")
        T_World_E = OpenraveToKDL(link_E.GetTransform())
        T_World_W = OpenraveToKDL(link_W.GetTransform())
        self.T_W_E = T_World_W.Inverse() * T_World_E
        self.T_E_W = self.T_W_E.Inverse()

        self.key_traj1_T_B_W = []
        for angle in np.linspace(0.0/180.0*math.pi, -180.0/180.0*math.pi, 10):
            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
            self.key_traj1_T_B_W.append( (T_B_W, angle) )
        self.key_traj2_T_B_W = []
        for angle in np.linspace(0.0/180.0*math.pi, 180.0/180.0*math.pi, 10):
            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
            self.key_traj2_T_B_W.append( (T_B_W, angle) )
        self.velma_solvers = velmautils.VelmaSolvers()

        self.dof_names = [
#        "head_pan_joint",
#        "head_tilt_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
#        "left_arm_4_joint",
#        "left_arm_5_joint",
#        "left_arm_6_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "right_arm_4_joint",
        "right_arm_5_joint",
        "right_arm_6_joint",
        "torso_0_joint",
#        "torso_1_joint",
        ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_names_ik = [
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "right_arm_4_joint",
        "right_arm_5_joint",
        "right_arm_6_joint",
        ]

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
        # interpolate trajectory (in the cartesian space)
        self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
        link_E = self.openrave.robot_rave.GetLink("right_HandPalmLink")
        T_World_E = OpenraveToKDL(link_E.GetTransform())
        T_B_O = self.openrave.T_World_Br.Inverse() * T_World_E * self.T_E_O
        diff = PyKDL.diff(self.T_B_O_nearHole, T_B_O)
        if diff.vel.Norm() > 0.02 or diff.rot.Norm() > 10.0/180.0*math.pi:
#            print "too far from goal"
            return False

        angle1 = 0.0
        for T_B_W, angle in self.key_traj1_T_B_W:
            init_js = self.openrave.getRobotConfigurationRos()
            traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_W)
            if traj == None:
                break
            angle1 = angle
            qar = {}
            for qi in range(len(traj[-1])):
                qar["right_arm_"+str(qi)+"_joint"] = traj[-1][qi]
            self.openrave.updateRobotConfigurationRos(qar)

        self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
        angle2 = 0.0
        for T_B_W, angle in self.key_traj2_T_B_W:
            init_js = self.openrave.getRobotConfigurationRos()
            traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_W)
            if traj == None:
                break
            angle2 = angle
            qar = {}
            for qi in range(len(traj[-1])):
                qar["right_arm_"+str(qi)+"_joint"] = traj[-1][qi]
            self.openrave.updateRobotConfigurationRos(qar)

        if abs(angle1-angle2) > 190.0/180.0*math.pi:
            return True
        else:
            return False

    def SampleGoal(self, start_q, shortest_path_len):
        self.openrave.switchCollisionModel("velmasimplified0")

        T_B_E = self.T_B_O_nearHole * self.T_O_E

        q_goal = np.empty(len(self.dof_names))
        for tries in range(200):
            if shortest_path_len == None:
                for i in range(len(self.dof_names)):
                    q_goal[i] = random.uniform(self.dof_limits[i][0]+0.01, self.dof_limits[i][1]-0.01)
            else:
                q_goal = uniformInBall2(shortest_path_len, self.dof_limits, start_q)

            free_dof_idx = self.dof_indices_map[self.openrave.free_joint["right_arm"]]
            freevalues = [ (q_goal[free_dof_idx]-self.dof_limits[free_dof_idx][0])/(self.dof_limits[free_dof_idx][1]-self.dof_limits[free_dof_idx][0]) ]
            self.openrave.robot_rave.SetDOFValues(q_goal, self.dof_indices)
            solutions = self.openrave.findIkSolutions(T_B_E, man_name="right_arm", freevalues=freevalues)

            found_goal = False
            for sol in solutions:
                for arm_dof_idx in range(len(self.dof_names_ik)):
                    dof_name = self.dof_names_ik[arm_dof_idx]
                    q_goal[self.dof_indices_map[dof_name]] = sol[arm_dof_idx]
                if self.checkGoal(q_goal):
                    found_goal = True
                    break
            if found_goal:
#                self.openrave.robot_rave.SetDOFValues(q_goal, dof_indices)
#                raw_input(".")
                return q_goal

        return None

class TestOrOctomap:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def inputThread(self, arg):
        raw_input("Press ENTER to stop planning...")
        self.stop_planning = True

    def planVis(self, openrave):
      with openrave.env:
        debug = True
        m_id = 0

        if debug:
            self.pub_marker.eraseMarkers(0,3000, frame_id='world')
            rospy.sleep(0.01)

        #
        # RRT*
        #
        def RRTstar(openrave):

#            print "planning for %s joints"%(len(dof_indices))

#            ETA = 60.0/180.0*math.pi
#            gamma = 0.5
#            d = len(dof_indices)
            
            def Distance(q1, q2):
                return np.linalg.norm(q1-q2)

            def GetPath(E, q_idx):
                if not q_idx in E:
                    return [q_idx]
                parent_idx = E[q_idx]
                q_list = GetPath(E, parent_idx)
                q_list.append(q_idx)
                return q_list

            def CostLine(q1, q2):
                cost = Distance(q1, q2)# * (np.linalg.norm(q1-self.q_init) + np.linalg.norm(q2-self.q_init)) * 0.5
                return cost

            def Cost(V, E, q_idx):
                if not q_idx in E:
                    return 0.0
                parent_idx = E[q_idx]
                if not parent_idx in V:
                    print "not parent_idx in V: %s"%(parent_idx)
                if not q_idx in V:
                    print "not q_idx in V: %s"%(q_idx)
                cost = CostLine(V[parent_idx], V[q_idx]) + Cost(V, E, parent_idx)
                return cost

            def DrawPath(V, E, q_idx):
                if not q_idx in E:
                    return 0
                parent_idx = E[q_idx]
                m_id = DrawPath(V, E, parent_idx)
                m_id = self.pub_marker.publishVectorMarker(PyKDL.Vector(V[parent_idx][0], V[parent_idx][1], V[parent_idx][2]), PyKDL.Vector(V[q_idx][0], V[q_idx][1], V[q_idx][2]), m_id, 1, 0, 0, frame='world', namespace='shortest_path', scale=0.02)
                return m_id

            if debug:
                edge_ids = {}
                edge_id = 0

            shortest_path_len = None
            shortest_path_len_prev = None
            best_vis = 0
            best_q = None
            best_q_idx = None
            V_vis = []
            V = {}
            E = {}
            goal_V_ids = []
            q_init = openrave.robot_rave.GetDOFValues()

            self.queue_master.put( ("addFirstNode", q_init), True )
            resp = self.queue_slave.get(True)
            if resp[0] != "addFirstNode":
                print "ERROR resp (addFirstNode):", resp[0]
                exit(0)
            V_update_q_new, dof_names = resp[1:]

            V[0] = V_update_q_new

#            self.q_init = np.array(q_init)
            q_new_idx = 0
#            V[0] = np.array(q_init)
            total_collision_checks = 0
            for k in range(100000):
              produced = 0
              consumed = 0

              self.queue_master.put( ("addNode", V, E, shortest_path_len, best_q_idx, goal_V_ids), True )
              while True:
                try:
                    resp = self.queue_slave.get(False)
                except:
                    break
                if resp[0] != "addNode":
                    print "ERROR resp (addNode):", resp[0]
                    exit(0)

                V_update_q_new, E_update, goal, collision_checks = resp[1:]
                total_collision_checks += collision_checks
#                print "             total_collision_checks", total_collision_checks
                if V_update_q_new != None:
                    allow_update = True
                    for (vi_ch, vi_pa) in E_update:
                        if vi_ch != -1 and not vi_ch in V:
                            allow_update = False
                            break
                        if vi_pa != -1 and not vi_pa in V:
                            allow_update = False
                            break
                    if not allow_update:
                        print "discarding changes"
                        continue
                    # update the graph
                    q_new_idx += 1
                    V[q_new_idx] = V_update_q_new
                    for (vi_ch, vi_pa) in E_update:
                        if vi_ch == -1:
                            vi_ch = q_new_idx
                        elif not vi_ch in V:
                            print "ERROR: vi_ch",vi_ch
                            continue
                        if vi_pa == -1:
                            vi_pa = q_new_idx
                        elif not vi_pa in V:
                            print "ERROR: vi_pa",vi_pa
                            continue
                        E[vi_ch] = vi_pa
                        if not vi_ch in edge_ids:
                            edge_ids[vi_ch] = edge_id
                            edge_id += 1
                        self.pub_marker.publishVectorMarker(PyKDL.Vector(V[vi_pa][0], V[vi_pa][1], V[vi_pa][2]), PyKDL.Vector(V[vi_ch][0], V[vi_ch][1], V[vi_ch][2]), edge_ids[vi_ch], 0, 1, 0, frame='world', namespace='edges', scale=0.01)

                    if goal:
                        goal_V_ids.append(q_new_idx)

                    print "len(V) len(goal_V_ids)", len(V), len(goal_V_ids)

                    for goal_idx in goal_V_ids:
                        goal_cost = Cost(V, E, goal_idx)
                        if shortest_path_len == None or shortest_path_len > goal_cost:
                            best_q_idx = goal_idx
                            best_q = V[best_q_idx]
                            shortest_path_len_old = shortest_path_len
                            shortest_path_len =  goal_cost
                            #print "*********** found better goal:", shortest_path_len
                            print " %s  , shortest_path: %s"%(k, shortest_path_len)
                            self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                            DrawPath(V, E, best_q_idx)

                    if True and shortest_path_len_prev != shortest_path_len:
                        rem_nodes = []
                        for vi in V:
                            if CostLine(V[0], V[vi]) > shortest_path_len:
                                rem_nodes.append(vi)
                        print "removing nodes:", len(rem_nodes)
                        for vi in rem_nodes:
                            del V[vi]
                            self.pub_marker.eraseMarkers(edge_ids[vi], edge_ids[vi]+1, frame_id='torso_base', namespace='edges')
                            del edge_ids[vi]
                            del E[vi]
                            if vi in goal_V_ids:
                                goal_V_ids.remove(vi)
                        shortest_path_len_prev = shortest_path_len
                        while True:
                          orphan_nodes = []
                          for vi in E:
                              if (not vi in rem_nodes) and (E[vi] in rem_nodes):
                                  orphan_nodes.append(vi)
                          print "orphan_nodes", len(orphan_nodes)
                          rem_nodes = orphan_nodes
                          if len(rem_nodes) == 0:
                              break
#                          print "removing nodes:", len(rem_nodes)
                          for vi in rem_nodes:
                              del V[vi]
                              self.pub_marker.eraseMarkers(edge_ids[vi], edge_ids[vi]+1, frame_id='torso_base', namespace='edges')
                              del edge_ids[vi]
                              del E[vi]
                              if vi in goal_V_ids:
                                  goal_V_ids.remove(vi)

              if self.stop_planning:
                  break


            path = GetPath(E, best_q_idx)
            print path

            traj = []
            for i in range(len(path)-1):
                q_idx1 = path[i]
                q_idx2 = path[i+1]
                for f in np.linspace(0.0, 1.0, 40):
                    traj.append( V[q_idx1] * (1.0 - f) + V[q_idx2] * f )

            while True:
                raw_input(".")
                openrave.showTrajectory(dof_names, 10.0, traj)

#            for q in V_vis:
#                openrave.robot_rave.SetDOFValues(q, dof_indices)
#                openrave.env.UpdatePublishedBodies()
#                raw_input(".")

        self.stop_planning = False
        thread.start_new_thread(self.inputThread, (None,))

        RRTstar(openrave)

    def openraveWorker(self, process_id, env_file, xacro_uri, srdf_path, configuration_ros, queue_master, queue_slave):
      openrave = openraveinstance.OpenraveInstance()
      openrave.startOpenraveURDF(env_file=env_file, viewer=False)
      openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

      openrave.updateRobotConfigurationRos(configuration_ros)
      collision_checks = [0]

      taskrrt = LooAtTaskRRT(openrave)
#      taskrrt = KeyRotTaskRRT(openrave)

      with openrave.env:

        ETA = 60.0/180.0*math.pi
        gamma = 0.5

        def Distance(q1, q2):
                return np.linalg.norm(q1-q2)

        def GetPath(E, q_idx):
                if not q_idx in E:
                    return [q_idx]
                parent_idx = E[q_idx]
                q_list = GetPath(E, parent_idx)
                q_list.append(q_idx)
                return q_list

        def CostLine(q1, q2):
                cost = Distance(q1, q2)# * (np.linalg.norm(q1-q_start) + np.linalg.norm(q2-q_start)) * 0.5
                return cost

        def isStateValid(q, dof_indices):
            openrave.switchCollisionModel("velmasimplified1")

            collision_checks[0] += 1
            is_valid = True
            current_q = openrave.robot_rave.GetDOFValues(dof_indices)
            openrave.robot_rave.SetDOFValues(q, dof_indices)
            report1 = CollisionReport()
            report2 = CollisionReport()
 	    if openrave.env.CheckCollision(openrave.robot_rave, report2) or openrave.robot_rave.CheckSelfCollision(report1):
                is_valid = False
            return is_valid

        def SampleFree(dof_indices, dof_limits, start_q, shortest_path_len, best_q_idx, goal_V_ids, V, E):
                num_dof = len(dof_limits)
                search_near_subpath = False

                random_0_1 = random.uniform(0,1)

                if len(goal_V_ids) > 0 and random_0_1 < 0.05:
                    path = GetPath(E, best_q_idx)
#                    path = GetPath(E, goal_V_ids[random.randint(0, len(goal_V_ids)-1)])
                    if len(path) >=4:
                        search_near_subpath = True
                        subpath_tries = 0
                        while True:
                            subpath_tries += 1
                            subpath_len = random.randint(3, len(path))
                            subpath_start_idx = random.randint(0, len(path)-subpath_len)
                            subpath = []
                            for path_idx in range(subpath_start_idx, subpath_start_idx+subpath_len):
                                subpath.append( path[path_idx] )
                            if len(subpath) != subpath_len:
                                print "ERROR SampleFree: len(subpath) != subpath_len", len(subpath), subpath_len

                            path_len = 0.0
                            for p_idx in range(len(subpath)-1):
                                path_len += CostLine(V[subpath[p_idx]], V[subpath[p_idx+1]])
#                            print "path_len", path_len
                            if path_len < 4.0:
                                break
                            if subpath_tries > 20:
                                search_near_subpath = False
                                
                        if search_near_subpath:
                            phs = ProlateHyperspheroid(len(dof_indices), V[subpath[0]], V[subpath[-1]], CostLine(V[subpath[0]], V[subpath[-1]]))
                            phs.setTransverseDiameter(path_len)

                tries = 0
                while True:
                    tries += 1
                    if search_near_subpath:
                        sphere = phs.uniformInBall(1.0, num_dof)
                        # Transform to the PHS
                        vec = phs.transform(sphere)
                        q_rand = np.empty(num_dof)
                        wrong_config = False
                        for i in range(num_dof):
                            q_rand[i] = vec[i]
                            if q_rand[i] < dof_limits[i][0] or q_rand[i] > dof_limits[i][1]:
                                wrong_config = True
                                break
                        if wrong_config:
                            continue
                    else:
                        if shortest_path_len == None:
                            q_rand = np.empty(num_dof)
                            for i in range(num_dof):
                                q_rand[i] = random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01)
                        else:
                            q_rand = uniformInBall2(shortest_path_len, dof_limits, start_q)

                    if isStateValid(q_rand, dof_indices):
                        return q_rand

        def Nearest(V, q):
                q_near = None
                for vi in V:
                    dist = Distance(q, V[vi])
                    if q_near == None or dist < q_near[0]:
                        q_near = (dist, vi)
                return q_near[1]

        def Steer(q_nearest, q_rand):
                dist = Distance(q_nearest, q_rand)
                if dist > ETA:
                    factor = ETA / dist
                else:
                    factor = 1.0
                q_diff = q_rand - q_nearest
                q_new = q_nearest + q_diff * factor
                return q_new

        def Near(V, q, near_dist):
                result = []
                for vi in V:
                    if Distance(q, V[vi]) < near_dist:
                        result.append(vi)
                return result

        def Cost(V, E, q_idx):
                if not q_idx in E:
                    return 0.0
                parent_idx = E[q_idx]
                if not parent_idx in V:
                    print "not parent_idx in V: %s"%(parent_idx)
                if not q_idx in V:
                    print "not q_idx in V: %s"%(q_idx)
                cost = CostLine(V[parent_idx], V[q_idx]) + Cost(V, E, parent_idx)
                return cost

        def CollisionFree(q1, q2, dof_indices):
                dist = max(abs(q1-q2))
#                dist = Distance(q1,q2)
                steps = int(dist / (10.0/180.0*math.pi))
                if steps < 2:
                    steps = 2
                for i in range(1, steps):
                    t = float(i)/float(steps-1)
                    current_q = q1 * (1.0-t) + q2 * t
                    if not isStateValid(current_q, dof_indices):
                        return False
                return True

        queue_slave.put( ("init_complete",) )

        while True:
            msg = queue_master.get()
            cmd = msg[0]
            if cmd == "exit":
                break
            elif cmd == "isStateValid":
                q = msg[1]
                dof_indices = msg[2]
                openrave.switchCollisionModel("velmasimplified1")
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                report1 = CollisionReport()
                report2 = CollisionReport()
                if openrave.env.CheckCollision(openrave.robot_rave, report2) or openrave.robot_rave.CheckSelfCollision(report1):
                    queue_slave.put( ("isStateValid", False, q) )
                else:
                    queue_slave.put( ("isStateValid", True, q) )

            elif cmd == "addFirstNode":
                q_start_all = msg[1]
                V_update_q_new = taskrrt.getActiveDOF(q_start_all)
                queue_slave.put( ("addFirstNode", V_update_q_new, taskrrt.dof_names) )

            elif cmd == "addNode":
#              try:
                collision_checks[0] = 0
                V, E, shortest_path_len, best_q_idx, goal_V_ids = msg[1:]
#                q_start = taskrrt.getActiveDOF(q_start_all)

                if random.uniform(0,1) < 0.05:
                    for i in range(10):
                        q_rand = taskrrt.SampleGoal(V[0], shortest_path_len)
                        if q_rand == None:
                            continue
                        if shortest_path_len != None and CostLine(V[0], q_rand) > shortest_path_len:
                            q_rand = None
                            continue
                        if not isStateValid(q_rand, taskrrt.dof_indices):
                            print "goal in collision"
                            q_rand = None
                            continue
                        break
                    if q_rand == None:
                        q_rand = SampleFree(taskrrt.dof_indices, taskrrt.dof_limits, V[0], shortest_path_len, best_q_idx, goal_V_ids, V, E)
                    else:
                        print "SampleGoal"
                else:
                    q_rand = SampleFree(taskrrt.dof_indices, taskrrt.dof_limits, V[0], shortest_path_len, best_q_idx, goal_V_ids, V, E)
                q_nearest_idx = Nearest(V, q_rand)
                q_nearest = V[q_nearest_idx]
                q_new = Steer(q_nearest, q_rand)

                if shortest_path_len != None and CostLine(V[0], q_new) > shortest_path_len:
                    continue

                col_free = CollisionFree(q_nearest, q_new, taskrrt.dof_indices)
                if col_free:

                    near_dist = 120.0/180.0*math.pi
                    q_near_idx_list = Near(V, q_new, near_dist)

                    # sort the neighbours
                    cost_q_near_idx_list = []
                    q_min_idx = q_nearest_idx
                    c_min = Cost(V, E, q_nearest_idx) + CostLine(q_nearest, q_new)
                    for q_near_idx in q_near_idx_list:
                        if q_nearest_idx == q_near_idx:
                            continue
                        q_near = V[q_near_idx]
                        new_cost = Cost(V, E, q_near_idx) + CostLine(q_near, q_new)
                        if new_cost > c_min:
                            continue
                        cost_q_near_idx_list.append( (new_cost, q_near_idx) ) 

                    sorted_cost_q_near_idx_list = sorted(cost_q_near_idx_list, key=operator.itemgetter(0))

                    collision_checked = {}
                    for (new_cost, q_near_idx) in sorted_cost_q_near_idx_list:
                        q_near = V[q_near_idx]
                        if CollisionFree(q_near, q_new, taskrrt.dof_indices):
                            collision_checked[q_near_idx] = True
                            q_min_idx = q_near_idx
                            c_min = new_cost
                            break
                        else:
                            collision_checked[q_near_idx] = False

                    q_new_idx = 1000000
                    V[q_new_idx] = q_new
                    V_update_q_new = q_new

                    E[q_new_idx] = q_min_idx
                    E_update = []
                    E_update.append( (-1, q_min_idx) )

                    cost_q_new = Cost(V, E, q_new_idx)
                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if cost_q_new + CostLine(q_new, q_near) < Cost(V, E, q_near_idx):
                            if q_near_idx in collision_checked:
                                col = collision_checked[q_near_idx]
                            else:
                                col = CollisionFree(q_new, q_near, taskrrt.dof_indices)
                            if col:
                                q_parent_idx = E[q_near_idx]
                                print "rem: %s  %s"%(q_parent_idx, q_near_idx)
                                E[q_near_idx] = q_new_idx
                                E_update.append( (q_near_idx, -1) )

                    goal = taskrrt.checkGoal(q_new)
                    if goal:
                        print "found goal"

                    queue_slave.put( ("addNode", V_update_q_new, E_update, goal, collision_checks[0]) )
                else:
                    queue_slave.put( ("addNode", None, None, None, collision_checks[0]) )
#              except:
#                print "exception in process", process_id
#                queue_slave.put( ("addNode", None, None, None, collision_checks[0]) )

    def spin(self):

        simulation = True

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        rospy.sleep(0.5)
        self.velma.updateTransformations()

        if simulation:
            hv = [3.2, 3.2, 3.2, 3.2]
        ht = [3000, 3000, 3000, 3000]
        self.velma.moveHandLeft([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)
        self.velma.moveHandRight([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)

        rospy.sleep(1.0)

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/key/vis_test.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        self.num_proc = 3
        self.proc = []
        self.queue_master = Queue(maxsize=self.num_proc)
        self.queue_slave = Queue()
        for i in range(self.num_proc):
#            self.queue_master.append( Queue() )
#            self.queue_slave.append( Queue() )
            self.proc.append( Process(target=self.openraveWorker, args=(i, env_file, xacro_uri, srdf_path, self.velma.js_pos, self.queue_master, self.queue_slave,)) )
            self.proc[-1].start()

        # receive n messages
        for i in range(self.num_proc):
            result = self.queue_slave.get()

        print "all processes initalized"
#        for i in range(self.num_proc):
#            self.proc[i].join()
#        return

        #
        # Initialise Openrave
        #

#        a1 = np.array([1,2,3])
#        a2 = np.array([3,2,1])
#        print a1*3
#        exit(0)

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file)
#        openrave.readRobot(xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro', srdf_uri=rospack.get_path('velma_description') + '/robots/velma_simplified.srdf')
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

        target_link = "head_tilt_link"
        l_idx1 = openrave.robot_rave.GetLink(target_link).GetIndex()
        l_idx2 = openrave.robot_rave.GetLink("torso_base").GetIndex()
        joints = openrave.robot_rave.GetChain(l_idx1, l_idx2)
        active_joints = []
        for joint in joints:
            if joint.GetDOFIndex() >= 0:
                active_joints.append(joint.GetName())
        print "active_joints", active_joints

        root_joints = []
        for joint in openrave.robot_rave.GetJoints():
            if joint.GetName() in active_joints:
                continue
            idx1 = joint.GetHierarchyParentLink().GetIndex()
            for ac_joint_name in active_joints:
                ac_joint = openrave.robot_rave.GetJoint(ac_joint_name)
                idx2 = ac_joint.GetHierarchyChildLink().GetIndex()
                chain = openrave.robot_rave.GetChain(idx1, idx2)
                all_passive = True
                for chain_jnt in chain:
                    if chain_jnt.GetDOFIndex() > 0:
                        all_passive = False
                        break
                if all_passive and not joint.GetName() in root_joints:
                    root_joints.append(joint.GetName())

        print "root_joints", root_joints

        disabled_links = []
        for joint_name in root_joints:
            joint = openrave.robot_rave.GetJoint(joint_name)
            for link in openrave.robot_rave.GetLinks():
                if openrave.robot_rave.DoesAffect(joint.GetJointIndex(), link.GetIndex()) != 0:
                    disabled_links.append(link.GetName())

#        for link in openrave.robot_rave.GetLinks():
#            if link.GetName() in disabled_links:
#                link.SetVisible(False)
#                link.Enable(False)
#            else:
#                link.SetVisible(True)
#                link.Enable(True)

#        raw_input("Press ENTER to continue...")

#        exit(0)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))


#        T_W_T = self.velma.T_W_E * PyKDL.Frame(PyKDL.Vector(0,0,0.17))
#        print T_W_T.M.GetQuaternion()
#        print T_W_T.p
#        exit(0)

        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        # TEST: key hole goal
        if False:
            task = KeyRotTaskRRT(openrave)
            task.SampleGoal(None, None)
            exit(0)


        # TEST: head IK
        if False:
            v_rot = 0.800
            v_lean = 0.375
            v_head = 0.392
            h_cam = 0.0
            v_cam = 0.225
            head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)
            openrave.addSphere("target_sphere", 0.05)

            while not rospy.is_shutdown():
                head_kin.UpdateTorsoPose(openrave.robot_rave.GetJoint("torso_0_joint").GetValue(0), openrave.robot_rave.GetJoint("torso_1_joint").GetValue(0))
                target_pos = PyKDL.Vector(1, random.uniform(-1.5, 1.5), random.uniform(0,2))
                head_kin.UpdateTargetPosition(target_pos.x(), target_pos.y(), target_pos.z())
                openrave.updatePose("target_sphere", PyKDL.Frame(target_pos))
                head_kin.TransformTargetToHeadFrame()
                joint_pan, joint_tilt = head_kin.CalculateHeadPose()
                if joint_pan == None:
                    continue

                openrave.robot_rave.SetDOFValues([joint_pan, joint_tilt], [openrave.robot_rave.GetJoint("head_pan_joint").GetDOFIndex(), openrave.robot_rave.GetJoint("head_tilt_joint").GetDOFIndex()])

                raw_input("Press ENTER to continue...")
            exit(0)

#        report1 = CollisionReport()
# 	print "self-collision:", openrave.robot_rave.CheckSelfCollision(report1)
#        print report1.plink1
#        print report1.plink2

#        exit(0)
        raw_input("Press ENTER to continue...")
        
        self.planVis(openrave)

        raw_input("Press ENTER to exit...")
        exit(0)


        rospy.sleep(1)
        openrave.runOctomap()

        sphere = RaveCreateKinBody(openrave.env,'')
        sphere.SetName("sphere")
        sphere.InitFromSpheres(numpy.array([[0,0,0,0.05]]),True)
        openrave.env.Add(sphere,True)

        # test the collision checker for octomap
        if True:
            raw_input("Press ENTER to continue...")

            ob = openrave.env.GetKinBody("_OCTOMAP_MAP_")
            cc = openrave.env.GetCollisionChecker()

            m_id = 0
            for x in np.linspace(0,1.5,30):
                for y in np.linspace(-1,1,40):
                    for z in np.linspace(1,2,20):
#                        print x,y,z
                        tr = KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,y,z)))
                        sphere.SetTransform(tr)
                        openrave.env.UpdatePublishedBodies()
                        report = CollisionReport()
                        ret = cc.CheckCollision(sphere, report)
#                        ret = openrave.env.CheckCollision(ob, report)
#                        print ret
                        if ret:
                            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)

                        continue
                        if report.plink1 == None:
                            print None
                        else:
                            print report.plink1.GetParent().GetName(), report.plink2.GetName() 
#                            print "   ", report.vLinkColliding
                            for link1, link2 in report.vLinkColliding:
                                print "   ", link1.GetParent().GetName(), link2.GetName()
#                            print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 

        exit(0)

        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)

        raw_input("Press ENTER to look around...")

#        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-0.5,1))))
#        raw_input("Press ENTER to exit...")

#        exit(0)
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))



        raw_input(".")

        exit(0)

if __name__ == '__main__':

    rospy.init_node('test_or_octomap')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


