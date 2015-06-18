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
import rosparam

from multiprocessing import Process, Queue

class TestOrOctomap:
    """

"""

    def KDLToOpenrave(self, T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

    def OpenraveToKDL(self, T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def inputThread(self, arg):
        raw_input("Press ENTER to stop planning...")
        self.stop_planning = True

    def planVis(self, openrave):
      with openrave.env:
        debug = True
        update_published_bodies = False
        m_id = 0

        if debug:
            self.pub_marker.eraseMarkers(0,3000, frame_id='world')
            rospy.sleep(0.01)

        kinect_fov = 30.0/180.0*math.pi

        # target: key pocket
        vis_targets = [
        ("vis_target_0", 0.1, PyKDL.Vector(0, -0.4, 1.0)),
        ("vis_target_1", 0.1, PyKDL.Vector(0.1, -0.4, 1.0)),
        ("vis_target_2", 0.1, PyKDL.Vector(0.1, -0.5, 1.0)),
        ("vis_target_3", 0.1, PyKDL.Vector(0, -0.5, 1.0)),
#        ("vis_target_4", 0.1, PyKDL.Vector(0.05, -0.45, 1.0)),
        ]

        vis_bodies = []
        # target: test (vertical axis at the door plane)
#        vis_targets = [
#        ("vis_target_0", 0.1, PyKDL.Vector(1, 0.0, 1.2)),
#        ("vis_target_1", 0.1, PyKDL.Vector(1, 0.0, 1.3)),
#        ("vis_target_2", 0.1, PyKDL.Vector(1, 0.0, 1.4)),
#        ("vis_target_3", 0.1, PyKDL.Vector(1, 0.0, 1.5)),
#        ("vis_target_4", 0.1, PyKDL.Vector(1, 0.0, 1.6)),
#        ]

        for (name, diam, pos) in vis_targets:
            if debug:
                m_id = self.pub_marker.publishSinglePointMarker(pos, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(diam, diam, diam), T=None)
                rospy.sleep(0.01)
            body = openrave.addSphere(name, diam)
            body.SetTransform(self.KDLToOpenrave(PyKDL.Frame(pos)))
            vis_bodies.append( body )
            openrave.env.Remove( body )

        def getVisibility(openrave, vis_bodies, q=None, dof_indices=None):
            openrave.switchCollisionModel("velmasimplified0")
            rays_hit = 0
            m_id = 0

            if q != None and dof_indices != None:
                current_q = openrave.robot_rave.GetDOFValues(dof_indices)
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                if update_published_bodies:
                    openrave.env.UpdatePublishedBodies()

            for body in vis_bodies:
                openrave.env.Add( body )
            T_W_C = self.OpenraveToKDL(openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()
            cam_W = T_W_C * PyKDL.Vector()
            cam_dir_W = PyKDL.Frame(T_W_C.M) * PyKDL.Vector(0,0,0.5)
            if debug:
                m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+cam_dir_W, m_id, 1, 1, 1, frame='world', namespace='kinect_head_rays', scale=0.01)

            # create rays connecting the optical frame and the target objects
            for (name, diam, pos_W) in vis_targets:
                pos_C = T_C_W * pos_W
                dir_W = pos_W - cam_W
                if pos_C.z() < 0.1:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 0, 1, frame='world', namespace='kinect_head_rays', scale=0.01)
                    continue
                if velmautils.getAngle(PyKDL.Vector(0,0,1), pos_C) > kinect_fov:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 0, 1, frame='world', namespace='kinect_head_rays', scale=0.01)
                    continue

                report = CollisionReport()
                ret = openrave.env.CheckCollision(Ray((cam_W[0], cam_W[1], cam_W[2]), (dir_W[0], dir_W[1], dir_W[2])), report)
                if ret and report.plink1 != None and report.plink1.GetParent().GetName().find("vis_target_") == 0:
                    rays_hit += 1
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 1, 0, frame='world', namespace='kinect_head_rays', scale=0.01)
                else:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 1, 0, 0, frame='world', namespace='kinect_head_rays', scale=0.01)

            for body in vis_bodies:
                openrave.env.Remove( body )

            if q != None and dof_indices != None:
                openrave.robot_rave.SetDOFValues(current_q, dof_indices)
                if update_published_bodies:
                    openrave.env.UpdatePublishedBodies()

            return rays_hit

        def checkGoal(q, dof_indices):
            return getVisibility(openrave, vis_bodies, q, dof_indices) == 4

#        def isStateValid(openrave, q, dof_indices):
#            openrave.switchCollisionModel("velmanohands")
#            is_valid = True
#            current_q = openrave.robot_rave.GetDOFValues(dof_indices)
#            openrave.robot_rave.SetDOFValues(q, dof_indices)
#            if update_published_bodies:
#                openrave.env.UpdatePublishedBodies()
#            report1 = CollisionReport()
#            report2 = CollisionReport()
# 	    if openrave.env.CheckCollision(openrave.robot_rave, report2) or openrave.robot_rave.CheckSelfCollision(report1):
#                is_valid = False
#            openrave.robot_rave.SetDOFValues(current_q, dof_indices)
#            if update_published_bodies:
#                openrave.env.UpdatePublishedBodies()
#            return is_valid

        def isStateValidPut(q, dof_indices):
            self.queue_master.put( ("isStateValid", q, dof_indices) )

        def isStateValidGetWait():
            resp = self.queue_slave.get()
            if resp[0] != "isStateValid":
                print "ERROR: isStateValidGetWait", resp
                exit(0)
            return resp[1], resp[2]

        def isStateValidGetNoWait():
            try:
                resp = self.queue_slave.get(False)
            except:
                return None, None
            if resp[0] != "isStateValid":
                print "ERROR: isStateValidGetNoWait", resp
                exit(0)
            return resp[1], resp[2]

#            self.num_proc = 3
#            self.proc = []



        dof_names = [
        "head_pan_joint",
        "head_tilt_joint",
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
#        "right_arm_4_joint",
#        "right_arm_5_joint",
#        "right_arm_6_joint",
        "torso_0_joint",
#        "torso_1_joint",
        ]

        #
        # RRT*
        #
        def RRTstar(openrave, dof_names, checkGoalFn):
            dof_indices = []
            dof_limits = []
            for joint_name in dof_names:
                joint = openrave.robot_rave.GetJoint(joint_name)
                dof_indices.append( joint.GetDOFIndex() )
                lim_lo, lim_up = joint.GetLimits()
                dof_limits.append( (lim_lo[0], lim_up[0]) )

            print "planning for %s joints"%(len(dof_indices))

            ETA = 60.0/180.0*math.pi
            gamma = 0.5
            d = len(dof_indices)
            
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
                cost = Distance(q1, q2) * (np.linalg.norm(q1-self.q_init) + np.linalg.norm(q2-self.q_init)) * 0.5
                return cost

            def uniformInBall2(r, limits):
                tries = 0
                while True:
                    tries += 1
                    n = len(limits)
                    vec = np.empty(n)
                    for i in range(n):
                        lo_limit = max(limits[i][0], self.q_init[i]-math.sqrt(r * 2.0))
                        up_limit = min(limits[i][1], self.q_init[i]+math.sqrt(r * 2.0))
                        vec[i] = random.uniform(lo_limit, up_limit)
                    if CostLine(self.q_init, vec) <= r:
#                        print tries
                        return vec

            def SampleFree(openrave, dof_indices, dof_limits, best_q, start_q, shortest_path_len, best_q_idx, V, E):
                search_near_path = False

                random_0_1 = random.uniform(0,1)
                if best_q_idx != None and random_0_1 < 0.05:
                    path = GetPath(E, best_q_idx)
                    search_near_path = True

                produced = 0
                consumed = 0
#                procs = []
#                for i in range(self.num_proc):
#                    procs.append(False)
                tries = 0
                found = False
                found_q = None
                while not found:
                    # put into pool
#                    for proc_id in range(self.num_proc):
#                        if procs[proc_id] == False:
                    tries += 1
                    if search_near_path:
                        q_rand_list = []
                        p_idx = random.randint(0, len(path)-1)
                        search_near_q = V[path[p_idx]]
                        for i in range(len(dof_limits)):
                            lim_lo = max(dof_limits[i][0], search_near_q[i]-10.0/180.0*math.pi)
                            lim_up = min(dof_limits[i][1], search_near_q[i]+10.0/180.0*math.pi)
                            q_rand_list.append( random.uniform(lim_lo, lim_up) )
                        q_rand = np.array(q_rand_list)
                    else:
                        if shortest_path_len == None:
                            q_rand_list = []
                            for i in range(len(dof_limits)):
                                q_rand_list.append( random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01) )
                            q_rand = np.array(q_rand_list)
                        else:
                            q_rand = uniformInBall2(shortest_path_len, dof_limits)

#                    print "SampleFree: put into pool", proc_id
                    isStateValidPut(q_rand, dof_indices)
                    produced += 1
#                    procs[proc_id] = True
#                    break
                    # get from pool
                    for proc_id in range(self.num_proc):
                        valid, q = isStateValidGetNoWait()
                        if valid != None:
                            consumed += 1
                            if valid == True:
                                found = True
                                found_q = q
                                break
                        else:
                            break
                for i in range(produced-consumed):
                    isStateValidGetWait()
                return found_q

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

            def CollisionFree(openrave, q1, q2, dof_indices):
                dist = max(abs(q1-q2))
                steps = int(dist / (10.0/180.0*math.pi))
                if steps < 2:
                    steps = 2
                produced = 0
                consumed = 0

                colliding = False
                i = 1
                while i < steps and not colliding:
                    # put into pool
                    t = float(i)/float(steps-1)
                    current_q = q1 * (1.0-t) + q2 * t
                    isStateValidPut(current_q, dof_indices)
                    produced += 1
                    i += 1

                    # get from pool
                    for proc_id in range(self.num_proc):
                        valid, q = isStateValidGetNoWait()
                        if valid != None:
                            consumed += 1
                            if valid == False:
                                colliding = True
                                break
                        else:
                            break

                for i in range(produced - consumed):
                    valid, q = isStateValidGetWait()
                    if valid == False:
                        colliding = True

                return not colliding

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
            q_init = openrave.robot_rave.GetDOFValues(dof_indices)
            self.q_init = np.array(q_init)
            q_new_idx = 0
            V[0] = np.array(q_init)
            for k in range(100000):
                time = []
                time.append( rospy.Time.now() )
                q_rand = SampleFree(openrave, dof_indices, dof_limits, best_q, q_init, shortest_path_len, best_q_idx, V, E)
                time.append( rospy.Time.now() )
                q_nearest_idx = Nearest(V, q_rand)
                time.append( rospy.Time.now() )
                q_nearest = V[q_nearest_idx]
                q_new = Steer(q_nearest, q_rand)
                col_free = CollisionFree(openrave, q_nearest, q_new, dof_indices)
#                q_new = CollisionFree(openrave, q_nearest, q_new, dof_indices, return_closest_q=True)
#                col_free = (q_new != None)
#                col_free = isStateValid(openrave, q_new, dof_indices)  # only for small ETA
                time.append( rospy.Time.now() )
                if col_free:

                    if shortest_path_len != None and CostLine(self.q_init, q_new) > shortest_path_len:
                        continue

                    near_dist = 120.0/180.0*math.pi
#                    near_dist = gamma*math.log(len(V))/len(V)
#min(gamma*math.pow(math.log(len(V))/len(V), 1.0/d), ETA)
#                    print k, near_dist
#                    near_dist = min(gamma*math.log(len(V))/len(V), ETA)
                    q_near_idx_list = Near(V, q_new, near_dist)
#                    print "q_near_idx_list", len(q_near_idx_list)

                    time.append( rospy.Time.now() )

                    # sort the neighbours
                    cost_q_near_idx_list = []
                    q_min_idx = q_nearest_idx
                    c_min = Cost(V, E, q_nearest_idx) + CostLine(q_nearest, q_new)
                    for q_near_idx in q_near_idx_list:
                        if q_nearest_idx == q_near_idx:
                            continue
                        q_near = V[q_near_idx]
                        cost_q_near_idx_list.append( (Cost(V, E, q_near_idx) + CostLine(q_near, q_new), q_near_idx) ) 

                    sorted_cost_q_near_idx_list = sorted(cost_q_near_idx_list, key=operator.itemgetter(0))

                    time.append( rospy.Time.now() )

                    for (new_cost, q_near_idx) in sorted_cost_q_near_idx_list:
                        q_near = V[q_near_idx]
                        if CollisionFree(openrave, q_near, q_new, dof_indices):
                            q_min_idx = q_near_idx
                            c_min = new_cost
                            break

                    time.append( rospy.Time.now() )

                    q_new_idx += 1
                    V[q_new_idx] = q_new
                    E[q_new_idx] = q_min_idx
                    print "len(V) len(neighbours) len(goal_V_ids)", len(V), len(q_near_idx_list), len(goal_V_ids)

                    if debug:
                        edge_ids[(q_min_idx, q_new_idx)] = edge_id
                        self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_min_idx][0], V[q_min_idx][1], V[q_min_idx][2]), PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), edge_id, 0, 1, 0, frame='world', namespace='edges', scale=0.01)
                        edge_id += 1

                    time.append( rospy.Time.now() )

                    cost_q_new = Cost(V, E, q_new_idx)
                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if cost_q_new + CostLine(q_new, q_near) < Cost(V, E, q_near_idx) and CollisionFree(openrave, q_new, q_near, dof_indices):
                            # Parent()
                            q_parent_idx = E[q_near_idx]
                            print "rem: %s  %s"%(q_parent_idx, q_near_idx)

                            edge_id_del = None
                            if debug:
                                edge_id_del = edge_ids[(q_parent_idx, q_near_idx)]
                            E[q_near_idx] = q_new_idx
                            cost_q_new = Cost(V, E, q_new_idx)
                            if debug:
                                edge_ids[(q_new_idx, q_near_idx)] = edge_id_del
                                self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), PyKDL.Vector(V[q_near_idx][0], V[q_near_idx][1], V[q_near_idx][2]), edge_id_del, 0, 1, 0, frame='world', namespace='edges', scale=0.01)

                    time.append( rospy.Time.now() )

                    goal = checkGoalFn(q_new, dof_indices)

                    if goal:
                        goal_V_ids.append(q_new_idx)

#                        if shortest_path_len == None or shortest_path_len > Cost(V, E, q_new_idx):
#                            best_q = q_new
#                            best_q_idx = q_new_idx
#                            shortest_path_len_old = shortest_path_len
#                            shortest_path_len = Cost(V, E, q_new_idx)
#
#                            self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
#                            DrawPath(V, E, q_new_idx)
#                            if shortest_path_len_old == None:
#                                shortest_path_len_old = shortest_path_len
#                            print " %s  , shortest_path: %s   delta: %s"%(k, shortest_path_len, shortest_path_len_old - shortest_path_len)

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
                            if CostLine(self.q_init, V[vi]) > shortest_path_len:
                                rem_nodes.append(vi)
                        print "removing nodes:", len(rem_nodes)
                        for vi in rem_nodes:
                            del V[vi]
                            self.pub_marker.eraseMarkers(edge_ids[(E[vi], vi)], edge_ids[(E[vi], vi)]+1, frame_id='torso_base', namespace='edges')
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
                              self.pub_marker.eraseMarkers(edge_ids[(E[vi], vi)], edge_ids[(E[vi], vi)]+1, frame_id='torso_base', namespace='edges')
                              del E[vi]
                              if vi in goal_V_ids:
                                  goal_V_ids.remove(vi)


#                    if shortest_path_len != None and shortest_path_len < 3.2:
#                        break
                    if self.stop_planning:
                        break
                    time.append( rospy.Time.now() )

#                    t_diff = []
#                    for ti in range(len(time)-1):
#                        t_diff.append( (time[ti+1] - time[ti]).to_sec() )
#                    print "noncol", t_diff
#                else:
#                    t_diff = []
#                    for ti in range(len(time)-1):
#                        t_diff.append( (time[ti+1] - time[ti]).to_sec() )
#                    print "col   ", t_diff

            path = GetPath(E, best_q_idx)
            print path

            traj = []
#            for q_idx in path:
#                traj.append(V[q_idx])
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

        RRTstar(openrave, dof_names, checkGoal)

    def openraveWorker(self, process_id, env_file, xacro_uri, srdf_path, queue_master, queue_slave):
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file, viewer=False)
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

        queue_slave.put( ("init_complete",) )

        while True:
            msg = queue_master.get()
            cmd = msg[0]
#            print "slave", process_id, cmd
            if cmd == "exit":
                break
            elif cmd == "isStateValid":
                q = msg[1]
                dof_indices = msg[2]
                openrave.switchCollisionModel("velmanohands")
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                report1 = CollisionReport()
                report2 = CollisionReport()
                if openrave.env.CheckCollision(openrave.robot_rave, report2) or openrave.robot_rave.CheckSelfCollision(report1):
                    queue_slave.put( ("isStateValid", False, q) )
                else:
                    queue_slave.put( ("isStateValid", True, q) )


    def spin(self):

        simulation = True

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/key/vis_test.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        self.num_proc = 1
        self.proc = []
        self.queue_master = Queue(maxsize=self.num_proc)
        self.queue_slave = Queue()
        for i in range(self.num_proc):
#            self.queue_master.append( Queue() )
#            self.queue_slave.append( Queue() )
            self.proc.append( Process(target=self.openraveWorker, args=(i, env_file, xacro_uri, srdf_path, self.queue_master, self.queue_slave,)) )
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

        target_link = "right_HandPalmLink"#head_tilt_link"
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

#        T_W_T = self.velma.T_W_E * PyKDL.Frame(PyKDL.Vector(0,0,0.17))
#        print T_W_T.M.GetQuaternion()
#        print T_W_T.p
#        exit(0)

        openrave.updateRobotConfigurationRos(self.velma.js_pos)


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
                        tr = self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,y,z)))
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


