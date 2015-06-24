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
        m_id = 0

        if debug:
            self.pub_marker.eraseMarkers(0,3000, frame_id='world')
            rospy.sleep(0.01)

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
        def RRTstar(openrave, dof_names):
            dof_indices = []
            dof_limits = []
            for joint_name in dof_names:
                joint = openrave.robot_rave.GetJoint(joint_name)
                dof_indices.append( joint.GetDOFIndex() )
                lim_lo, lim_up = joint.GetLimits()
                dof_limits.append( (lim_lo[0], lim_up[0]) )

            print "planning for %s joints"%(len(dof_indices))

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
            q_init = openrave.robot_rave.GetDOFValues(dof_indices)
            self.q_init = np.array(q_init)
            q_new_idx = 0
            V[0] = np.array(q_init)
            total_collision_checks = 0
            for k in range(100000):
              produced = 0
              consumed = 0

              self.queue_master.put( ("addNode", V, E, dof_indices, dof_limits, dof_names, self.q_init, shortest_path_len, best_q_idx, goal_V_ids), True )
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
                            if CostLine(self.q_init, V[vi]) > shortest_path_len:
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

        RRTstar(openrave, dof_names)

    def openraveWorker(self, process_id, env_file, xacro_uri, srdf_path, configuration_ros, queue_master, queue_slave):
      openrave = openraveinstance.OpenraveInstance()
      openrave.startOpenraveURDF(env_file=env_file, viewer=False)
      openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

      openrave.updateRobotConfigurationRos(configuration_ros)
      collision_checks = [0]

      v_rot = 0.800
      v_lean = 0.375
      v_head = 0.392
      h_cam = 0.0
      v_cam = 0.225
      head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)

      with openrave.env:

        ETA = 60.0/180.0*math.pi
        gamma = 0.5


        kinect_fov = 30.0/180.0*math.pi

        # target: key pocket
        vis_targets = [
        ("vis_target_0", 0.1, PyKDL.Vector(0, -0.4, 1.0)),
        ("vis_target_1", 0.1, PyKDL.Vector(0.1, -0.4, 1.0)),
        ("vis_target_2", 0.1, PyKDL.Vector(0.1, -0.5, 1.0)),
        ("vis_target_3", 0.1, PyKDL.Vector(0, -0.5, 1.0)),
#        ("vis_target_4", 0.1, PyKDL.Vector(0.05, -0.45, 1.0)),
        ]
        head_target_B = PyKDL.Vector()
        for target in vis_targets:
            head_target_B += target[2]
        head_target_B = head_target_B / len(vis_targets)

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
            body = openrave.addSphere(name, diam)
            body.SetTransform(self.KDLToOpenrave(PyKDL.Frame(pos)))
            vis_bodies.append( body )
            openrave.env.Remove( body )

        def getVisibility(openrave, vis_bodies, q, dof_indices):
            openrave.switchCollisionModel("velmasimplified0")
            rays_hit = 0
            m_id = 0

            openrave.robot_rave.SetDOFValues(q, dof_indices)
#            openrave.env.UpdatePublishedBodies()

            for body in vis_bodies:
                openrave.env.Add( body )
            T_W_C = self.OpenraveToKDL(openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()
            cam_W = T_W_C * PyKDL.Vector()
            cam_dir_W = PyKDL.Frame(T_W_C.M) * PyKDL.Vector(0,0,0.5)

            # create rays connecting the optical frame and the target objects
            for (name, diam, pos_W) in vis_targets:
                pos_C = T_C_W * pos_W
                dir_W = pos_W - cam_W
                if pos_C.z() < 0.1:
                    continue
                if velmautils.getAngle(PyKDL.Vector(0,0,1), pos_C) > kinect_fov:
                    continue

                report = CollisionReport()
                ret = openrave.env.CheckCollision(Ray((cam_W[0], cam_W[1], cam_W[2]), (dir_W[0], dir_W[1], dir_W[2])), report)
                if ret and report.plink1 != None and report.plink1.GetParent().GetName().find("vis_target_") == 0:
                    rays_hit += 1
                else:
                    pass
            for body in vis_bodies:
                openrave.env.Remove( body )

            return rays_hit

        def checkGoal(q, dof_indices):
            return getVisibility(openrave, vis_bodies, q, dof_indices) == 4

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
                    if CostLine(q_start, vec) <= r:
#                        print tries
                        return vec

        def SampleGoal(dof_indices, dof_limits, dof_names, start_q, shortest_path_len):
            dof_indices_map = {}
            for i in range(len(dof_names)):
                dof_indices_map[dof_names[i]] = i
            q_goal = np.empty(len(dof_names))
            for tries in range(200):
                if shortest_path_len == None:
                    for i in range(len(dof_names)):
                        q_goal[i] = random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01)
                else:
                    q_goal = uniformInBall2(shortest_path_len, dof_limits, start_q)

                head_kin.UpdateTorsoPose(q_goal[dof_indices_map["torso_0_joint"]], openrave.robot_rave.GetJoint("torso_1_joint").GetValue(0))
                head_kin.UpdateTargetPosition(head_target_B.x(), head_target_B.y(), head_target_B.z())
                head_kin.TransformTargetToHeadFrame()
                joint_pan, joint_tilt = head_kin.CalculateHeadPose()
                if joint_pan == None:
                    continue
                joint_pan = max(joint_pan, dof_limits[dof_indices_map["head_pan_joint"]][0])
                joint_pan = min(joint_pan, dof_limits[dof_indices_map["head_pan_joint"]][1])
                joint_tilt = max(joint_tilt, dof_limits[dof_indices_map["head_tilt_joint"]][0])
                joint_tilt = min(joint_tilt, dof_limits[dof_indices_map["head_tilt_joint"]][1])
                q_goal[dof_indices_map["head_pan_joint"]] = joint_pan
                q_goal[dof_indices_map["head_tilt_joint"]] = joint_tilt

                if shortest_path_len != None and CostLine(start_q, q_goal) > shortest_path_len:
                    continue

                if isStateValid(q_goal, dof_indices) and checkGoal(q_goal, dof_indices):
                    return q_goal
            return None


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

            elif cmd == "addNode":
              try:
                collision_checks[0] = 0
                V, E, dof_indices, dof_limits, dof_names, q_start, shortest_path_len, best_q_idx, goal_V_ids = msg[1:]

                if random.uniform(0,1) < 0.05:
                    q_rand = SampleGoal(dof_indices, dof_limits, dof_names, q_start, shortest_path_len)
                    if q_rand == None:
                        q_rand = SampleFree(dof_indices, dof_limits, q_start, shortest_path_len, best_q_idx, goal_V_ids, V, E)
                    else:
                        print "SampleGoal"
                else:
                    q_rand = SampleFree(dof_indices, dof_limits, q_start, shortest_path_len, best_q_idx, goal_V_ids, V, E)
                q_nearest_idx = Nearest(V, q_rand)
                q_nearest = V[q_nearest_idx]
                q_new = Steer(q_nearest, q_rand)

                if shortest_path_len != None and CostLine(q_start, q_new) > shortest_path_len:
                    continue

                col_free = CollisionFree(q_nearest, q_new, dof_indices)
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
                        if CollisionFree(q_near, q_new, dof_indices):
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
                                col = CollisionFree(q_new, q_near, dof_indices)
                            if col:
                                q_parent_idx = E[q_near_idx]
                                print "rem: %s  %s"%(q_parent_idx, q_near_idx)
                                E[q_near_idx] = q_new_idx
                                E_update.append( (q_near_idx, -1) )

                    goal = checkGoal(q_new, dof_indices)
                    if goal:
                        print "found goal"

                    queue_slave.put( ("addNode", V_update_q_new, E_update, goal, collision_checks[0]) )
                else:
                    if process_id == 0:
                        print "exit"
                    queue_slave.put( ("addNode", None, None, None, collision_checks[0]) )
              except:
                print "exception in process", process_id
                queue_slave.put( ("addNode", None, None, None, collision_checks[0]) )

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


