# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
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
#
# Author: Dawid Seredynski
#

import rospy
import PyKDL
import numpy as np
from multiprocessing import Process, Queue
import math
import copy
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import openraveinstance
import velmautils
import tree
from prolatehyperspheroid import ProlateHyperspheroid
import objectstate

import random
from openravepy import *
import operator

class PlannerRRT:

    def openraveWorker(self, process_id, env_file, obj_filenames, srdf_path, queue_master, queue_master_special, queue_slave):
      openrave = openraveinstance.OpenraveInstance()
      openrave.startOpenrave(viewer=False, collision='fcl')
      openrave.loadEnv(env_file)
      openrave.readRobot(srdf_path=srdf_path)
      openrave.runOctomapClient()
      for filename in obj_filenames:
          body = openrave.env.ReadKinBodyXMLFile(filename)
          body.Enable(False)
          body.SetVisible(False)
          openrave.env.Add(body)

      mo_state = objectstate.MovableObjectsState(openrave.env, obj_filenames)

      joints_max_step = {
      "head_pan_joint" : 10.0/180.0*math.pi,
      "head_tilt_joint" : 10.0/180.0*math.pi,
      "left_HandFingerOneKnuckleOneJoint" : 5.0/180.0*math.pi,
      "left_HandFingerOneKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "left_HandFingerThreeKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "left_HandFingerTwoKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "left_arm_0_joint" : 2.0/180.0*math.pi,
      "left_arm_1_joint" : 2.0/180.0*math.pi,
      "left_arm_2_joint" : 2.0/180.0*math.pi,
      "left_arm_3_joint" : 3.0/180.0*math.pi,
      "left_arm_4_joint" : 7.0/180.0*math.pi,
      "left_arm_5_joint" : 7.0/180.0*math.pi,
      "left_arm_6_joint" : 7.0/180.0*math.pi,
      "right_HandFingerOneKnuckleOneJoint" : 5.0/180.0*math.pi,
      "right_HandFingerOneKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "right_HandFingerThreeKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "right_HandFingerTwoKnuckleTwoJoint" : 5.0/180.0*math.pi,
      "right_arm_0_joint" : 2.0/180.0*math.pi,
      "right_arm_1_joint" : 2.0/180.0*math.pi,
      "right_arm_2_joint" : 2.0/180.0*math.pi,
      "right_arm_3_joint" : 3.0/180.0*math.pi,
      "right_arm_4_joint" : 7.0/180.0*math.pi,
      "right_arm_5_joint" : 7.0/180.0*math.pi,
      "right_arm_6_joint" : 7.0/180.0*math.pi,
      "torso_0_joint" : 1.0/180.0*math.pi,
      "torso_1_joint" : 1.0/180.0*math.pi,
      }

      with openrave.env:

        ETA = 60.0/180.0*math.pi
#        ETA = 30.0/180.0*math.pi

        def isStateValid(q, dof_indices):
            openrave.switchCollisionModel("velmasimplified1")
            current_q = openrave.robot_rave.GetDOFValues(dof_indices)
            openrave.robot_rave.SetDOFValues(q, dof_indices)
            report = CollisionReport()
            if openrave.env.CheckCollision(openrave.robot_rave, report):
                return False
            report = CollisionReport()
            if openrave.robot_rave.CheckSelfCollision(report):
                return False
            return True

        def CollisionFree(q1, q2, dof_indices):
            q_diff = abs(q2-q1)
            max_steps = 0
            for q_idx in range(len(q_diff)):
                q_steps = int(q_diff[q_idx] / dof_max_step[q_idx])
                if q_steps > max_steps:
                    max_steps = q_steps

            steps = max_steps
            if steps < 2:
                steps = 2
            for i in range(1, steps):
                t = float(i)/float(steps-1)
                current_q = q1 * (1.0-t) + q2 * t
                if not isStateValid(current_q, dof_indices):
                    return False
            return True

        def SampleFree(dof_indices, dof_limits, start_q, shortest_path_len, best_q_idx, goal_V_ids, V, E):
                num_dof = len(dof_limits)
                search_near_subpath = False

                random_0_1 = random.uniform(0,1)
                if len(goal_V_ids) > 0 and random_0_1 < 0.05:
                    path = tree.GetPath(E, best_q_idx)
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
                                path_len += tree.CostLine(V[subpath[p_idx]], V[subpath[p_idx+1]])
                            if path_len < 4.0:
                                break
                            if subpath_tries > 20:
                                search_near_subpath = False
                                
                        if search_near_subpath:
                            phs = ProlateHyperspheroid(len(dof_indices), V[subpath[0]], V[subpath[-1]], tree.CostLine(V[subpath[0]], V[subpath[-1]]))
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
                            q_rand = tree.uniformInBall(shortest_path_len, dof_limits, start_q)

                    if isStateValid(q_rand, dof_indices):
                        return q_rand

        def Nearest(V, q):
                q_near = None
                for vi in V:
                    dist = tree.Distance(q, V[vi])
                    if q_near == None or dist < q_near[0]:
                        q_near = (dist, vi)
                return q_near[1]

        def Steer(q_nearest, q_rand):
                dist = tree.Distance(q_nearest, q_rand)
                reached = False
                if dist > ETA:
                    factor = ETA / dist
                    reached = False
                else:
                    factor = 1.0
                    reached = True

                q_diff = q_rand - q_nearest
                q_new = q_nearest + q_diff * factor
                return q_new, reached

        def Near(V, q, near_dist):
                result = []
                for vi in V:
                    if tree.Distance(q, V[vi]) < near_dist:
                        result.append(vi)
                return result

        def Extend(V, E, q, q_new_idx):
            q_nearest_idx = Nearest(V, q)
            q_nearest = V[q_nearest_idx]
            q_new, reached = Steer(q_nearest, q)
            col_free = CollisionFree(q_nearest, q_new, taskrrt.GetDofIndices())
            update = None
            if col_free:
                if reached:
                    status = "reached"
                else:
                    status = "advanced"
                V[q_new_idx] = q_new
                V_update_q_new = q_new
                E[q_new_idx] = q_nearest_idx
                E_update = q_nearest_idx
                update = (V_update_q_new, q_new_idx, E_update)
                q_new_idx += 1
            else:
                status = "trapped"
            return status, update, q_new_idx

        def Connect(V, E, q, q_new_idx):
            updates = []
            while True:
                status, update, q_new_idx = Extend(V, E, q, q_new_idx)
                if update != None:
                    updates.append(update)
                if status != "advanced":
                    break
            return status, updates, q_new_idx

        def applyStar(V, E, q_new_idx):
                        q_new = V[q_new_idx]
                        q_near_idx_list = Near(V, q_new, near_dist)
                        q_nearest_idx = E[q_new_idx]
                        q_nearest = V[q_nearest_idx]

                        # sort the neighbours
                        cost_q_near_idx_list = []
                        q_min_idx = q_nearest_idx
                        c_min = tree.Cost(V, E, q_nearest_idx) + tree.CostLine(q_nearest, q_new)
                        for q_near_idx in q_near_idx_list:
                            if q_nearest_idx == q_near_idx or q_near_idx == child_id:
                                continue
                            q_near = V[q_near_idx]
                            new_cost = tree.Cost(V, E, q_near_idx) + tree.CostLine(q_near, q_new)
                            if new_cost > c_min:
                                continue
                            cost_q_near_idx_list.append( (new_cost, q_near_idx) ) 

                        sorted_cost_q_near_idx_list = sorted(cost_q_near_idx_list, key=operator.itemgetter(0))

                        collision_checked = {}
                        for (new_cost, q_near_idx) in sorted_cost_q_near_idx_list:
                            q_near = V[q_near_idx]
                            if CollisionFree(q_near, q_new, taskrrt.GetDofIndices()):
                                collision_checked[q_near_idx] = True
                                q_min_idx = q_near_idx
                                c_min = new_cost
                                break
                            else:
                                collision_checked[q_near_idx] = False

                        E[q_new_idx] = q_min_idx
                        E_updates = []

                        try:
                            cost_q_new = tree.Cost(V, E, q_new_idx)
                        except:
                            print "updates_start"
                            print updates_start
                            print "V.keys()"
                            print V.keys()
                            print "E"
                            print E
                            print "child_id, parent_id", child_id, parent_id

                        E_updates = []
                        for q_near_idx in q_near_idx_list:
                            q_near = V[q_near_idx]
                            if cost_q_new + tree.CostLine(q_new, q_near) < tree.Cost(V, E, q_near_idx):
                                if q_near_idx in collision_checked:
                                    col = collision_checked[q_near_idx]
                                else:
                                    col = CollisionFree(q_new, q_near, taskrrt.GetDofIndices())
                                if col:
                                    q_parent_idx = E[q_near_idx]
                                    E[q_near_idx] = q_new_idx
                                    E_updates.append( (q_near_idx, child_id) )
                        return q_min_idx, E_updates

        queue_slave.put( ("init_complete",) )

        while not rospy.is_shutdown():
            msg = queue_master.get()
            cmd = msg[0]

            if cmd == "exit":
                break
            elif cmd == "specialCommand":
                queue_slave.put( ("specialCommand", True) )
                msg_s = queue_master_special.get()
                cmd_s = msg_s[0]
                if cmd_s == "setInitialConfiguration":
                    env_state = msg_s[1]
                    q, mo_state.obj_map, tree_serialized, grasped_bodies = env_state
                    mo_state.updateOpenrave(openrave.env)
                    openrave.or_octomap_client.SendCommand("SetOcTree " + tree_serialized)
                    openrave.robot_rave.SetDOFValues(q)
                    openrave.env.UpdatePublishedBodies()
                    for gr in grasped_bodies:
                        body_name, link_name = gr
                        openrave.robot_rave.Grab(openrave.env.GetKinBody(body_name), openrave.robot_rave.GetLink(link_name))

                    queue_slave.put( ("setInitialConfiguration", True) )
                if cmd_s == "setTaskSpec":
                    taskrrt = msg_s[1](openrave, msg_s[2])
                    dof_max_step = []
                    for joint_name in taskrrt.GetDofNames():
                        dof_max_step.append( joints_max_step[joint_name] )
                    queue_slave.put( ("setTaskSpec", True) )

            elif cmd == "addFirstNode":
                q_start_all = msg[1]
                V_update_q_new = taskrrt.getActiveDOF(q_start_all)
                queue_slave.put( ("addFirstNode", V_update_q_new, taskrrt.GetDofNames()) )

            elif cmd == "addNode":
#              try:
                tree_start, trees_goal, shortest_path_len, best_q_idx, goal_V_ids, job_id = msg[1:]
                V, E = tree_start
                sample_goal = random.uniform(0,1) < 0.05
                goal_found = False
                if sample_goal:

                    other_dof = taskrrt.GetOtherDofIndices()
                    start_conf = np.empty(len(other_dof))
                    limits = []
                    for dof_idx_idx in range(len(other_dof)):
                        start_conf[dof_idx_idx] = V[0][other_dof[dof_idx_idx]]
                        limits.append( taskrrt.GetDofLimits()[other_dof[dof_idx_idx]] )

                    for goal_tries in range(10):
                        goal_list = taskrrt.SampleGoal(V[0], shortest_path_len)
                        if goal_list == None or len(goal_list) == 0:
                            continue
                        for goal in goal_list:

                            if not isStateValid(goal, taskrrt.GetDofIndices()):
                                continue

                            # try to move the goal closer to the starting configuration
                            goal_conf = np.empty(len(other_dof))
                            for dof_idx_idx in range(len(other_dof)):
                                goal_conf[dof_idx_idx] = goal[other_dof[dof_idx_idx]]
                            goal_dist = np.linalg.norm(start_conf-goal_conf)
                            for dist_max in np.linspace(0.0, goal_dist*0.5, 5):
                                vol = velmautils.nSphereVolume(len(other_dof), dist_max)
                                max_tries = int(vol / 0.01)
                                if max_tries < 1:
                                    max_tries = 1
                                if max_tries > 1000:
                                    max_tries = 1000
                                new_goal_available = False
                                for try_idx in range(max_tries):
                                    goal_part = tree.uniformInBall(dist_max, limits, start_conf, ignore_dof=[])
                                    goal_new = copy.copy(goal)
                                    for dof_idx_idx in range(len(other_dof)):
                                        goal_new[other_dof[dof_idx_idx]] = goal_part[dof_idx_idx]
                                    if isStateValid(goal_new, taskrrt.GetDofIndices()):
                                        print "moved goal", tree.Distance(V[0], goal), tree.Distance(V[0], goal_new)
                                        goal = goal_new
                                        new_goal_available = True
                                        break

                                if new_goal_available:
                                    break

                            # check if the goal is too close to any other goal
                            goal_too_close_to_other = False
                            for tree_id in trees_goal:
                                gV, gE = trees_goal[tree_id]
                                if tree.Distance(gV[tree_id], goal) < 10.0/180.0*math.pi:
                                    print "found goal that is too close to other goal"
                                    goal_too_close_to_other = True
                                    break
                            if goal_too_close_to_other:
                                continue
                            if shortest_path_len != None and tree.CostLine(V[0], q_new) > shortest_path_len:
                                print "foung valid goal - too far"
                                continue
                            print "foung valid goal"
                            goal_found = True
                            q_new = goal
                            break
                        if goal_found:
                            break

                # a new goal is found - create a new tree
                if goal_found:
                    E_updates_start = []
                    queue_slave.put( ("addNode", None, E_updates_start, {-1:(q_new,None)}, {}, goal_found, {}, job_id), False )
                else:
                    q_new_idx = 1000000 * (1+process_id)
                    updates_start = []
                    updates_goals = {}

                    paths_found = {}

                    # RRT-connect step 1
                    q_rand = SampleFree(taskrrt.GetDofIndices(), taskrrt.GetDofLimits(), V[0], shortest_path_len, best_q_idx, goal_V_ids, V, E)
                    status, update, q_new_idx = Extend(V, E, q_rand, q_new_idx)
                    if status != "trapped":
                        connect_idx = q_new_idx-1
                        updates_start.append(update)
                        q_new = V[connect_idx]
                        for tree_id in trees_goal:
                            gV, gE = trees_goal[tree_id]
                            status, updates, q_new_idx = Connect(gV, gE, q_new, q_new_idx)
                            trees_goal[tree_id] = (gV, gE)
                            if len(updates) > 0:
                                updates_goals[tree_id] = updates

                            if status == "reached":
                                path_start = tree.GetPath(E, connect_idx)
                                path_goal = tree.GetPath(gE, q_new_idx-1)
                                path_goal.reverse()
                                paths_found[tree_id] = path_start[:-1] + path_goal

                    # RRT-connect step 2
                    q_rand = SampleFree(taskrrt.GetDofIndices(), taskrrt.GetDofLimits(), V[0], shortest_path_len, best_q_idx, goal_V_ids, V, E)
                    for tree_id in trees_goal:
                        gV, gE = trees_goal[tree_id]
                        status, update, q_new_idx = Extend(gV, gE, q_rand, q_new_idx)
                        trees_goal[tree_id] = (gV, gE)
                        if status != "trapped":
                            connect_idx = q_new_idx-1
                            if not tree_id in updates_goals:
                                updates_goals[tree_id] = []
                            updates_goals[tree_id].append(update)
                            q_new = gV[connect_idx]
                            status, updates, q_new_idx = Connect(V, E, q_new, q_new_idx)
                            updates_start += updates
                            if status == "reached":
                                path_start = tree.GetPath(E, q_new_idx-1)
                                path_goal = tree.GetPath(gE, connect_idx)
                                path_goal.reverse()
                                paths_found[tree_id] = path_start[:-1] + path_goal


                    # RRT* for the main tree
                    E_updates_start = []
                    near_dist = 120.0/180.0*math.pi
#                    near_dist = 60.0/180.0*math.pi
                    for update_idx in range(len(updates_start)):
                        V_update_q_new, child_id, parent_id = updates_start[update_idx]
                        if child_id <= parent_id:
                            print "updates_start"
                            print updates_start
                            print "ERROR: child_id <= parent_id", child_id, "<=", parent_id
                            exit(0)
                        q_min_idx, E_updates = applyStar(V, E, child_id)
                        E_updates_start += E_updates
                        updates_start[update_idx] = V_update_q_new, child_id, q_min_idx

                    E_updates_goals = {}
                    for tree_id in updates_goals:
                        E_updates_goals[tree_id] = []
                        for update_idx in range(len(updates_goals[tree_id])):
#                            print "updates_goals[tree_id][update_idx]"
#                            print updates_goals[tree_id][update_idx]
                            V_update_q_new, child_id, parent_id = updates_goals[tree_id][update_idx]
                            if child_id <= parent_id:
                                print "updates_goals"
                                print updates_goals
                                print "ERROR: child_id <= parent_id", child_id, "<=", parent_id
                                exit(0)
                        gV, gE = trees_goal[tree_id]
                        q_min_idx, E_updates = applyStar(gV, gE, child_id)
                        E_updates_goals[tree_id] += E_updates
                        updates_goals[tree_id][update_idx] = V_update_q_new, child_id, q_min_idx

                    queue_slave.put( ("addNode", updates_start, E_updates_start, updates_goals, E_updates_goals, goal_found, paths_found, job_id), False )

#              except:
#                print "exception in process", process_id
#                queue_slave.put( ("addNode", None, None, None) )

    def sendToAll(self, msg):
        for proc_id in range(self.num_proc):
            self.queue_master.put( ("specialCommand", True) )
        for proc_id in range(self.num_proc):
            resp = self.queue_slave.get(True)
            if resp[0] != "specialCommand":
                print "ERROR resp (specialCommand)", msg[0], resp[0]
                exit(0)
        for proc_id in range(self.num_proc):
            self.queue_master_special.put( msg )
        for proc_id in range(self.num_proc):
            resp = self.queue_slave.get(True)
            if resp[0] != msg[0]:
                print "ERROR resp", msg[0], resp[0]
                exit(0)

    def __init__(self, num_proc, env_file, obj_filenames, srdf_path, debug=True):
        self.debug = debug
        if self.debug:
            self.pub_marker = velmautils.MarkerPublisher()

        self.num_proc = num_proc
        self.proc = []
        self.queue_master = Queue(maxsize=self.num_proc)
        self.queue_master_special = Queue(maxsize=self.num_proc)
        self.queue_slave = Queue(maxsize=20)
        for i in range(self.num_proc):
            self.proc.append( Process(target=self.openraveWorker, args=(i, env_file, obj_filenames, srdf_path, self.queue_master, self.queue_master_special, self.queue_slave,)) )
            self.proc[-1].start()

    def waitForInit(self):
        # receive n messages
        for i in range(self.num_proc):
            result = self.queue_slave.get()
        print "PlannerRRT: all processes initalized"

    def cleanup(self):
        for i in range(self.num_proc):
            self.queue_master.put( ("exit", True), True )
        for proc in self.proc:
            proc.join()

    def RRTstar(self, env_state, classTaskRRT, taskArgs, max_time):

            def DrawPath(V, E, q_idx):
                if not q_idx in E:
                    return 0
                parent_idx = E[q_idx]
                m_id = DrawPath(V, E, parent_idx)
                m_id = self.pub_marker.publishVectorMarker(PyKDL.Vector(V[parent_idx][0], V[parent_idx][1], V[parent_idx][2]), PyKDL.Vector(V[q_idx][0], V[q_idx][1], V[q_idx][2]), m_id, 1, 0, 0, frame='world', namespace='shortest_path', scale=0.02)
                return m_id

            if self.debug:
                edge_ids = {}
                edge_id = 0
                self.pub_marker.eraseMarkers(0,3000, frame_id='world')

            shortest_path_len = None
            goal_not_found = True
            shortest_path_len_prev = None
            best_q = None
            best_q_idx = None
            V = {}
            E = {}
            goal_V_ids = []

            q_init = env_state[0]
            self.sendToAll( ("setInitialConfiguration", env_state,) )

            self.sendToAll( ("setTaskSpec", classTaskRRT, taskArgs) )

            self.queue_master.put( ("addFirstNode", q_init), True )
            resp = self.queue_slave.get(True)
            if resp[0] != "addFirstNode":
                print "ERROR resp (addFirstNode):", resp[0]
                exit(0)
            V_update_q_new, dof_names = resp[1:]

            V[0] = V_update_q_new

            job_id = 1
            trees_goal = {}
            q_new_idx = 0
            tokens = 0
            for proc_id in range(self.num_proc):
                self.queue_master.put( ("addNode", (V, E), trees_goal, shortest_path_len, best_q_idx, goal_V_ids, job_id), True )
                job_id += 1
                tokens += 1

            first_valid_job_id = 0
            start_time = rospy.Time.now()
            for k in range(100000):
                if (rospy.Time.now()-start_time).to_sec() > max_time:
                    break

                resp = self.queue_slave.get(True)
                self.queue_master.put( ("addNode", (V, E), trees_goal, shortest_path_len, best_q_idx, goal_V_ids, job_id), False )
                job_id += 1
                if resp[0] != "addNode":
                    print "ERROR resp (addNode):", resp[0]
                    exit(0)
                
                updates_start, E_updates_start, updates_goals, E_updates_goals, goal, paths_found, resp_job_id = resp[1:]
                if resp_job_id < first_valid_job_id:
                    print "discarding changes:", resp_job_id, "<", first_valid_job_id
                    continue

                ids_map = {}
                if updates_start != None:
                    for update in updates_start:
                        V_update_q_new, child_id, parent_id = update
                        if parent_id in ids_map:
                            parent_id = ids_map[parent_id]
                        if not parent_id in V:
                            print "ERROR: updates_start: not parent_id in V, update:", update
                            exit(0)
                        q_new_idx += 1
                        ids_map[child_id] = q_new_idx
                        V[q_new_idx] = V_update_q_new
                        E[q_new_idx] = parent_id

                        if not q_new_idx in edge_ids:
                            edge_ids[q_new_idx] = edge_id
                            edge_id += 1
                        if self.debug:
                            self.pub_marker.publishVectorMarker(PyKDL.Vector(V[parent_id][0], V[parent_id][1], V[parent_id][2]), PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), edge_ids[q_new_idx], 0, 1, 0, frame='world', namespace='edges', scale=0.01)

                for E_update in E_updates_start:
                    child_id = E_update[0]
                    parent_id = E_update[1]
                    if child_id in ids_map:
                        child_id = ids_map[child_id]
                    if parent_id in ids_map:
                        parent_id = ids_map[parent_id]
                    E[child_id] = parent_id

                # add or modify goal trees
                if len(updates_goals) > 0:
                    for tree_id in updates_goals:
                        # add new tree
                        if tree_id == -1:
                            q_new, _None = updates_goals[tree_id]
                            q_new_idx += 1
                            trees_goal[q_new_idx] = ({q_new_idx:q_new}, {})
                            if not q_new_idx in edge_ids:
                                edge_ids[q_new_idx] = edge_id
                                edge_id += 1
                            if self.debug:
                                self.pub_marker.publishSinglePointMarker(PyKDL.Vector(q_new[0], q_new[1], q_new[2]), edge_ids[q_new_idx], r=0, g=0, b=1, m_type=Marker.SPHERE, frame_id='world', namespace='edges', scale=Vector3(0.05, 0.05, 0.05))
                        elif tree_id in trees_goal:
                            gV, gE = trees_goal[tree_id]
                            for update in updates_goals[tree_id]:
                                V_update_q_new, child_id, parent_id = update
                                if parent_id in ids_map:
                                    parent_id = ids_map[parent_id]
                                if not parent_id in gV:
                                    print "ERROR"
                                    print gV
                                    print ids_map
                                    print updates_goals[tree_id]
                                    print updates_start
                                    print parent_id
                                    exit(0)
                                q_new_idx += 1
                                ids_map[child_id] = q_new_idx
                                gV[q_new_idx] = V_update_q_new
                                gE[q_new_idx] = parent_id

                                if not q_new_idx in edge_ids:
                                    edge_ids[q_new_idx] = edge_id
                                    edge_id += 1
                                if self.debug:
                                    self.pub_marker.publishVectorMarker(PyKDL.Vector(gV[parent_id][0], gV[parent_id][1], gV[parent_id][2]), PyKDL.Vector(gV[q_new_idx][0], gV[q_new_idx][1], gV[q_new_idx][2]), edge_ids[q_new_idx], 0, 0, 1, frame='world', namespace='edges', scale=0.01)

                            trees_goal[tree_id] = gV, gE

                            for E_update in E_updates_goals[tree_id]:
                                child_id = E_update[0]
                                parent_id = E_update[1]
                                if child_id in ids_map:
                                    child_id = ids_map[child_id]
                                if parent_id in ids_map:
                                    parent_id = ids_map[parent_id]
                                gE[child_id] = parent_id

                # paths to goals found: remove goal trees and add paths to goals to the main tree
                for tree_id in paths_found:
                    if not tree_id in trees_goal:
                        print "ERROR: not tree_id in trees_goal", tree_id
                        print "trees_goal"
                        print trees_goal
                        exit(0)
                    first_valid_job_id = job_id

                    path = paths_found[tree_id]
                    print "adding path to goal: ", path
                    gV, gE = trees_goal[tree_id]
                    parent_q_idx = None
                    for q_path_idx in range(len(path)):
                        q_idx = path[q_path_idx]
                        if q_idx in ids_map:
                            q_idx = ids_map[q_idx]
                        if q_idx in gV and not q_idx in V:
                            parent_q_idx = path[q_path_idx-1]
                            if parent_q_idx in ids_map:
                                parent_q_idx = ids_map[parent_q_idx]
                            V[q_idx] = gV[q_idx]
                            E[q_idx] = parent_q_idx
                            if not q_idx in edge_ids:
                                edge_ids[q_idx] = edge_id
                                edge_id += 1
                            if self.debug:
                                print parent_q_idx, q_idx, edge_id
                                self.pub_marker.publishVectorMarker(PyKDL.Vector(V[parent_q_idx][0], V[parent_q_idx][1], V[parent_q_idx][2]), PyKDL.Vector(V[q_idx][0], V[q_idx][1], V[q_idx][2]), edge_ids[q_idx], 0, 1, 0, frame='world', namespace='edges', scale=0.01)
                    del trees_goal[tree_id]

                    goal_V_ids.append(q_idx)

                trees_sizes = []
                for tree_id in trees_goal:
                    trees_sizes.append(len(trees_goal[tree_id][0]))
                print "V goals, trees", len(V), len(goal_V_ids), len(trees_goal), trees_sizes

                if True:
                    for goal_idx in goal_V_ids:
                        goal_cost = tree.Cost(V, E, goal_idx)
                        if goal_not_found or shortest_path_len > goal_cost:
                            goal_not_found = False
                            best_q_idx = goal_idx
                            best_q = V[best_q_idx]
                            shortest_path_len =  goal_cost
                            #print "*********** found better goal:", shortest_path_len
                            if self.debug:
                                print " %s  , shortest_path: %s"%(k, shortest_path_len)
                                self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                                DrawPath(V, E, best_q_idx)

                    if shortest_path_len_prev != shortest_path_len:
                        # sanity check
                        for vi in V:
                            tree.GetPath(E, vi)
                        for vi in E:
                            tree.GetPath(E, vi)
                        rem_nodes = []
                        for vi in V:
                            if tree.CostLine(V[0], V[vi]) > shortest_path_len:
                                rem_nodes.append(vi)
                        print "removing nodes:", len(rem_nodes)
                        if len(rem_nodes) > 0:
                            first_valid_job_id = job_id
                        for vi in rem_nodes:
                            del V[vi]
                            if self.debug:
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
                            for vi in rem_nodes:
                                if not vi in V:
                                    print "ERROR"
                                    print "V:"
                                    for vii in V:
                                        print vii
                                    print E
                                del V[vi]
                                if self.debug:
                                    self.pub_marker.eraseMarkers(edge_ids[vi], edge_ids[vi]+1, frame_id='torso_base', namespace='edges')
                                del edge_ids[vi]
                                del E[vi]
                                if vi in goal_V_ids:
                                    goal_V_ids.remove(vi)

                        rem_trees = []
                        for tree_id in trees_goal:
                            gV, gE = trees_goal[tree_id]
                            if tree.CostLine(V[0], gV[tree_id]) > shortest_path_len:
                                rem_trees.append(tree_id)

                        print "removing trees", rem_trees
                        for tree_id in rem_trees:
                            del trees_goal[tree_id]

            print "planning finished, waiting for all jobs..."

            for proc_id in range(self.num_proc):
                resp = self.queue_slave.get(True)
                if resp[0] != "addNode":
                    print "ERROR resp (addNode):", resp[0]
                    exit(0)

            path = tree.GetPath(E, best_q_idx)
            print "path", path

            path_q = []
            for v_idx in path:
                path_q.append( V[v_idx] )
            return path_q, dof_names

