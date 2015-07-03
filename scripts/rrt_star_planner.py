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

import rospy
import PyKDL
import numpy as np
from multiprocessing import Process, Queue
import math
import openraveinstance
import velmautils
import tree
from prolatehyperspheroid import ProlateHyperspheroid
import random
from openravepy import *
import operator

class PlannerRRT:

    def openraveWorker(self, process_id, env_file, xacro_uri, srdf_path, queue_master, queue_master_special, queue_slave):
      openrave = openraveinstance.OpenraveInstance()
      openrave.startOpenraveURDF(env_file=env_file, viewer=False)
      openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

      with openrave.env:

        ETA = 60.0/180.0*math.pi
        gamma = 0.5

        def isStateValid(q, dof_indices):
            openrave.switchCollisionModel("velmasimplified1")

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
#                            print "path_len", path_len
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
#                        print "SampleFree", tries
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
                    if tree.Distance(q, V[vi]) < near_dist:
                        result.append(vi)
                return result

        def CollisionFree(q1, q2, dof_indices):
                dist = max(abs(q1-q2))
                steps = int(dist / (5.0/180.0*math.pi))
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
            elif cmd == "specialCommand":
                queue_slave.put( ("specialCommand", True) )
                msg_s = queue_master_special.get()
                cmd_s = msg_s[0]
                if cmd_s == "setInitialConfiguration":
                    q = msg_s[1]
                    openrave.robot_rave.SetDOFValues(q)
                    queue_slave.put( ("setInitialConfiguration", True) )
                if cmd_s == "setTaskSpec":
                    taskrrt = msg_s[1](openrave, msg_s[2])
                    queue_slave.put( ("setTaskSpec", True) )

            elif cmd == "addFirstNode":
                q_start_all = msg[1]
                V_update_q_new = taskrrt.getActiveDOF(q_start_all)
                queue_slave.put( ("addFirstNode", V_update_q_new, taskrrt.GetDofNames()) )

            elif cmd == "addNode":
#              try:
                V, E, shortest_path_len, best_q_idx, goal_V_ids = msg[1:]
                sample_goal = random.uniform(0,1) < 0.05

                goal_found = False
                if sample_goal:
                    for goal_tries in range(10):
                        goal_list = taskrrt.SampleGoal(V[0], shortest_path_len)
                        if goal_list == None or len(goal_list) == 0:
                            continue
#                        print len(goal_list)
                        for goal in goal_list:
#                            openrave.showConfiguration(taskrrt.GetDofNames(), goal)
                            if isStateValid(goal, taskrrt.GetDofIndices()):
                                print "foung valid goal"
                                q_nearest_idx = Nearest(V, goal)
                                q_nearest = V[q_nearest_idx]
                                q_new = goal
                                if shortest_path_len != None and tree.CostLine(V[0], q_new) > shortest_path_len:
                                    print "foung valid goal - too far"
                                    continue
                                col_free = CollisionFree(q_nearest, q_new, taskrrt.GetDofIndices())
                                if col_free:
                                    print "foung valid goal - may connect"
                                    goal_found = True
                                    break
                        if goal_found:
                            break
                if not goal_found:
                    while True:
                        q_rand = SampleFree(taskrrt.GetDofIndices(), taskrrt.GetDofLimits(), V[0], shortest_path_len, best_q_idx, goal_V_ids, V, E)
                        q_nearest_idx = Nearest(V, q_rand)
                        q_nearest = V[q_nearest_idx]
                        q_new = Steer(q_nearest, q_rand)
                        if shortest_path_len != None and tree.CostLine(V[0], q_new) > shortest_path_len:
                            continue
                        col_free = CollisionFree(q_nearest, q_new, taskrrt.GetDofIndices())
                        if col_free:
                            break

                if not col_free:
                    print "ERROR: not col_free"
                    exit(0)
                if col_free:

                    near_dist = 120.0/180.0*math.pi
                    q_near_idx_list = Near(V, q_new, near_dist)
#                    print "q_near_idx_list", len(q_near_idx_list)

                    # sort the neighbours
                    cost_q_near_idx_list = []
                    q_min_idx = q_nearest_idx
                    c_min = tree.Cost(V, E, q_nearest_idx) + tree.CostLine(q_nearest, q_new)
                    for q_near_idx in q_near_idx_list:
                        if q_nearest_idx == q_near_idx:
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

                    q_new_idx = 1000000
                    V[q_new_idx] = q_new
                    V_update_q_new = q_new

                    E[q_new_idx] = q_min_idx
                    E_update = []
                    E_update.append( (-1, q_min_idx) )

                    cost_q_new = tree.Cost(V, E, q_new_idx)
                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if cost_q_new + tree.CostLine(q_new, q_near) < tree.Cost(V, E, q_near_idx):
                            if q_near_idx in collision_checked:
                                col = collision_checked[q_near_idx]
                            else:
                                col = CollisionFree(q_new, q_near, taskrrt.GetDofIndices())
                            if col:
                                q_parent_idx = E[q_near_idx]
                                print "rem: %s  %s"%(q_parent_idx, q_near_idx)
                                E[q_near_idx] = q_new_idx
                                E_update.append( (q_near_idx, -1) )

                    goal = goal_found or taskrrt.checkGoal(q_new)
                    if goal:
                        print "found goal"

                    queue_slave.put( ("addNode", V_update_q_new, E_update, goal), False )
                else:
                    queue_slave.put( ("addNode", None, None, None), False )
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

    def __init__(self, num_proc, env_file, xacro_uri, srdf_path, debug=True):
        self.debug = debug
        if self.debug:
            self.pub_marker = velmautils.MarkerPublisher()

        self.num_proc = num_proc
        self.proc = []
        self.queue_master = Queue(maxsize=self.num_proc)
        self.queue_master_special = Queue(maxsize=self.num_proc)
        self.queue_slave = Queue(maxsize=20)
        for i in range(self.num_proc):
            self.proc.append( Process(target=self.openraveWorker, args=(i, env_file, xacro_uri, srdf_path, self.queue_master, self.queue_master_special, self.queue_slave,)) )
            self.proc[-1].start()

        # receive n messages
        for i in range(self.num_proc):
            result = self.queue_slave.get()

        print "PlannerRRT: all processes initalized"

    def cleanup(self):
        for i in range(self.num_proc):
            self.queue_master.put( ("exit", True), True )
        for proc in self.proc:
            proc.join()

    def RRTstar(self, q_init, classTaskRRT, taskArgs, max_time):

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
#            shortest_path_len = 6.0
            goal_not_found = True
            shortest_path_len_prev = None
            best_q = None
            best_q_idx = None
            V = {}
            E = {}
            goal_V_ids = []

            self.sendToAll( ("setInitialConfiguration", q_init) )

            self.sendToAll( ("setTaskSpec", classTaskRRT, taskArgs) )

            self.queue_master.put( ("addFirstNode", q_init), True )
            resp = self.queue_slave.get(True)
            if resp[0] != "addFirstNode":
                print "ERROR resp (addFirstNode):", resp[0]
                exit(0)
            V_update_q_new, dof_names = resp[1:]

            V[0] = V_update_q_new

            q_new_idx = 0
            tokens = 0
            for proc_id in range(self.num_proc):
                self.queue_master.put( ("addNode", V, E, shortest_path_len, best_q_idx, goal_V_ids), True )
                tokens += 1

            start_time = rospy.Time.now()
            for k in range(100000):
                if (rospy.Time.now()-start_time).to_sec() > max_time:
                    break
                resp = self.queue_slave.get(True)
                self.queue_master.put( ("addNode", V, E, shortest_path_len, best_q_idx, goal_V_ids), False )
                if resp[0] != "addNode":
                    print "ERROR resp (addNode):", resp[0]
                    exit(0)

                V_update_q_new, E_update, goal = resp[1:]
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
                        if self.debug:
                            self.pub_marker.publishVectorMarker(PyKDL.Vector(V[vi_pa][0], V[vi_pa][1], V[vi_pa][2]), PyKDL.Vector(V[vi_ch][0], V[vi_ch][1], V[vi_ch][2]), edge_ids[vi_ch], 0, 1, 0, frame='world', namespace='edges', scale=0.01)

                    if goal:
                        goal_V_ids.append(q_new_idx)

                    print "len(V) len(goal_V_ids)", len(V), len(goal_V_ids)

                    for goal_idx in goal_V_ids:
                        goal_cost = tree.Cost(V, E, goal_idx)
#                        if shortest_path_len == None or shortest_path_len > goal_cost:
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
                        rem_nodes = []
                        for vi in V:
                            if tree.CostLine(V[0], V[vi]) > shortest_path_len:
                                rem_nodes.append(vi)
                        print "removing nodes:", len(rem_nodes)
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
#                            print "removing nodes:", len(rem_nodes)
                            for vi in rem_nodes:
                                del V[vi]
                                if self.debug:
                                    self.pub_marker.eraseMarkers(edge_ids[vi], edge_ids[vi]+1, frame_id='torso_base', namespace='edges')
                                del edge_ids[vi]
                                del E[vi]
                                if vi in goal_V_ids:
                                    goal_V_ids.remove(vi)

#                if self.stop_planning:
#                  print "planning stopped"
#                  break

            for proc_id in range(self.num_proc):
                resp = self.queue_slave.get(True)
                if resp[0] != "addNode":
                    print "ERROR resp (addNode):", resp[0]
                    exit(0)

            path = tree.GetPath(E, best_q_idx)

            path_q = []
            for v_idx in path:
                path_q.append( V[v_idx] )
            return path_q, dof_names

