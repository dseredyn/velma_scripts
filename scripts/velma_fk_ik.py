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

import PyKDL

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class VelmaFkIkSolver:
    def calculateFk(self, link_name, joint_states):
        ja_idx = 0
        for js_idx in self.fk_joint_state_idx[link_name]:
            self.joint_arrays[link_name][ja_idx] = joint_states.position[js_idx]
            ja_idx += 1

        fr = PyKDL.Frame()
        self.fk_solvers[link_name].JntToCart(self.joint_arrays[link_name], fr)
        return fr

    def __init__(self, joint_states):
        self.robot = URDF.from_parameter_server()

        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.fk_links = [
        "torso_link2",
        "left_arm_7_link",
        "right_arm_7_link",
        "left_HandPalmLink",
        "right_HandPalmLink",
        ]
        self.chains = {}
        self.fk_solvers = {}
        self.joint_arrays = {}
        self.fk_joint_state_idx = {}
        for link_name in self.fk_links:
            self.chains[link_name] = self.tree.getChain("torso_base", link_name)
            self.fk_solvers[link_name] = PyKDL.ChainFkSolverPos_recursive(self.chains[link_name])

            self.joint_arrays[link_name] = PyKDL.JntArray(self.chains[link_name].getNrOfJoints())
            self.fk_joint_state_idx[link_name] = []
            for seg_idx in range(self.chains[link_name].getNrOfSegments()):
                joint = self.chains[link_name].getSegment(seg_idx).getJoint()
                if joint.getType() == PyKDL.Joint.None:
                    continue
                joint_name = joint.getName()

                for js_idx in range(len(joint_states.name)):
                    if joint_name == joint_states.name[js_idx]:
                        break
                self.fk_joint_state_idx[link_name].append(js_idx)

#        self.q_min = PyKDL.JntArray(7)
#        self.q_max = PyKDL.JntArray(7)
#        self.q_limit = 0.26
#        self.q_min[0] = -2.96 + self.q_limit
#        self.q_min[1] = -2.09 + self.q_limit
#        self.q_min[2] = -2.96 + self.q_limit
#        self.q_min[3] = -2.09 + self.q_limit
#        self.q_min[4] = -2.96 + self.q_limit
#        self.q_min[5] = -2.09 + self.q_limit
#        self.q_min[6] = -2.96 + self.q_limit
#        self.q_max[0] = 2.96 - self.q_limit
#        self.q_max[1] = 2.09 - self.q_limit
#        self.q_max[2] = 2.96 - self.q_limit
#        self.q_max[3] = 2.09 - self.q_limit
#        self.q_max[4] = 2.96 - self.q_limit
#        self.q_max[5] = 2.09 - self.q_limit
#        self.q_max[6] = 2.96 - self.q_limit

#        self.fk_solver = PyKDL.TreeFkSolverPos_recursive(self.tree)
#        self.vel_ik_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
#        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(self.chain, self.q_min, self.q_max, self.fk_solver, self.vel_ik_solver, 100)

#        self.q_out = PyKDL.JntArray(7)


    def calculateIk(self, x_index_start, x_index_end, print_lock, next_lock):
#process...
#print_lock.acquire
#print
#print_lock.release
#next_lock.release

        ret = []
        for x in self.x_set[x_index_start:x_index_end]:
            ret_x = []
            print >> sys.stderr, "x=%s"%(x)
            if rospy.is_shutdown():
                break
            y = 0
            for y in self.y_set:
                ret_y = []
                for z in self.z_set:
                    ret_z = []
                    if rospy.is_shutdown():
                        break
                    rot_index = 0
                    dist = (self.pt_c_in_T2.x()-x)*(self.pt_c_in_T2.x()-x) + (self.pt_c_in_T2.y()-y)*(self.pt_c_in_T2.y()-y) + (self.pt_c_in_T2.z()-z)*(self.pt_c_in_T2.z()-z)
                    if dist <= self.max_dist2 and dist >= self.min_dist2:
                        for r in self.rot:
                            fr = PyKDL.Frame(r, PyKDL.Vector(x,y,z))
                            success = False
                            for i in range(0,5):
                                q_init = PyKDL.JntArray(7)
                                for j in range(0,7):
                                    q_init[j] = random.uniform(self.q_min[j]+0.1, self.q_max[j]-0.1)
                                status = self.ik_solver.CartToJnt(q_init, fr, self.q_out)
                                if status == 0 and not self.hasSingularity(self.q_out):
                                    success = True
#                                    print >> sys.stderr, "sing. %s"%(i)
                                    break
                            if success:
                                ret_z.append(rot_index)
                            rot_index += 1
                    ret_y.append(ret_z)
                ret_x.append(ret_y)
            ret.append(ret_x)

        if print_lock != None:
            print_lock.acquire()
        self.printLookupTablePart(ret)
        if print_lock != None:
            print_lock.release()
        if next_lock != None:
            next_lock.release()

