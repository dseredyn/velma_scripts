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

    def calculateFk(self, link_name, js_pos):
        q = PyKDL.JntArray(self.fk_chains[link_name].getNrOfJoints())
        ja_idx = 0
        for js_name in self.fk_joint_state_name[link_name]:
            q[ja_idx] = js_pos[js_name]
            ja_idx += 1

        fr = PyKDL.Frame()
        self.fk_solvers[link_name].JntToCart(q, fr)
        return fr

    def simulateTrajectory(self, link_name, init_js, T_B_Ed):

        chain_length = self.ik_chains[link_name].getNrOfJoints()

        q_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q_init = PyKDL.JntArray(chain_length)
        for ja_idx in range(chain_length):
            q_init[ja_idx] = init_js[self.ik_joint_state_name[link_name][ja_idx]]

        T_B_BB = self.calculateFk(self.ik_base, init_js)
        T_BB_B = T_B_BB.Inverse()

        q_out = PyKDL.JntArray(chain_length)
        T_BB_Ed = T_BB_B * T_B_Ed
        status = self.ik_solvers[link_name].CartToJnt(q_init, T_BB_Ed, q_out)
        if status != 0:
            return None
        for i in range(chain_length):
            q_end[i] = q_out[i]
        return q_end

    def __init__(self):
        self.robot = URDF.from_parameter_server()

        self.tree = kdl_tree_from_urdf_model(self.robot)
        fk_links = [
        "torso_link2",
        "left_arm_7_link",
        "right_arm_7_link",
        "left_HandPalmLink",
        "right_HandPalmLink",
        ]
        self.fk_chains = {}
        self.fk_solvers = {}
        self.fk_joint_state_name = {}
        for link_name in fk_links:
            self.fk_chains[link_name] = self.tree.getChain("torso_base", link_name)
            self.fk_solvers[link_name] = PyKDL.ChainFkSolverPos_recursive(self.fk_chains[link_name])

            self.fk_joint_state_name[link_name] = []
            for seg_idx in range(self.fk_chains[link_name].getNrOfSegments()):
                joint = self.fk_chains[link_name].getSegment(seg_idx).getJoint()
                if joint.getType() == PyKDL.Joint.None:
                    continue
                joint_name = joint.getName()

                self.fk_joint_state_name[link_name].append(joint_name)

        self.ik_base = "torso_link2"
        ik_links = [
        "left_HandPalmLink",
        "right_HandPalmLink",
        "left_arm_7_link",
        "right_arm_7_link",
        ]

        joint_limit_map = {}
        for j in self.robot.joints:
            if j.limit != None:
                joint_limit_map[j.name] = j.limit

        self.ik_fk_solver = {}
        self.vel_ik_solver = {}
        self.q_min = {}
        self.q_max = {}
        self.ik_solvers = {}
        self.ik_chains = {}
        self.ik_joint_state_name = {}
        for link_name in ik_links:
            # get chain
            self.ik_chains[link_name] = self.tree.getChain(self.ik_base, link_name)

            # get limits
            self.q_min[link_name] = PyKDL.JntArray(self.ik_chains[link_name].getNrOfJoints())
            self.q_max[link_name] = PyKDL.JntArray(self.ik_chains[link_name].getNrOfJoints())
            j_idx = 0
            self.ik_joint_state_name[link_name] = []
            for seg_idx in range(self.ik_chains[link_name].getNrOfSegments()):
                joint = self.ik_chains[link_name].getSegment(seg_idx).getJoint()
                if joint.getType() == PyKDL.Joint.None:
                    continue
                joint_name = joint.getName()
                self.q_min[link_name][j_idx] = joint_limit_map[joint_name].lower
                self.q_max[link_name][j_idx] = joint_limit_map[joint_name].upper
                self.ik_joint_state_name[link_name].append(joint_name)
                j_idx += 1
            # prepare fk solver for ik solver
            self.ik_fk_solver[link_name] = PyKDL.ChainFkSolverPos_recursive(self.ik_chains[link_name])
            self.vel_ik_solver[link_name] = PyKDL.ChainIkSolverVel_pinv(self.ik_chains[link_name])
            self.ik_solvers[link_name] = PyKDL.ChainIkSolverPos_NR_JL(self.ik_chains[link_name], self.q_min[link_name], self.q_max[link_name], self.ik_fk_solver[link_name], self.vel_ik_solver[link_name], 100)


