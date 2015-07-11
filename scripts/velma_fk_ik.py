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

import PyKDL
import numpy as np

from urdf_parser_py.urdf import URDF
import pykdl_utils.kdl_parser as kdl_urdf

# this hack in for fixed torso_1_joint
def kdl_tree_from_urdf_model_velma(urdf, torso_1_joint_value):
    segment_map = {}
    segment_id = 0
    segment_name_id_map = {}
    segment_parent_map = {}
    root = urdf.get_root()
    tree = PyKDL.Tree(root)

    segment_map[segment_id] = None
    segment_parent_map[segment_id] = None
    segment_name_id_map[root] = segment_id
    segment_id += 1

    def add_children_to_tree(parent, segment_id):
        print "add_children_to_tree", parent, segment_id
        if parent in urdf.child_map:
            for joint, child_name in urdf.child_map[parent]:
                if parent == 'torso_link0' and child_name == 'torso_link1':
                    joint_rot = -torso_1_joint_value
                    urdf.joint_map[joint].joint_type = 'fixed'
                elif parent == 'torso_link1' and child_name == 'torso_link2':
                    joint_rot = torso_1_joint_value
                    urdf.joint_map[joint].joint_type = 'fixed'
                else:
                    joint_rot = 0.0
                child = urdf.link_map[child_name]
                if child.inertial is not None:
                    kdl_inert = kdl_urdf.urdf_inertial_to_kdl_rbi(child.inertial)
                else:
                    kdl_inert = PyKDL.RigidBodyInertia()
                kdl_jnt = kdl_urdf.urdf_joint_to_kdl_joint(urdf.joint_map[joint])
                kdl_origin = kdl_urdf.urdf_pose_to_kdl_frame(urdf.joint_map[joint].origin) * PyKDL.Frame(PyKDL.Rotation.RotZ(joint_rot))
                kdl_sgm = PyKDL.Segment(child_name, kdl_jnt,
                                      kdl_origin, kdl_inert)

                segment_map[segment_id] = kdl_sgm
                segment_parent_map[segment_id] = segment_name_id_map[parent]
                segment_name_id_map[child_name] = segment_id
                segment_id += 1

                tree.addSegment(kdl_sgm, parent)
                segment_id = add_children_to_tree(child_name, segment_id)
        return segment_id
    add_children_to_tree(root, segment_id)
    return tree, segment_map, segment_parent_map, segment_name_id_map

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

    def calculateFk2(self, base_name, end_name, q):
        q_fk = PyKDL.JntArray( self.jac_solver_chain_len_map[(base_name, end_name)] )
        q_fk_idx = 0
        for q_idx in self.jac_solver_q_indices_map[(base_name, end_name)]:
            q_fk[q_fk_idx] = q[q_idx]
#            print q_fk_idx, q_idx, q_fk[q_fk_idx]
            q_fk_idx += 1
        fr = PyKDL.Frame()
        self.fk_solver_map[(base_name, end_name)].JntToCart(q_fk, fr)
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

    def createSegmentToJointMap(self, joint_names_vector):
        self.segment_id_q_id_map = {}
        for q_idx in range(len(joint_names_vector)):
            joint_name = joint_names_vector[q_idx]
            for seg_id in self.segment_map:
                seg = self.segment_map[seg_id]
                if seg == None:
                    continue
#                print "   ", seg.getJoint().getName()
                if joint_name == seg.getJoint().getName():
                    self.segment_id_q_id_map[seg_id] = q_idx
                    print "createSegmentToJointMap", joint_name, seg_id, q_idx
#        print "self.segment_id_q_id_map"
#        print self.segment_id_q_id_map

    def createJacobianFkSolvers(self, base_name, end_name, joint_names_vector):
        if not hasattr(self, 'jac_solver_chain_map'):
            self.jac_solver_chain_map = {}
            self.jac_solver_map = {}
            self.jac_solver_names_map = {}
            self.jac_solver_q_indices_map = {}
            self.jac_solver_chain_len_map = {}
            self.fk_solver_map = {}
        if not (base_name, end_name) in self.jac_solver_chain_map:
            chain = self.tree.getChain(base_name, end_name)
            self.jac_solver_chain_map[(base_name, end_name)] = chain
            self.jac_solver_map[(base_name, end_name)] = PyKDL.ChainJntToJacSolver( chain )
            self.jac_solver_names_map[(base_name, end_name)] = joint_names_vector
            self.jac_solver_q_indices_map[(base_name, end_name)] = []
            self.fk_solver_map[(base_name, end_name)] = PyKDL.ChainFkSolverPos_recursive(chain)
            for chain_q_idx in range(chain.getNrOfSegments()):
                joint = chain.getSegment(chain_q_idx).getJoint()
                chain_joint_name = joint.getName()
                chain_joint_type = joint.getType()
                if chain_joint_type == PyKDL.Joint.None:
                    continue
                print "chain", chain_joint_name, chain_joint_type
                q_idx = 0
                for joint_name in joint_names_vector:
                    if joint_name == chain_joint_name:
                        self.jac_solver_q_indices_map[(base_name, end_name)].append(q_idx)
                        break
                    q_idx += 1
                if q_idx == len(joint_names_vector):
                    print "ERROR: createJacobianSolver", chain_joint_name, " not in", joint_names_vector
                    exit(0)
            self.jac_solver_chain_len_map[(base_name, end_name)] = len(self.jac_solver_q_indices_map[(base_name, end_name)])
            print "joints in the chain:", self.jac_solver_chain_len_map[(base_name, end_name)]
        else:
            print "ERROR: createJacobianSolver: solver already exists"
            exit(0)

    def getJacobian(self, base_name, end_name, q):
        # extract joint values for the chain
        q_jac = PyKDL.JntArray( self.jac_solver_chain_len_map[(base_name, end_name)] )
        q_jac_idx = 0
        for q_idx in self.jac_solver_q_indices_map[(base_name, end_name)]:
            q_jac[q_jac_idx] = q[q_idx]
            q_jac_idx += 1
        jac_small = PyKDL.Jacobian( self.jac_solver_chain_map[(base_name, end_name)].getNrOfJoints() )
        self.jac_solver_map[(base_name, end_name)].JntToJac(q_jac, jac_small)

        # create the jacobian for all joints
        jac_big = np.matrix(np.zeros( (len(q), 6) ))

        for col_idx in range(jac_small.columns()):
            q_idx = self.jac_solver_q_indices_map[(base_name, end_name)][col_idx]
            col = jac_small.getColumn(col_idx)
            for row_idx in range(6):
                jac_big[q_idx, row_idx] = col[row_idx]
        return jac_big

    def __init__(self, torso_1_joint_value):
        self.robot = URDF.from_parameter_server()

        self.tree, self.segment_map, self.segment_parent_map, self.segment_name_id_map = kdl_tree_from_urdf_model_velma(self.robot, torso_1_joint_value)

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

        self.joint_limit_map = {}
        for j in self.robot.joints:
            if j.limit != None:
                self.joint_limit_map[j.name] = j.limit

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
                self.q_min[link_name][j_idx] = self.joint_limit_map[joint_name].lower
                self.q_max[link_name][j_idx] = self.joint_limit_map[joint_name].upper
                self.ik_joint_state_name[link_name].append(joint_name)
                j_idx += 1
            # prepare fk solver for ik solver
            self.ik_fk_solver[link_name] = PyKDL.ChainFkSolverPos_recursive(self.ik_chains[link_name])
            self.vel_ik_solver[link_name] = PyKDL.ChainIkSolverVel_pinv(self.ik_chains[link_name])
            self.ik_solvers[link_name] = PyKDL.ChainIkSolverPos_NR_JL(self.ik_chains[link_name], self.q_min[link_name], self.q_max[link_name], self.ik_fk_solver[link_name], self.vel_ik_solver[link_name], 100)


