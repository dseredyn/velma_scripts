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
import rospkg
import multiprocessing

import velmautils
from velma import Velma
import openraveinstance
import conversions as conv
#import rrt_star_planner_ee
import rrt_star_connect_planner
import tree
import rosparam
import tasks

def identityMatrix(size):
    I = np.matrix(numpy.zeros( (size, size) ))
    for idx in range(size):
        I[idx,idx] = 1.0
    return I

class TestHierarchyControl:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):

        simulation = True

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/jar/cabinet_test.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        print "done."

        #
        # Initialise Openrave
        #

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file, viewer=True)
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)
        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))

        velma.waitForInit()

        openrave.updateRobotConfigurationRos(velma.js_pos)

        non_adj_links_ids = openrave.robot_rave.GetNonAdjacentLinks()

        velma.switchToJoint()

        lim_bo_soft, lim_up_soft = velma.getJointSoftLimitsVectors()
        lim_bo, lim_up = velma.getJointLimitsVectors()

        velma.fk_ik_solver.createJacobianFkSolvers('torso_base', 'right_HandPalmLink', velma.getJointStatesVectorNames())
        velma.fk_ik_solver.createSegmentToJointMap(velma.getJointStatesVectorNames(), velma.getInactiveJointStatesVector())

        print velma.getJointStatesVectorNames()
        x_HAND_targets = [
#        PyKDL.Frame(PyKDL.Vector(0.5,0,1.8)),
#        PyKDL.Frame(PyKDL.Rotation.RotY(170.0/180.0*math.pi), PyKDL.Vector(0.5,0,1.6)),
        PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.2,0.0,1.0)),
        PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.2,-0.5,1.0)),
        ]
        target_idx = 0
        x_HAND_target = x_HAND_targets[target_idx]
        target_idx += 1

        last_time = rospy.Time.now()
        q = velma.getJointStatesVector()
        iq = velma.getInactiveJointStatesVector()

        counter = 0
        while not rospy.is_shutdown():
            if counter > 300:
                x_HAND_target = x_HAND_targets[target_idx]
                target_idx = (target_idx + 1)%len(x_HAND_targets)
                counter = 0
            counter += 1

            time_elapsed = rospy.Time.now() - last_time

            J_JLC = np.matrix(numpy.zeros( (len(q), len(q)) ))
            delta_V_JLC = np.empty(len(q))
            for q_idx in range(len(q)):
                if q[q_idx] < lim_bo_soft[q_idx]:
                    delta_V_JLC[q_idx] = q[q_idx] - lim_bo_soft[q_idx]
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(q[q_idx] - lim_bo_soft[q_idx]) / abs(lim_bo[q_idx] - lim_bo_soft[q_idx]))
                elif q[q_idx] > lim_up_soft[q_idx]:
                    delta_V_JLC[q_idx] = q[q_idx] - lim_up_soft[q_idx]
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(q[q_idx] - lim_up_soft[q_idx]) / abs(lim_up[q_idx] - lim_up_soft[q_idx]))
                else:
                    delta_V_JLC[q_idx] = 0.0
                    J_JLC[q_idx,q_idx] = 0.0

            J_JLC_inv = np.linalg.pinv(J_JLC)

            N_JLC = identityMatrix(len(q)) - (J_JLC_inv * J_JLC)

            v_max_JLC = 20.0/180.0*math.pi
            kp_JLC = 1.0
            dx_JLC_des = kp_JLC * delta_V_JLC

            # min(1.0, v_max_JLC/np.linalg.norm(dx_JLC_des))
            if v_max_JLC > np.linalg.norm(dx_JLC_des):
                 vv_JLC = 1.0
            else:
                vv_JLC = v_max_JLC/np.linalg.norm(dx_JLC_des)
            dx_JLC_ref = - vv_JLC * dx_JLC_des

            J_HAND = velma.fk_ik_solver.getJacobian('torso_base', 'right_HandPalmLink', q)
            J_HAND_inv = np.linalg.pinv(J_HAND)

            T_B_E = velma.fk_ik_solver.calculateFk2('torso_base', 'right_HandPalmLink', q)
            x_HAND_current = T_B_E
            x_HAND_diff = PyKDL.diff(x_HAND_target, x_HAND_current)
#            print x_HAND_diff.vel.Norm(), x_HAND_diff.rot.Norm()
            delta_V_HAND = np.empty(6)
            delta_V_HAND[0] = x_HAND_diff.vel[0]
            delta_V_HAND[1] = x_HAND_diff.vel[1]
            delta_V_HAND[2] = x_HAND_diff.vel[2]
            delta_V_HAND[3] = x_HAND_diff.rot[0]
            delta_V_HAND[4] = x_HAND_diff.rot[1]
            delta_V_HAND[5] = x_HAND_diff.rot[2]

            v_max_HAND = 2.0
            kp_HAND = 2.0
            dx_HAND_des = kp_HAND * delta_V_HAND

#            vv_HAND = min(1.0, v_max_HAND/np.linalg.norm(dx_HAND_des))
            if v_max_HAND > np.linalg.norm(dx_HAND_des):
                vv_HAND = 1.0
            else:
                vv_HAND = v_max_HAND/np.linalg.norm(dx_HAND_des)

#            print vv_JLC, vv_HAND
            dx_HAND_ref = - vv_HAND * dx_HAND_des

            link_collision_map = {}
            openrave.updateRobotConfiguration(q, velma.getJointStatesVectorNames())
            if True:
                openrave.switchCollisionModel("velmasimplified1")
                col_chk = openrave.env.GetCollisionChecker()
                col_opt = col_chk.GetCollisionOptions()
                col_chk.SetCollisionOptions(0x04) # CO_Contacts(0x04), CO_AllLinkCollisions(0x20)
                total_contacts = 0
                for link1_idx, link2_idx in non_adj_links_ids:
                    link1 = openrave.robot_rave.GetLinks()[link1_idx]
                    link2 = openrave.robot_rave.GetLinks()[link2_idx]
                    report = CollisionReport()
                    if col_chk.CheckCollision(link1=link1, link2=link2, report=report):
                        T_L1_W = conv.OpenraveToKDL(link1.GetTransform()).Inverse()
                        TR_L1_W = PyKDL.Frame(T_L1_W.M)
                        T_L2_W = conv.OpenraveToKDL(link2.GetTransform()).Inverse()
                        TR_L2_W = PyKDL.Frame(T_L2_W.M)
                        swapped = False
                        if link1_idx > link2_idx:
                            link1_idx, link2_idx = link2_idx, link1_idx
                            swapped = True
                        if not (link1_idx, link2_idx) in link_collision_map:
                            link_collision_map[(link1_idx, link2_idx)] = []
                        for contact in report.contacts:
                            pos_W = PyKDL.Vector(contact.pos[0], contact.pos[1], contact.pos[2])
                            norm_W = PyKDL.Vector(contact.norm[0], contact.norm[1], contact.norm[2])
                            if swapped:
                                link_collision_map[(link1_idx, link2_idx)].append( (pos_W, -norm_W, T_L2_W * pos_W, T_L1_W * pos_W, TR_L2_W * (-norm_W), TR_L1_W * (-norm_W), contact.depth) )
                            else:
                                link_collision_map[(link1_idx, link2_idx)].append( (pos_W, norm_W, T_L1_W * pos_W, T_L2_W * pos_W, TR_L1_W * norm_W, TR_L2_W * norm_W, contact.depth) )
                        total_contacts += len(report.contacts)
                col_chk.SetCollisionOptions(col_opt)

                print "links in contact:", len(link_collision_map), "total contacts:", total_contacts

            omega_col = np.matrix(np.zeros( (len(q),1) ))
            Ncol = identityMatrix(len(q))
            for link1_idx, link2_idx in link_collision_map:
                link1_name = openrave.robot_rave.GetLinks()[link1_idx].GetName()
                link2_name = openrave.robot_rave.GetLinks()[link2_idx].GetName()
                contacts = link_collision_map[ (link1_idx, link2_idx) ]
                for c in contacts:
                    pos_W, norm_W, pos_L1, pos_L2, norm_L1, norm_L2, depth = c

                    m_id = self.pub_marker.publishVectorMarker(pos_W, pos_W + norm_W*0.05, 1, 1, 0, 0, frame='world', namespace='default', scale=0.005)

                    if depth < 0:
                        print "ERROR: depth < 0:", depth
                        exit(0)

                    jac1 = PyKDL.Jacobian(len(q))
                    velma.fk_ik_solver.getJacobianForX(jac1, link1_name, pos_L1, q, iq)
                    jac2 = PyKDL.Jacobian(len(q))
                    velma.fk_ik_solver.getJacobianForX(jac2, link2_name, pos_L2, q, iq)

                    # repulsive velocity
                    V_max = 0.1
                    depth_max = 0.002
                    if depth > depth_max:
                        depth = depth_max
                    Vrep = V_max * depth * depth / (depth_max * depth_max)

                    # the mapping between motions along contact normal and the Cartesian coordinates
                    e1 = norm_L1
                    e2 = norm_L2
                    Jd1 = np.matrix([e1[0], e1[1], e1[2]])
                    Jd2 = np.matrix([e2[0], e2[1], e2[2]])
#                    print "Jd1.shape", Jd1.shape

                    # rewrite the linear part of the jacobian
                    jac1_mx = np.matrix(np.zeros( (3, len(q)) ))
                    jac2_mx = np.matrix(np.zeros( (3, len(q)) ))
                    for q_idx in range(len(q)):
                        col1 = jac1.getColumn(q_idx)
                        col2 = jac2.getColumn(q_idx)
                        for row_idx in range(3):
                            jac1_mx[row_idx, q_idx] = col1[row_idx]
                            jac2_mx[row_idx, q_idx] = col2[row_idx]

#                    print "jac1_mx.shape", jac1_mx.shape
                    Jcol1 = Jd1 * jac1_mx
                    Jcol2 = Jd2 * jac2_mx
#                    print "Jcol2.shape", Jcol2.shape

                    Jcol = np.matrix(np.zeros( (2, len(q)) ))
                    for q_idx in range(len(q)):
                        Jcol[0, q_idx] = Jcol1[0, q_idx]
                        Jcol[1, q_idx] = Jcol2[0, q_idx]

#                    print Jcol.shape
#                    print "Jcol"
#                    print Jcol
                    Jcol_pinv = np.linalg.pinv(Jcol)
#                    print "Jcol_pinv"
#                    print Jcol_pinv

#                    Ncol1 = identityMatrix(len(q)) - np.linalg.pinv(Jcol1) * Jcol1
#                    Ncol2 = identityMatrix(len(q)) - np.linalg.pinv(Jcol2) * Jcol2
#                    Ncol = Ncol * Ncol1
#                    Ncol = Ncol * Ncol2
#                    omega_col += np.linalg.pinv(Jcol1) * (-Vrep)
#                    omega_col += np.linalg.pinv(Jcol2) * (Vrep)
#                    continue

#                    activation = min(1.0, depth/0.001)
#                    a_des = np.matrix(np.zeros( (len(q),len(q)) ))
#                    a_des[0,0] = a_des[1,1] = 1.0#activation

#                    U, S, V = numpy.linalg.svd(Jcol, full_matrices=True, compute_uv=True)

#                    print "V"
#                    print V
#                    print "S"
#                    print S

                    Ncol12 = identityMatrix(len(q)) - Jcol_pinv * Jcol
#                    Ncol12 = identityMatrix(len(q)) - Jcol.transpose() * (Jcol_pinv).transpose()
#                    Ncol12 = identityMatrix(len(q)) - (V * a_des * V.transpose())
#                    Ncol12 = identityMatrix(len(q)) - (Jcol.transpose() * a_des * Jcol)
                    Ncol = Ncol * Ncol12
                    d_omega = Jcol_pinv * np.matrix([-Vrep, Vrep]).transpose()
                    print "d_omega", d_omega
                    print "Vrep", Vrep
                    print "Jcol", Jcol
                    print "Jcol_pinv", Jcol_pinv
#                    print "a_des", a_des

                    omega_col += d_omega

                    print "depth", depth
                    raw_input(".")

            print "omega_col", omega_col
#            print dx_HAND_ref

            omega_HAND = (J_HAND_inv * np.matrix(dx_HAND_ref).transpose())

            omega = J_JLC_inv * np.matrix(dx_JLC_ref).transpose() + N_JLC.transpose() * (omega_col + Ncol * omega_HAND)
            print "omega", omega

#            print "dx_JLC_ref"
#            print dx_JLC_ref
#            print "dx_HAND_ref"
#            print dx_HAND_ref

            omega_vector = np.empty(len(q))
            for q_idx in range(len(q)):
                omega_vector[q_idx] = omega[q_idx][0]
            q += omega_vector * 0.001

            
            if time_elapsed.to_sec() > 0.2:
                last_time = rospy.Time.now()
                velma.moveJoint(q, velma.getJointStatesVectorNames(), 0.05, start_time=0.14, stamp=None)

#            rospy.sleep(0.01)



if __name__ == '__main__':

    rospy.init_node('test_hierarchy_control')

    task = TestHierarchyControl()
    rospy.sleep(0.5)

    task.spin()


