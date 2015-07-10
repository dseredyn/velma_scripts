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

        rospy.sleep(0.5)
        velma.switchToJoint()

        velma.initSolvers()
        rospy.sleep(0.5)

        lim_bo_soft, lim_up_soft = velma.getJointSoftLimitsVectors()
        lim_bo, lim_up = velma.getJointLimitsVectors()

        velma.fk_ik_solver.createJacobianSolver('torso_base', 'right_HandPalmLink', velma.getJointStatesVectorNames())

        x_HAND_targets = [PyKDL.Frame(PyKDL.Vector(0.5,0,1.8)),
        PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.2,-0.5,1.0))]
        target_idx = 0
        x_HAND_target = x_HAND_targets[target_idx]
        target_idx += 1

        last_time = rospy.Time.now()
        q = velma.getJointStatesVector()

        counter = 0
        while not rospy.is_shutdown():
            if counter > 300:
                x_HAND_target = x_HAND_targets[target_idx]
                target_idx = (target_idx + 1)%2
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

#            print delta_V_JLC
#            print "J_JLC"
#            print J_JLC

            J_JLC_inv = np.linalg.pinv(J_JLC)
#            print "J_JLC_inv"
#            print J_JLC_inv

            N_JLC = identityMatrix(len(q)) - (J_JLC_inv * J_JLC)
#            print "N_JLC", N_JLC.shape
#            print N_JLC

            J_HAND = velma.fk_ik_solver.getJacobian('torso_base', 'right_HandPalmLink', q).transpose()
            J_HAND_inv = np.linalg.pinv(J_HAND)
#            print "J_HAND", J_HAND.shape
#            print J_HAND

            v_max_JLC = 10.0/180.0*math.pi
            kp_JLC = 1.0
            dx_JLC_des = kp_JLC * delta_V_JLC

            vv_JLC = min(1.0, v_max_JLC/np.linalg.norm(dx_JLC_des))
            dx_JLC_ref = - vv_JLC * dx_JLC_des

#            print dx_JLC_ref

            velma.updateTransformations()

            x_HAND_current = velma.T_B_W * velma.T_W_E
            x_HAND_diff = PyKDL.diff(x_HAND_target, x_HAND_current)
            delta_V_HAND = np.empty(6)
            delta_V_HAND[0] = x_HAND_diff.vel[0]
            delta_V_HAND[1] = x_HAND_diff.vel[1]
            delta_V_HAND[2] = x_HAND_diff.vel[2]
            delta_V_HAND[3] = x_HAND_diff.rot[0]
            delta_V_HAND[4] = x_HAND_diff.rot[1]
            delta_V_HAND[5] = x_HAND_diff.rot[2]

            v_max_HAND = 0.05
            kp_HAND = 1.0
            dx_HAND_des = kp_HAND * delta_V_HAND

            vv_HAND = min(1.0, v_max_HAND/np.linalg.norm(dx_HAND_des))
            print vv_JLC, vv_HAND
            dx_HAND_ref = - vv_JLC * dx_HAND_des
#            print dx_HAND_ref
#            print "np.matrix(dx_HAND_ref).transpose()"
#            print np.matrix(dx_HAND_ref).transpose()
            omega = J_JLC_inv * np.matrix(dx_JLC_ref).transpose()
            omega_HAND = (J_HAND_inv * np.matrix(dx_HAND_ref).transpose())
#            print "omega_HAND", omega_HAND.shape
#            print omega_HAND
            omega += N_JLC.transpose() * omega_HAND

#            print "dx_JLC_ref"
#            print dx_JLC_ref
#            print "dx_HAND_ref"
#            print dx_HAND_ref
#            print omega

#            raw_input("Press ENTER to continue...")
#            exit(0)

            omega_vector = np.empty(len(q))
            for q_idx in range(len(q)):
                omega_vector[q_idx] = omega[q_idx][0]
#            print omega_vector
            q += omega_vector * 0.01

            
            if time_elapsed.to_sec() > 0.2:
                last_time = rospy.Time.now()
                velma.moveJoint(q, velma.getJointStatesVectorNames(), 0.05, start_time=0.14, stamp=None)

#            print q
            rospy.sleep(0.01)



if __name__ == '__main__':

    rospy.init_node('test_hierarchy_control')

    task = TestHierarchyControl()
    rospy.sleep(0.5)

    task.spin()


