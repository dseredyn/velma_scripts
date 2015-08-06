#!/usr/bin/env python

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

import roslib
roslib.load_manifest('velma_scripts')

import rospy
import tf

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

from velma import Velma
import velmautils
import pose_lookup_table_left as plutl
import pose_lookup_table_right as plutr
import velma_fk_ik

if __name__ == '__main__':

    # literature:
    # https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/PyKDL/kinfam.sip
    # http://wiki.ros.org/pykdl_utils
    # https://github.com/gt-ros-pkg/hrl-kdl/blob/hydro/pykdl_utils/src/pykdl_utils/kdl_parser.py
    # http://people.mech.kuleuven.be/~rsmits/kdl/api/html/annotated.html
    # http://people.mech.kuleuven.be/~rsmits/kdl/api/html/classKDL_1_1ChainIkSolverPos__NR__JL.html

    # to read:
    # https://github.com/benersuay/rmaps
    # http://www.dlr.de/rmc/rm/Portaldata/52/Resources/images/institute/robotersysteme/rollin_justin/mobile_manipulation/leidner2014object.pdf

    pub_marker = velmautils.MarkerPublisher()

    side = "left"

    rospy.init_node('velma_ik_draw_workspace')
    rospy.sleep(1)

    print "creating interface for Velma..."
    # create the interface for Velma robot
    velma = Velma()
    print "done."
    rospy.sleep(1)
    velma.updateTransformations()

    r = 147.0/256.0
    g = 80.0/256.0
    b = 0.0/256.0
    table_pos = PyKDL.Vector(0.7,0,0.8)
    # draw a table
#    pub_marker.publishSinglePointMarker(table_pos, i, r=r, g=g, b=b, a=1, namespace='default', frame_id="world", m_type=Marker.CUBE, scale=Vector3(0.8, 1.4, 0.05))
#    i += 1
#    pub_marker.publishSinglePointMarker(table_pos+PyKDL.Vector(0.35,0.65,-0.4), i, r=r, g=g, b=b, a=1, namespace='default', frame_id="world", m_type=Marker.CUBE, scale=Vector3(0.05, 0.05, 0.75))
#    i += 1
#    pub_marker.publishSinglePointMarker(table_pos+PyKDL.Vector(-0.35,0.65,-0.4), i, r=r, g=g, b=b, a=1, namespace='default', frame_id="world", m_type=Marker.CUBE, scale=Vector3(0.05, 0.05, 0.75))
#    i += 1
#    pub_marker.publishSinglePointMarker(table_pos+PyKDL.Vector(-0.35,-0.65,-0.4), i, r=r, g=g, b=b, a=1, namespace='default', frame_id="world", m_type=Marker.CUBE, scale=Vector3(0.05, 0.05, 0.75))
#    i += 1
#    pub_marker.publishSinglePointMarker(table_pos+PyKDL.Vector(0.35,-0.65,-0.4), i, r=r, g=g, b=b, a=1, namespace='default', frame_id="world", m_type=Marker.CUBE, scale=Vector3(0.05, 0.05, 0.75))
#    i += 1

    if side == "left":
        plut = plutl
        i = 10000
    else:
        plut = plutr
        i = 0
    print "ok 2"

    listener = tf.TransformListener();
    rospy.sleep(1)

    pose = listener.lookupTransform('torso_base', 'torso_link2', rospy.Time(0))
    T_B_T2 = pm.fromTf(pose)

    pose = listener.lookupTransform('torso_link2', side+'_arm_2_link', rospy.Time(0))
    T_T2_L2 = pm.fromTf(pose)

    pt_c_in_T2 = T_T2_L2 * PyKDL.Vector()
    max_dist = 0.0
    min_dist = 10000.0
    x_i = 0
    print "len: %s %s %s"%(len(plut.x_set), len(plut.y_set), len(plut.z_set))
    for coord in plut.lookup_table:
            break
            if rospy.is_shutdown():
                break
            xi, yi, zi = coord
            if xi != 1:
                continue
            if yi != 4:
                continue
            print zi
            if zi != 10 and zi != 9:
                continue
            x = plut.x_set[xi]
            y = plut.y_set[yi]
            z = plut.z_set[zi]

            rot_set = plut.lookup_table[coord]

            pt_B = T_B_T2 * PyKDL.Vector(x,y,z)
            l = len(rot_set)
            if l > 0:
                dist = (pt_c_in_T2.x()-x)*(pt_c_in_T2.x()-x) + (pt_c_in_T2.y()-y)*(pt_c_in_T2.y()-y) + (pt_c_in_T2.z()-z)*(pt_c_in_T2.z()-z)
                if dist > max_dist:
                    max_dist = dist
                if dist < min_dist:
                    min_dist = dist
                size = float(l)/float(len(plut.rotations)-1)
                pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), i, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.08*size, 0.08*size, 0.08*size))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
                i += 1
                pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), i, r=0, g=0, b=1, a=0.5, namespace='default', frame_id="torso_link2", m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.08))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
                i += 1

#                pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,-z), i, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.05*size, 0.05*size, 0.05*size))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
#                i += 1
#            raw_input(".")
            rospy.sleep(0.001)

    def publishCoord(coord, m_id):
        size = float(len(plut.lookup_table[coord]))/float(len(plut.rotations)-1)
        pos = PyKDL.Vector(plut.x_set[coord[0]],plut.y_set[coord[1]],plut.z_set[coord[2]])
        m_id = pub_marker.publishSinglePointMarker(pos, m_id, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.08*size, 0.08*size, 0.08*size))
        m_id = pub_marker.publishSinglePointMarker(pos, m_id, r=0, g=0, b=1, a=0.5, namespace='default', frame_id="torso_link2", m_type=Marker.CUBE, scale=Vector3(0.003, 0.003, 0.08))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
        return m_id

    def getPoseT2(coord, rot_idx):
        return PyKDL.Frame(plut.rotations[rot_idx], PyKDL.Vector(plut.x_set[coord[0]], plut.y_set[coord[1]], plut.z_set[coord[2]]))

    limit_submap = {
    "left_arm_1_joint" : (15.0/180.0*math.pi, 2.09),
    "left_arm_3_joint" : (-2.09, -15.0/180.0*math.pi),
    "left_arm_5_joint" : (15.0/180.0*math.pi, 2.09),
    }
    fkik = velma_fk_ik.VelmaFkIkSolver([], None, limit_submap=limit_submap)

    T_B_El = velma.T_B_Wl * velma.T_Wl_El
    T_T2_El = velma.T_T2_B * T_B_El
    x_idx = plut.getIdxX(T_T2_El.p.x())
    y_idx = plut.getIdxY(T_T2_El.p.y())
    z_idx = plut.getIdxZ(T_T2_El.p.z())
    coord = (x_idx, y_idx, z_idx)
    closest_rot = plut.getClosestRot(T_T2_El)
    print coord, closest_rot
    m_id = 0
    m_id = publishCoord(coord, m_id)

#    coord2 = (x_idx, y_idx, z_idx+1)
#    m_id = publishCoord(coord2, m_id)

    joint_names = [
    "left_arm_0_joint",
    "left_arm_1_joint",
    "left_arm_2_joint",
    "left_arm_3_joint",
    "left_arm_4_joint",
    "left_arm_5_joint",
    "left_arm_6_joint",
    ]
    print "simulating motion to the closest pose..."
    T_B_E1 = velma.T_B_T2 * getPoseT2(coord, closest_rot)
    q = fkik.simulateTrajectory("left_HandPalmLink", velma.js_pos, T_B_E1)
    print q
    velma.switchToJoint()
    velma.moveJoint(q, joint_names, 1.0, start_time=0.2)
    velma.waitForJoint()
    rospy.sleep(1.0)
    velma.updateTransformations()

    print "simulating motion to the all rotations of the closest point..."
    for rot_idx in plut.lookup_table[coord]:
        print "rot_idx", rot_idx
        T_B_E2 = velma.T_B_T2 * getPoseT2(coord, rot_idx)

        pub_marker.publishFrameMarker(T_B_E2, m_id, scale=0.1, frame='torso_base', namespace='default')
        q = fkik.simulateTrajectory("left_HandPalmLink", velma.js_pos, T_B_E2)
        print q
        raw_input(".")

#        diff = PyKDL.diff(T_B_El, T_B_E2)
#        for t in np.linspace(0.0, 1.0, 100):
#            T_B_Ed = PyKDL.addDelta(T_B_El, diff, t)
#            pub_marker.publishFrameMarker(T_B_Ed, m_id, scale=0.1, frame='torso_base', namespace='default')
#            q = fkik.simulateTrajectory("left_HandPalmLink", velma.js_pos, T_B_Ed)
#            print q
#            raw_input(".")
        
    rospy.sleep(1)

    exit(0)

    coord1 = (1,4,9)
    rot_set = plut.lookup_table[ coord1 ]
    print rot_set
    for rot_idx in rot_set:
        break
    T_B_E1 = velma.T_B_T2 * PyKDL.Frame(plut.rotations[rot_idx], PyKDL.Vector(plut.x_set[coord1[0]], plut.y_set[coord1[1]], plut.z_set[coord1[2]]))
    q = fkik.simulateTrajectory("left_HandPalmLink", velma.js_pos, T_B_E1)
    print q

    print "done."

