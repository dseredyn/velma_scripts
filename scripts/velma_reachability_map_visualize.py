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

import velmautils
import pose_lookup_table_left as plutl
import pose_lookup_table_right as plutr

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

    r = 147.0/256.0
    g = 80.0/256.0
    b = 0.0/256.0
    table_pos = PyKDL.Vector(0.7,0,0.8)
    i = 0
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
    else:
        plut = plutr
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
            if rospy.is_shutdown():
                break
            xi, yi, zi = coord
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
                size = float(l)/40.0
                pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), i, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.05*size, 0.05*size, 0.05*size))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
                i += 1
#                pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,-z), i, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.05*size, 0.05*size, 0.05*size))#, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
#                i += 1
            rospy.sleep(0.001)
    print i
    print math.sqrt(min_dist)
    print math.sqrt(max_dist)
    rospy.sleep(3)

    print "done."

