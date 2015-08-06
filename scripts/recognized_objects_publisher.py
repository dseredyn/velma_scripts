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
import rospkg
import ar_track_alvar_msgs.msg
from visualization_msgs.msg import *
from geometry_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

class ROP:

    def __init__(self):
        pass

    def markerCallback(self, msg):
        for marker in msg.markers:
            if marker.id == 0:
                pos = PyKDL.Vector(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
                ori = PyKDL.Rotation.Quaternion(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)
                T_C_M = PyKDL.Frame(ori, pos)
#                T_M_J = PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Vector(0,0.045, -0.04))
                T_M_J = PyKDL.Frame(PyKDL.Vector(0,-0.04, -0.045))
                T_C_J = T_C_M * T_M_J
                pose = pm.toMsg(T_C_J)
                self.tf_br.sendTransform([pose.position.x, pose.position.y, pose.position.z], [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], rospy.Time.now(), "jar", marker.header.frame_id)


    def spin(self):

        self.tf_br = tf.TransformBroadcaster()
        rospy.Subscriber('/ar_pose_marker', ar_track_alvar_msgs.msg.AlvarMarkers, self.markerCallback)

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

if __name__ == '__main__':

    rospy.init_node('recognized_objects_publisher')
    v = ROP()
    v.spin()

