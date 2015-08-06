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

import conversions as conv
import velmautils

class TestQ5Q6:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):
            from sensor_msgs.msg import JointState
            js = {}
            def jsCallback(data):
                joint_idx = 0
                for joint_name in data.name:
                    js[joint_name] = data.position[joint_idx]
                    joint_idx += 1

            joint_states_listener = rospy.Subscriber('/joint_states', JointState, jsCallback)

            tab2=[
            [-0.397855401039,-2.90307354927],
            [2.12894010544,-2.90307354927],
            [2.12043237686,-1.87363839149],
            [1.92475450039,-1.43123674393],
            [0.77621114254,-1.39720571041],
            [0.350824713707,-1.00585031509],
            [0.401871085167,-0.571956157684],
            [0.810242056847,0.414940297604],
            [1.34622907639,0.942419290543],
            [2.11192464828,1.01898884773],
            [2.12894010544,2.8906891346],
            [-0.814733862877,2.8906891346],
            [-1.22310483456,2.27813267708],
            [-2.21850919724,2.29514837265],
            [-2.22701668739,-1.32063627243],
            [-1.81013822556,-1.66945314407],
            [-0.814733862877,-1.73751521111],
            [-0.423378348351,-2.09483933449],
            ]
            m_id = 0
            for pt in tab2:
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(pt[0], pt[1], 0.0), m_id, r=1, g=0, b=0, m_type=Marker.SPHERE, frame_id='world', namespace='edges', scale=Vector3(0.05, 0.05, 0.05))
                rospy.sleep(0.01)

            while not rospy.is_shutdown():
                rospy.sleep(0.01)
                if not "left_arm_5_joint" in js or not "left_arm_6_joint" in js:
                    continue
#                q5 = -js["left_arm_5_joint"]
#                q6 = -js["left_arm_6_joint"]
                q5 = js["right_arm_5_joint"]
                q6 = js["right_arm_6_joint"]
                print velmautils.point_inside_polygon(q5,q6,tab2)
                self.pub_marker.publishSinglePointMarker(PyKDL.Vector(q5, q6, 0.0), m_id, r=1, g=1, b=1, m_type=Marker.SPHERE, frame_id='world', namespace='edges', scale=Vector3(0.05, 0.05, 0.05))
                
if __name__ == '__main__':

    rospy.init_node('q5q6_self_collision_test')

    task = TestQ5Q6()
    rospy.sleep(1)

    task.spin()


