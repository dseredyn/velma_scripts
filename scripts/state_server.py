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

import sensor_msgs.msg
import geometry_msgs.msg
import actionlib
import actionlib_msgs.msg
import cartesian_trajectory_msgs.msg
import barrett_hand_controller_srvs.msg
import barrett_hand_controller_srvs.srv
import controller_manager_msgs.srv
import std_srvs.srv
import control_msgs.msg
import rospkg
from velma import Velma
import conversions as conv

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

import copy

import urdf_parser_py.urdf
from openravepy import *
import openraveinstance
import struct
import binascii

class ObjectState:
    def __init__(self):
        self.kinbody = None
        self.masked = False
        self.last_update = None
        self.T_W_O = None

class StateServer:

    def __init__(self):
        pass

    def spin(self):

        rospack = rospkg.RosPack()
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        obj_filenames = [
        rospack.get_path('velma_scripts') + '/data/jar/jar.kinbody.xml'
        ]

        obj_list = []

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        print "done."

        self.listener = tf.TransformListener()
#        rospy.sleep(0.5)
#        self.br = tf.TransformBroadcaster()

        #
        # Initialise Openrave
        #
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenrave(viewer=True)

        openrave.runOctomapServer()
#        openrave.env.GetCollisionChecker().SetCollisionOptions(0)#4)
        collision_models_urdf = {
        "velmasimplified0" : ("velma_simplified.srdf", False, True, 0.0, False),
        }
        openrave.readRobot(srdf_path=srdf_path, collision_models_urdf=collision_models_urdf)

        for filename in obj_filenames:
            obj = ObjectState()
            obj.kinbody = openrave.readBodyFromFile(filename)
            obj_list.append(obj)

        openrave.addMaskedObjectToOctomap("velmasimplified0");

        while not rospy.is_shutdown():
            time_now, js = velma.getLastJointState()
            openrave.updateRobotConfigurationRos(js)

            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            for obj in obj_list:
                obj_name = obj.kinbody.GetName()
                if self.listener.canTransform('world', obj_name, time_tf):
                    pose = self.listener.lookupTransform('world', obj_name, time_tf)
                    obj.T_W_O = pm.fromTf(pose)
                    if not obj.masked:
                        obj.kinbody.SetTransform(conv.KDLToOpenrave(obj.T_W_O))
                        openrave.env.Add(obj.kinbody)
                        openrave.addMaskedObjectToOctomap(obj_name);
                        obj.masked = True
                    obj.last_update = time_tf
                else:
                    if obj.masked:
                        if (time_tf - obj.last_update).to_sec() > 2.0:
                            openrave.removeMaskedObjectFromOctomap(obj_name);
                            openrave.env.Remove(obj.kinbody)
                            obj.masked = False

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('state_server')
    v = StateServer()
    v.spin()

