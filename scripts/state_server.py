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
import rospkg
from visualization_msgs.msg import *
from geometry_msgs.msg import *

from velma import Velma
import conversions as conv
import objectstate
import velmautils
import state_server_msgs.srv

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

import openravepy
from openravepy import *
import openraveinstance
import struct
import binascii

class StateServer:

    def __init__(self):
        pass

    def handle_change_state(self, req):
        print "handle_change_state", req.cmd
        cmd = req.cmd.split()
        if cmd[0] == "grasp":
            link_name = cmd[1]
            object_name = cmd[2]
            T_W_O = conv.OpenraveToKDL( self.openrave.env.GetKinBody(object_name).GetTransform() )
            T_W_L = conv.OpenraveToKDL( self.openrave.robot_rave.GetLink(link_name).GetTransform() )
            T_L_O = T_W_L.Inverse() * T_W_O
            self.grasped[link_name] = (object_name, T_L_O)
        if cmd[0] == "drop":
            link_name = cmd[1]
            if link_name in self.grasped:
                del self.grasped[link_name]
            else:
                print "ERROR: drop for link that does not grab anything", link_name
        return state_server_msgs.srv.ChangeStateResponse(True)

    def spin(self):

        self.pub_marker = velmautils.MarkerPublisher()

        rospack = rospkg.RosPack()
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        obj_filenames = [
        rospack.get_path('velma_scripts') + '/data/jar/jar.kinbody.xml',
#        rospack.get_path('velma_scripts') + '/data/jar/jar_collision.kinbody.xml',
        ]

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        print "done."

        self.listener = tf.TransformListener()
        rospy.sleep(0.5)
#        self.br = tf.TransformBroadcaster()

        #
        # Initialise Openrave
        #
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenrave(viewer=True)

        self.openrave = openrave

        openrave.runOctomapServer()
        collision_models_urdf = {
        "velmasimplified0" : ("velma_simplified.srdf", False, True, 0.0, False),
        }
        openrave.readRobot(srdf_path=srdf_path, collision_models_urdf=collision_models_urdf)

        mo_state = objectstate.MovableObjectsState(openrave.env, obj_filenames)

        for filename in obj_filenames:
            body = openrave.env.ReadKinBodyXMLFile(filename)
            body.Enable(True)
            body.SetVisible(True)
            openrave.env.Add(body)

        openrave.addMaskedObjectToOctomap("velmasimplified0");

        self.grasped = {}
        s = rospy.Service('change_state', state_server_msgs.srv.ChangeState, self.handle_change_state)

        counter = 0
        while not rospy.is_shutdown():
            time_now, js = velma.getLastJointState()
            openrave.updateRobotConfigurationRos(js)

            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            added, removed = mo_state.update(self.listener, time_tf)
            for link_name in self.grasped:
                object_name, T_L_O = self.grasped[link_name]
                if object_name in mo_state.obj_map:
                    T_W_L = conv.OpenraveToKDL(openrave.robot_rave.GetLink(link_name).GetTransform())
                    mo_state.obj_map[object_name].T_W_O = T_W_L * T_L_O

            mo_state.updateOpenrave(openrave.env)

            for obj_name in added:
                openrave.addMaskedObjectToOctomap(obj_name)
            for obj_name in removed:
                openrave.removeMaskedObjectFromOctomap(obj_name)

#            mo_state.updateOpenrave(openrave.env)
            if counter % 10 ==0:
                m_id = 0
                # publish markers
                for obj_name in mo_state.obj_map:
                    obj = mo_state.obj_map[obj_name]
                    body = openrave.env.GetKinBody(obj_name)
                    for link in body.GetLinks():
                        T_W_L = conv.OpenraveToKDL(link.GetTransform())
                        for geom in link.GetGeometries():
                            T_L_G = conv.OpenraveToKDL(geom.GetTransform())
                            g_type = geom.GetType()
    #                        print dir(openravepy_int.GeometryType)
                            if g_type == openravepy_int.GeometryType.Cylinder:
                                radius = geom.GetCylinderRadius()
                                height = geom.GetCylinderHeight()
                                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=1, namespace='state_server_objects', frame_id='world', m_type=Marker.CYLINDER, scale=Vector3(radius*2, radius*2, height), T=T_W_L*T_L_G)
                self.pub_marker.eraseMarkers(m_id, 1000, frame_id='world', namespace='state_server_objects')
                for obj_name in mo_state.obj_map:
                    openrave.UpdateMaskedObjectInOctomap(obj_name)

            rospy.sleep(0.01)
            counter += 1
            if counter >= 100:
                counter = 0

if __name__ == '__main__':

    rospy.init_node('state_server')
    v = StateServer()
    v.spin()

