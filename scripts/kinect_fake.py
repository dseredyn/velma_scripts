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

import sensor_msgs.msg
import rospkg
from velma import Velma

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np

import urdf_parser_py.urdf
from openravepy import *
import openraveinstance
import struct
import binascii

class KinectFake:

    def __init__(self):
        pass

    def spin(self):

        s = struct.Struct('f f f i B B B B i i i')
        s2 = struct.Struct('B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B')

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/jar/cabinet_jar.env.xml'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        object_name = 'jar'

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        print "done."

        self.listener = tf.TransformListener()
        rospy.sleep(0.5)

        self.br = tf.TransformBroadcaster()

        self.point_cloud_pub = rospy.Publisher("/head_kinect/depth_registered/points", sensor_msgs.msg.PointCloud2, queue_size=100)
        self.point_cloud = sensor_msgs.msg.PointCloud2()
        self.point_cloud.header.frame_id = 'head_kinect_rgb_optical_frame'
        self.point_cloud.header.seq = 0

#        self.point_cloud.height = 480
#        self.point_cloud.width = 640

        self.point_cloud.height = 60
        self.point_cloud.width = 80

        self.point_cloud.fields.append(sensor_msgs.msg.PointField('x', 0, 7, 1))
        self.point_cloud.fields.append(sensor_msgs.msg.PointField('y', 4, 7, 1))
        self.point_cloud.fields.append(sensor_msgs.msg.PointField('z', 8, 7, 1))
        self.point_cloud.fields.append(sensor_msgs.msg.PointField('rgb', 16, 7, 1))
        self.point_cloud.is_bigendian = False
        self.point_cloud.point_step = 32
        self.point_cloud.row_step = self.point_cloud.point_step * self.point_cloud.width
        self.point_cloud.is_dense = False

        #
        # Initialise Openrave
        #
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenrave(viewer=False)
#        openrave.runOctomapServer()
        openrave.loadEnv(env_file)
#        openrave.env.GetCollisionChecker().SetCollisionOptions(0)#4)
        collision_models_urdf = {
        "velmasimplified0" : ("velma_simplified.srdf", False, True, 0.0, False),
        }
        openrave.readRobot(srdf_path=srdf_path, collision_models_urdf=collision_models_urdf)

#        openrave.addMaskedObjectOctomap("velmasimplified0");

        while not rospy.is_shutdown():
            time_now, js = velma.getLastJointState()
            T_B_C = velma.fk_ik_solver.calculateFk('head_kinect_rgb_optical_frame', js)

            openrave.updateRobotConfigurationRos(js)
            rospy.sleep(0.1)
            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            if self.listener.canTransform('world', 'torso_base', time_tf):
                pose = self.listener.lookupTransform('world', 'torso_base', time_tf)
            else:
                print 'cannot transform'
                continue

            T_W_B = pm.fromTf(pose)
            T_W_C = T_W_B * T_B_C
            T_C_W = T_W_C.Inverse()

            valid_points = 0
            self.point_cloud.data = []
            for y in range(self.point_cloud.height):
                fy = float(y)/float(self.point_cloud.height)-0.5
                for x in range(self.point_cloud.width):
                    fx = float(x)/float(self.point_cloud.width)-0.5

                    origin = T_W_C * PyKDL.Vector()
                    d = T_W_C * PyKDL.Vector(fx, fy, 1.0) - origin
                    d *= 4.0
                    report = CollisionReport()
                    ret = openrave.env.CheckCollision(Ray([origin.x(),origin.y(),origin.z()],[d.x(),d.y(),d.z()]), report)
                    if len(report.contacts) > 0:
                        x_W, y_W, z_W = report.contacts[0].pos
                        p_C = T_C_W * PyKDL.Vector(x_W, y_W, z_W)
                        values = ( p_C.x(), p_C.y(), p_C.z(), 0, 128, 128, 128, 255, 0, 0, 0 )
                        packed_data = s.pack(*values)
                        unpacked = s2.unpack(packed_data)
                        for p in unpacked:
                            self.point_cloud.data.append(p)
                        valid_points += 1
                    else:
                        for i in range(32):
                            self.point_cloud.data.append(255)

            self.point_cloud.header.seq += 1
            self.point_cloud.header.stamp = time_now
            self.point_cloud_pub.publish(self.point_cloud)

            T_W_J = openrave.getPoseW(object_name)

            T_msg = pm.toMsg(T_W_J)
            self.br.sendTransform([T_msg.position.x, T_msg.position.y, T_msg.position.z], [T_msg.orientation.x, T_msg.orientation.y, T_msg.orientation.z, T_msg.orientation.w], rospy.Time.now(), object_name, "world")

if __name__ == '__main__':

    rospy.init_node('kinect')
    v = KinectFake()
    v.spin()

