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

from threading import Lock

class PointCloudFilter:

    def __init__(self):
        self.mutex = Lock()
        self.processsed = True
        self.pointcloud_orig = None


    def pc_callback(self, data):
        self.mutex.acquire()
        if self.processsed:
            self.pointcloud_orig = copy.deepcopy(data)
            self.processsed = False
        self.mutex.release()

    def spin(self):

        s = struct.Struct('f f f i B B B B i i i')
        s2 = struct.Struct('B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B')

        rospack = rospkg.RosPack()
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        self.listener = tf.TransformListener()
#        rospy.sleep(0.5)

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        rospy.Subscriber("/head_kinect/depth_registered/points", sensor_msgs.msg.PointCloud2, self.pc_callback)
        self.point_cloud_pub = rospy.Publisher("/pointcloud_filter/points", sensor_msgs.msg.PointCloud2, queue_size=100)
        pointcloud = sensor_msgs.msg.PointCloud2()

        #
        # Initialise Openrave
        #
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=None, viewer=False, collision='fcl')

        collision_models_urdf = {
        "velmasimplified0" : ("velma_simplified.srdf", False, True, 0.0, False),
        }
        openrave.readRobot(srdf_path=srdf_path, collision_models_urdf=collision_models_urdf)
        sphere_probe = openrave.addSphere('sphere_probe', 0.1)

        while not rospy.is_shutdown():
            self.mutex.acquire()
            processed = self.processsed
            self.mutex.release()

            if not processed:
                time_stamp = self.pointcloud_orig.header.stamp
                if self.listener.canTransform('world', self.pointcloud_orig.header.frame_id, time_stamp):
                    pose = self.listener.lookupTransform('world', self.pointcloud_orig.header.frame_id, time_stamp)
                    T_W_C = pm.fromTf(pose)
                    T_C_W = T_W_C.Inverse()
                else:
                    print 'cannot transform'
                    continue

                openrave.updateRobotConfigurationRos(self.velma.getJointStateAtTime(time_stamp))

                pointcloud.header = self.pointcloud_orig.header
                pointcloud.height = self.pointcloud_orig.height
                pointcloud.width = self.pointcloud_orig.width
                pointcloud.fields = self.pointcloud_orig.fields
                pointcloud.is_bigendian = self.pointcloud_orig.is_bigendian
                pointcloud.point_step = self.pointcloud_orig.point_step
                pointcloud.row_step = self.pointcloud_orig.row_step
                pointcloud.is_dense = self.pointcloud_orig.is_dense

#                print pointcloud.fields

                pointcloud.data = []
                for idx in range(0, len(self.pointcloud_orig.data), pointcloud.point_step):
                    packed = self.pointcloud_orig.data[idx:(idx+pointcloud.point_step)]
                    unpacked = s.unpack(packed)
                    if math.isnan(unpacked[0]) or math.isnan(unpacked[1]) or math.isnan(unpacked[3]):
                        unpacked2 = s2.unpack(packed)
                        for i in range(32):
                            pointcloud.data.append(unpacked2[i])
                        continue
                    x, y, z = unpacked[0:3]
                    pt_C = PyKDL.Vector(x,y,z)
                    pt_W = T_W_C * pt_C
                    sphere_probe.SetTransform(conv.KDLToOpenrave(PyKDL.Frame(pt_W)))
                    report = CollisionReport()
 	            if openrave.env.CheckCollision(sphere_probe, report):
                        for i in range(32):
                            pointcloud.data.append(255)
                    else:
                        unpacked2 = s2.unpack(packed)
                        for i in range(32):
                            pointcloud.data.append(unpacked2[i])

                self.point_cloud_pub.publish(pointcloud)

                self.mutex.acquire()
                self.processsed = True
                self.mutex.release()

            rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('pointcloud_filter')
    v = PointCloudFilter()
    v.spin()

