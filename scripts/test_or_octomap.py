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
import operator
import rospkg

import velmautils
import openraveinstance
import rosparam

class TestOrOctomap:
    """

"""

    def KDLToOpenrave(self, T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):

        #
        # Initialise Openrave
        #

        rospack = rospkg.RosPack()
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF()
        openrave.readRobot(xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro', srdf_uri=rospack.get_path('velma_description') + '/robots/velma.srdf')

        rospy.sleep(1)
        openrave.runOctomap()

        sphere = RaveCreateKinBody(openrave.env,'')
        sphere.SetName("sphere")
        sphere.InitFromSpheres(numpy.array([[0,0,0,0.1]]),True)
        openrave.env.Add(sphere,True)

        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0,1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0,1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0,-1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0,-1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))

        ob = openrave.env.GetKinBody("_OCTOMAP_MAP_")
        cc = openrave.env.GetCollisionChecker()

        for x in np.linspace(-2,2,10):
            for y in np.linspace(-2,2,10):
                for z in np.linspace(0,2,10):
                    print x,y,z
                    tr = self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,y,z)))
                    sphere.SetTransform(tr)
                    openrave.env.UpdatePublishedBodies()
                    report = CollisionReport()
                    ret = cc.CheckCollision(sphere, report)
#                    ret = openrave.env.CheckCollision(ob, report)
                    print ret
                    continue
                    if report.plink1 == None:
                        print None
                    else:
                        print report.plink1.GetParent().GetName(), report.plink2.GetName() 
#                        print "   ", report.vLinkColliding
                        for link1, link2 in report.vLinkColliding:
                            print "   ", link1.GetParent().GetName(), link2.GetName()
#                        print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 


        raw_input(".")

        exit(0)

if __name__ == '__main__':

    rospy.init_node('test_or_octomap')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


