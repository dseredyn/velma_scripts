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
import rrt_star_connect_planner
import tree
import rosparam
import tasks

class TestOrOctomap:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):

        simulation = True

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/common/velma_room.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

# TODO: 3 processes
        rrt = rrt_star_connect_planner.PlannerRRT(0, env_file, xacro_uri, srdf_path)

        self.listener = tf.TransformListener()

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)
        print "done."


        rospy.sleep(0.5)
        self.velma.updateTransformations()

        if simulation:
            hv = [3.2, 3.2, 3.2, 3.2]
        ht = [3000, 3000, 3000, 3000]
        self.velma.moveHandLeft([30.0/180.0*math.pi, 30.0/180.0*math.pi, 30.0/180.0*math.pi, 0], hv, ht, 5000, True)
        self.velma.moveHandRight([30.0/180.0*math.pi, 30.0/180.0*math.pi, 30.0/180.0*math.pi, 0], hv, ht, 5000, True)

        rospy.sleep(1.0)

        #
        # Initialise Openrave
        #

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file)
        openrave.runOctomap()
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)
        openrave.maskObject('velmasimplified0')
        openrave.maskObject('velmasimplified1')

# TODO
        for link in openrave.robot_rave.GetLinks():
            print link.GetName(), len(link.GetCollisionData().vertices), len(link.GetGeometries())
            for geom in link.GetGeometries():
                print "   ", geom.GetType(), geom.IsVisible(), geom.GetSphereRadius()
        exit(0)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))
        openrave.updateRobotConfigurationRos(self.velma.js_pos)

#        rospy.sleep(3)
#        openrave.pauseOctomap()

        while True:
            if self.listener.canTransform('world', 'jar', rospy.Time(0)):
                pose = self.listener.lookupTransform('world', 'jar', rospy.Time(0))
                T_B_J = pm.fromTf(pose)
                break

        rrt.waitForInit()

        T_B_E_list = []
        for angle_jar_axis in np.arange(0.0, math.pi*2.0, 10.0/180.0*math.pi):
            for translation_jar_axis in np.linspace(-0.03, 0.03, 7):
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.20))
                T_B_E_list.append(T_B_Ed)
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.20))
                T_B_E_list.append(T_B_Ed)

        print "grasps:", len(T_B_E_list)

        # look around
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.8))))
        rospy.sleep(2)
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,-1,1.8))))
        rospy.sleep(2)
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,1,1.8))))
        rospy.sleep(2)
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.8))))
        rospy.sleep(2)

        openrave.maskObject('velmasimplified0')
        openrave.maskObject('velmasimplified1')

        raw_input("Press ENTER to continue...")

        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.GraspTaskRRT, ("left", T_B_E_list), 120.0)

        traj = []
        for i in range(len(path)-1):
            q1 = path[i]
            q2 = path[i+1]
            for f in np.linspace(0.0, 1.0, 40):
                traj.append( q1 * (1.0 - f) + q2 * f )

        while True:
            if raw_input("Type e to exit") == 'e':
                break
            openrave.showTrajectory(dof_names, 10.0, traj)

        rrt.cleanup()
#        raw_input("Press ENTER to exit...")
        exit(0)

if __name__ == '__main__':

    rospy.init_node('test_or_octomap')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


