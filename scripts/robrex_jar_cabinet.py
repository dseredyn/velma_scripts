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

        if True:
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
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(pt[0], pt[1], 0.0), m_id, r=1, g=1, b=1, m_type=Marker.SPHERE, frame_id='world', namespace='edges', scale=Vector3(0.05, 0.05, 0.05))
                rospy.sleep(0.01)
            exit(0)

        simulation = True

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/common/velma_room.env.xml'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        rrt = rrt_star_connect_planner.PlannerRRT(3, env_file, srdf_path)

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
        openrave.startOpenraveURDF(env_file=env_file, collision='fcl')
        openrave.runOctomap()

        openrave.readRobot(srdf_path=srdf_path)

# TODO
#        for link in openrave.robot_rave.GetLinks():
#            print link.GetName(), len(link.GetCollisionData().vertices), len(link.GetGeometries())
#            for geom in link.GetGeometries():
#                print "   ", geom.GetType(), geom.IsVisible(), geom.GetSphereRadius(), len(geom.GetCollisionMesh().vertices), geom.GetBoxExtents()
#        exit(0)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))
        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        while True:
            if self.listener.canTransform('torso_base', 'jar', rospy.Time(0)):
                pose = self.listener.lookupTransform('torso_base', 'jar', rospy.Time(0))
                T_B_J = pm.fromTf(pose)
                break

        print "waiting for planner..."
        rrt.waitForInit()
        print "planner initialized"

        T_B_E_list = []
        for angle_jar_axis in np.arange(0.0, math.pi*2.0, 10.0/180.0*math.pi):
            for translation_jar_axis in np.linspace(-0.03, 0.03, 7):
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.20))
                T_B_E_list.append(T_B_Ed)
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.20))
                T_B_E_list.append(T_B_Ed)

        print "grasps:", len(T_B_E_list)

        if False:
            # look around
            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.8))))
            rospy.sleep(2)
            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,-1,1.8))))
            rospy.sleep(2)
            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,1,1.8))))
            rospy.sleep(2)
            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.8))))
            rospy.sleep(2)

        raw_input("Press ENTER to continue...")

        openrave.updateOctomap()

        print "octomap updated"

        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.GraspTaskRRT, ("left", T_B_E_list), 60.0)

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

        traj = self.velma.prepareTrajectory(path, self.velma.getJointStatesByNames(dof_names))
        self.velma.switchToJoint()
        self.velma.moveJointTraj(traj, dof_names, start_time=0.2)

        raw_input("Press ENTER to continue...")

        path.reverse()
        traj = self.velma.prepareTrajectory(path, self.velma.getJointStatesByNames(dof_names))
        self.velma.moveJointTraj(traj, dof_names, start_time=0.2)

        raw_input("Press ENTER to continue...")

        rrt.cleanup()

if __name__ == '__main__':

    rospy.init_node('robrex_jar_cabinet')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


