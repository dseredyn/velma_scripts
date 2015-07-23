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
import objectstate

class TestOrOctomap:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):

        simulation = True

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/common/velma_room.env.xml'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        obj_filenames = [
        rospack.get_path('velma_scripts') + '/data/jar/jar.kinbody.xml'
        ]

        rrt = rrt_star_connect_planner.PlannerRRT(3, env_file, obj_filenames, srdf_path)

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
        self.velma.moveHandLeft([45.0/180.0*math.pi, 45.0/180.0*math.pi, 45.0/180.0*math.pi, 0], hv, ht, 5000, True)
        self.velma.moveHandRight([45.0/180.0*math.pi, 45.0/180.0*math.pi, 45.0/180.0*math.pi, 0], hv, ht, 5000, True)

        rospy.sleep(1.0)
        #
        # Initialise Openrave
        #

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenrave(collision='fcl')
        openrave.loadEnv(env_file)
        openrave.runOctomapClient()
        openrave.readRobot(srdf_path=srdf_path)

        for filename in obj_filenames:
            body = openrave.env.ReadKinBodyXMLFile(filename)
            body.Enable(False)
            body.SetVisible(False)
            openrave.env.Add(body)


        mo_state = objectstate.MovableObjectsState(openrave.env, obj_filenames)

# TODO
#        for link in openrave.robot_rave.GetLinks():
#            print link.GetName(), len(link.GetCollisionData().vertices), len(link.GetGeometries())
#            for geom in link.GetGeometries():
#                print "   ", geom.GetType(), geom.IsVisible(), geom.GetSphereRadius(), len(geom.GetCollisionMesh().vertices), geom.GetBoxExtents()

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
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.17))
                T_B_E_list.append(T_B_Ed)
                T_B_Ed = T_B_J * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.17))
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
        for i in range(10):
            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            mo_state.update(self.listener, time_tf)
            mo_state.updateOpenrave(openrave.env)
            rospy.sleep(0.1)

        print "octomap updated"

        target_gripper = "left"
        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), mo_state.obj_map, tasks.GraspTaskRRT, (target_gripper, T_B_E_list), 60.0)

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

        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        print "before grasp"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        if target_gripper == "left":
            self.velma.moveHandLeft([70.0/180.0*math.pi, 70.0/180.0*math.pi, 70.0/180.0*math.pi, 0], hv, ht, 5000, True)
        else:
            self.velma.moveHandRight([70.0/180.0*math.pi, 70.0/180.0*math.pi, 70.0/180.0*math.pi, 0], hv, ht, 5000, True)

        raw_input("Press ENTER to continue...")
        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        print "after grasp"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        openrave.robot_rave.SetActiveManipulator(target_gripper+"_arm")
        openrave.robot_rave.Grab(openrave.env.GetKinBody("jar"))

        print "after grab"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        path.reverse()
        traj = self.velma.prepareTrajectory(path, self.velma.getJointStatesByNames(dof_names))
        self.velma.moveJointTraj(traj, dof_names, start_time=0.2)

        raw_input("Press ENTER to continue...")
        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        raw_input("Press ENTER to exit...")

        rrt.cleanup()

if __name__ == '__main__':

    rospy.init_node('robrex_jar_cabinet')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


