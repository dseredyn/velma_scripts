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
import rrt_star_planner
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
        env_file=rospack.get_path('velma_scripts') + '/data/key/vis_test.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        rrt = rrt_planner.PlannerRRT(3, env_file, xacro_uri, srdf_path)

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        rospy.sleep(0.5)
        self.velma.updateTransformations()

        if simulation:
            hv = [3.2, 3.2, 3.2, 3.2]
        ht = [3000, 3000, 3000, 3000]
        self.velma.moveHandLeft([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)
        self.velma.moveHandRight([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)

        rospy.sleep(1.0)

        #
        # Initialise Openrave
        #

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file)
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))

        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        # TEST: key hole goal
        if False:
            task = KeyRotTaskRRT(openrave)
            task.SampleGoal(None, None)
            exit(0)

        # TEST: head IK
        if False:
            v_rot = 0.800
            v_lean = 0.375
            v_head = 0.392
            h_cam = 0.0
            v_cam = 0.225
            head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)
            openrave.addSphere("target_sphere", 0.05)

            while not rospy.is_shutdown():
                head_kin.UpdateTorsoPose(openrave.robot_rave.GetJoint("torso_0_joint").GetValue(0), openrave.robot_rave.GetJoint("torso_1_joint").GetValue(0))
                target_pos = PyKDL.Vector(1, random.uniform(-1.5, 1.5), random.uniform(0,2))
                head_kin.UpdateTargetPosition(target_pos.x(), target_pos.y(), target_pos.z())
                openrave.updatePose("target_sphere", PyKDL.Frame(target_pos))
                head_kin.TransformTargetToHeadFrame()
                joint_pan, joint_tilt = head_kin.CalculateHeadPose()
                if joint_pan == None:
                    continue

                openrave.robot_rave.SetDOFValues([joint_pan, joint_tilt], [openrave.robot_rave.GetJoint("head_pan_joint").GetDOFIndex(), openrave.robot_rave.GetJoint("head_tilt_joint").GetDOFIndex()])

                raw_input("Press ENTER to continue...")
            exit(0)

        raw_input("Press ENTER to continue...")

#        T_B_E_list = [PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.6, 0, 1.6))]
#        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.GraspTaskRRT, ("right", T_B_E_list), 30.0)

#        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.KeyRotTaskRRT, ("right",), 30.0)

        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.LooAtTaskRRT, ("right",), 60.0)

#        path, dof_names = rrt.RRTstar(openrave.robot_rave.GetDOFValues(), tasks.MoveArmsCloseTaskRRT, ("right",), 30.0)

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


        rospy.sleep(1)
        openrave.runOctomap()

        sphere = RaveCreateKinBody(openrave.env,'')
        sphere.SetName("sphere")
        sphere.InitFromSpheres(numpy.array([[0,0,0,0.05]]),True)
        openrave.env.Add(sphere,True)

        # test the collision checker for octomap
        if True:
            raw_input("Press ENTER to continue...")

            ob = openrave.env.GetKinBody("_OCTOMAP_MAP_")
            cc = openrave.env.GetCollisionChecker()

            m_id = 0
            for x in np.linspace(0,1.5,30):
                for y in np.linspace(-1,1,40):
                    for z in np.linspace(1,2,20):
#                        print x,y,z
                        tr = conv.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,y,z)))
                        sphere.SetTransform(tr)
                        openrave.env.UpdatePublishedBodies()
                        report = CollisionReport()
                        ret = cc.CheckCollision(sphere, report)
#                        ret = openrave.env.CheckCollision(ob, report)
#                        print ret
                        if ret:
                            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)

                        continue
                        if report.plink1 == None:
                            print None
                        else:
                            print report.plink1.GetParent().GetName(), report.plink2.GetName() 
#                            print "   ", report.vLinkColliding
                            for link1, link2 in report.vLinkColliding:
                                print "   ", link1.GetParent().GetName(), link2.GetName()
#                            print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 

        exit(0)

        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)

        raw_input("Press ENTER to look around...")

#        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-0.5,1))))
#        raw_input("Press ENTER to exit...")

#        exit(0)
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-1,1.2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(0.2,-1,2))))
        raw_input("Press ENTER to look around...")
        self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,2))))



        raw_input(".")

        exit(0)

if __name__ == '__main__':

    rospy.init_node('test_or_octomap')

    task = TestOrOctomap()
    rospy.sleep(1)

    task.spin()


