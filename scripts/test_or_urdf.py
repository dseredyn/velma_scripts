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

class TestOrURDF:
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

#        parser = OptionParser(description='Openrave Velma interface')
#        OpenRAVEGlobalArguments.addOptions(parser)
#        (options, leftargs) = parser.parse_args()
#        options._collision = "fcl"
#        env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
#        env.GetCollisionChecker().SetCollisionOptions(4)
#        xacro_uri = "package://velma_description/robots/velma.urdf.xacro"
#        srdf_uri = "package://velma_description/robots/velma.srdf"

        rospack = rospkg.RosPack()
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF()#env_file=rospack.get_path('velma_scripts')+"/data/romoco/romoco.env.xml")#, collision='fcl')
        openrave.readRobot(xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro', srdf_uri=rospack.get_path('velma_description') + '/robots/velma.srdf')
#        openrave.startOpenrave(rospack.get_path('velma_scripts')+"/data/romoco/romoco_robot.env.xml")

#        print "geometry group:", openrave.env.GetCollisionChecker().GetGeometryGroup()
#        openrave.env.GetCollisionChecker().SetGeometryGroup("spheres")
#        print "geometry group:", openrave.env.GetCollisionChecker().GetGeometryGroup()

#        urdf_module = RaveCreateModule(env, 'urdf')
#        name = urdf_module.SendCommand('load ' + urdf_uri + ' ' + srdf_uri)
#        robot_rave = env.GetRobot(name)

#        for man in robot_rave.GetManipulators():
#            print "manipulator", man
#            print "gripper", man.GetEndEffector()

#        for joint in openrave.robot_rave.GetJoints():
#            print joint

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
            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(pt[0],pt[1],0.1), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), T=None)

        lim5_lo, lim5_up = openrave.robot_rave.GetJoint("right_arm_5_joint").GetLimits()
        lim6_lo, lim6_up = openrave.robot_rave.GetJoint("right_arm_6_joint").GetLimits()
        for q5 in np.linspace(lim5_lo[0], lim5_up[0], 20):
            for q6 in np.linspace(lim6_lo[0], lim6_up[0], 20):
                conf = {
                "right_arm_5_joint":q5,
                "right_arm_6_joint":q6,
                }
                openrave.updateRobotConfigurationRos(conf)
                openrave.env.UpdatePublishedBodies()
                report = CollisionReport()
                if openrave.robot_rave.CheckSelfCollision(report):
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(q5,q6,0), m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                else:
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(q5,q6,0), m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                rospy.sleep(0.01)

        raw_input(".")
        exit(0)

        for link in openrave.robot_rave.GetLinks():
            geoms = link.GetGeometries()
            print "geoms:", len(geoms)
            col_geoms = link.GetGroupNumGeometries("collision")
            print "col_geoms:", col_geoms
            vis_geoms = link.GetGroupNumGeometries("visual")
            print "vis_geoms:", vis_geoms
            print link, link.GetCollisionData()
            for g in geoms:
                info = g.GetInfo()
                print "   geom", g.GetType()
                print "      mesh_collision", info._meshcollision
                if len(info._meshcollision.vertices) > 0:
                    x = info._meshcollision.vertices[0][0]
                    y = info._meshcollision.vertices[0][1]
                    z = info._meshcollision.vertices[0][2]
                    print "      mesh_collision", math.sqrt(x*x+y*y+z*z)
                print "      modifable", info._bModifiable
                print "      render", info._filenamerender
                print "      coll", info._filenamecollision

        sphere = RaveCreateKinBody(openrave.env,'')
        sphere.SetName("sphere")
        sphere.InitFromSpheres(numpy.array([[0,0,0,0.1]]),True)
        openrave.env.Add(sphere,True)

        mj = openrave.robot_rave.GetJoint("right_HandFingerOneKnuckleThreeJoint")
        print "mimic", mj.IsMimic(), mj.GetMimicEquation()

        ji = openrave.robot_rave.GetJoint("right_HandFingerOneKnuckleTwoJoint").GetDOFIndex()
        openrave.robot_rave.SetDOFValues([1.5], [ji])

        with openrave.robot_rave.GetEnv():
                # stop rendering the non-gripper links
                for link in openrave.robot_rave.GetLinks():
                    if link not in openrave.robot_rave.GetActiveManipulator().GetChildLinks():
                        link.Enable(False)
#                        print "hiding:", link.GetName()
#                        for geom in link.GetGeometries():
#                            print "   geom", geom
#                            print geom.IsVisible()
#                            geom.SetDraw(False)
#                            print geom.IsVisible()
                openrave.env.UpdatePublishedBodies()
                raw_input(".")

        exit(0)


        x = -0.3
        while True:
            tr = self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,0.8,1.9)))
            sphere.SetTransform(tr)
            openrave.env.UpdatePublishedBodies()
            report = CollisionReport()
            ret = openrave.env.CheckCollision(sphere, report)
            if report.plink1 == None:
                print None
            else:
                print report.plink1.GetParent().GetName(), report.plink2.GetName() 
#                print "   ", report.vLinkColliding
                for link1, link2 in report.vLinkColliding:
                    print "   ", link1.GetParent().GetName(), link2.GetName()
#                print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 
            raw_input(".")
            x += 0.005

        exit(0)

# CollisionOptions:
# CO_Distance = 1, ///< Compute distance measurements, this is usually slow and not all checkers support it.
# CO_UseTolerance = 2, ///< not used
# CO_Contacts = 4, ///< Return the contact points of the collision in the \ref CollisionReport. Note that this takes longer to compute.
# CO_RayAnyHit = 8, ///< When performing collision with rays, if this is set, algorithm just returns any hit instead of the closest (can be faster)
# Allows planners to greatly reduce redundant collision checks.
# If set and the target object is a robot, then only the links controlled by the currently set active DOFs and their attached bodies will be checked for collisions.
# The things that **will not be** checked for collision are:
# - links that do not remove with respect to each other as a result of moving the active dofs.
# CO_ActiveDOFs = 0x10,
# CO_AllLinkCollisions = 0x20, ///< if set then all the link collisions will be returned inside CollisionReport::vLinkColliding. Collision is slower because more pairs have to be checked.
# CO_AllGeometryContacts = 0x40, ///< if set, then will return the contacts of all the colliding geometries. This option can be very slow.

        box = RaveCreateKinBody(env,'')
        box.SetName("box")
        box.InitFromBoxes(numpy.array([[0,0,0,0.1,0.1,0.1]]),True)
        env.Add(box,True)

        sphere = RaveCreateKinBody(env,'')
        sphere.SetName("sphere")
        sphere.InitFromSpheres(numpy.array([[0,0,0,0.1]]),True)
        env.Add(sphere,True)

        transforms = [
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(0,0,0.198))),
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(0,0,-0.198))),
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(0,0.198,0))),
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(0,-0.198,0))),
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(0.198,0,0))),
        self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(-0.198,0,0)))
        ]

        for tr in transforms:
            print "transform", tr
            sphere.SetTransform(tr)
            env.UpdatePublishedBodies()

            report = CollisionReport()
            ret = env.CheckCollision(box, report)
            print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 
            print report.contacts[0]

            ret = env.CheckCollision(sphere, report)
#            print ret
#            print report
            print report.plink1.GetParent().GetName(), report.plink2.GetParent().GetName() 
            print report.contacts[0]

        raw_input(".")

        exit(0)




if __name__ == '__main__':

    rospy.init_node('test_or_urdf')

    task = TestOrURDF()
    rospy.sleep(1)

    task.spin()


