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
from velma import Velma
import openraveinstance
import rosparam

from ompl import base as ob
from ompl import geometric as og
#from ompl import morse as om

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

    def OpenraveToKDL(self, T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def planVis(self, openrave):
      with openrave.env:
        debug = True
        m_id = 0

        if debug:
            self.pub_marker.eraseMarkers(0,3000, frame_id='world')
            rospy.sleep(0.01)

        kinect_fov = 30.0/180.0*math.pi

        # target: key pocket
        vis_targets = [
        ("vis_target_0", 0.1, PyKDL.Vector(0, -0.4, 1.0)),
        ("vis_target_1", 0.1, PyKDL.Vector(0.1, -0.4, 1.0)),
        ("vis_target_2", 0.1, PyKDL.Vector(0.1, -0.5, 1.0)),
        ("vis_target_3", 0.1, PyKDL.Vector(0, -0.5, 1.0)),
        ("vis_target_4", 0.1, PyKDL.Vector(0.05, -0.45, 1.0)),
        ]

        vis_bodies = []
        # target: test (vertical axis at the door plane)
#        vis_targets = [
#        ("vis_target_0", 0.1, PyKDL.Vector(1, 0.0, 1.2)),
#        ("vis_target_1", 0.1, PyKDL.Vector(1, 0.0, 1.3)),
#        ("vis_target_2", 0.1, PyKDL.Vector(1, 0.0, 1.4)),
#        ("vis_target_3", 0.1, PyKDL.Vector(1, 0.0, 1.5)),
#        ("vis_target_4", 0.1, PyKDL.Vector(1, 0.0, 1.6)),
#        ]

        for (name, diam, pos) in vis_targets:
            if debug:
                m_id = self.pub_marker.publishSinglePointMarker(pos, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(diam, diam, diam), T=None)
                rospy.sleep(0.01)
            body = openrave.addSphere(name, diam)
            body.SetTransform(self.KDLToOpenrave(PyKDL.Frame(pos)))
            vis_bodies.append( body )
            openrave.env.Remove( body )

        def getSightAngle(openrave, q=None, dof_indices=None):            
            if q != None and dof_indices != None:
                current_q = openrave.robot_rave.GetDOFValues(dof_indices)
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                openrave.env.UpdatePublishedBodies()

            T_W_C = self.OpenraveToKDL(openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()

            angle_sum = 0.0
            for (name, size, pos_W) in vis_targets:
                pos_C = T_C_W * pos_W
                angle_sum += velmautils.getAngle(pos_C, PyKDL.Vector(0,0,1))

            if q != None and dof_indices != None:
                openrave.robot_rave.SetDOFValues(current_q, dof_indices)
                openrave.env.UpdatePublishedBodies()

            return angle_sum

        def getVisibility(openrave, vis_bodies, q=None, dof_indices=None):
            rays_hit = 0
            m_id = 0

            if q != None and dof_indices != None:
                current_q = openrave.robot_rave.GetDOFValues(dof_indices)
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                openrave.env.UpdatePublishedBodies()

            for body in vis_bodies:
                openrave.env.Add( body )
            T_W_C = self.OpenraveToKDL(openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()
            cam_W = T_W_C * PyKDL.Vector()
            cam_dir_W = PyKDL.Frame(T_W_C.M) * PyKDL.Vector(0,0,0.5)
            if debug:
                m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+cam_dir_W, m_id, 1, 1, 1, frame='world', namespace='kinect_head_rays', scale=0.01)

            # create rays connecting the optical frame and the target objects
            for (name, diam, pos_W) in vis_targets:
                pos_C = T_C_W * pos_W
                dir_W = pos_W - cam_W
                if pos_C.z() < 0.1:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 0, 1, frame='world', namespace='kinect_head_rays', scale=0.01)
                    continue
                if velmautils.getAngle(PyKDL.Vector(0,0,1), pos_C) > kinect_fov:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 0, 1, frame='world', namespace='kinect_head_rays', scale=0.01)
                    continue

                report = CollisionReport()
                ret = openrave.env.CheckCollision(Ray((cam_W[0], cam_W[1], cam_W[2]), (dir_W[0], dir_W[1], dir_W[2])), report)
                if ret and report.plink1 != None and report.plink1.GetParent().GetName().find("vis_target_") == 0:
                    rays_hit += 1
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 0, 1, 0, frame='world', namespace='kinect_head_rays', scale=0.01)
                else:
                    if debug:
                        m_id = self.pub_marker.publishVectorMarker(cam_W, cam_W+dir_W, m_id, 1, 0, 0, frame='world', namespace='kinect_head_rays', scale=0.01)

            for body in vis_bodies:
                openrave.env.Remove( body )

            if q != None and dof_indices != None:
                openrave.robot_rave.SetDOFValues(current_q, dof_indices)
                openrave.env.UpdatePublishedBodies()

            return rays_hit

        # test visibility
#        if True:
#            print getVisibility(openrave)

        dof_names_torso = [
        "head_pan_joint",
        "head_tilt_joint",
        "torso_0_joint",
        ]

        dof_names_left_arm = [
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "left_arm_4_joint",
        "left_arm_5_joint",
        "left_arm_6_joint",
        ]

        dof_indices_torso = []
        dof_limits_torso = []
        for joint_name in dof_names_torso:
            joint = openrave.robot_rave.GetJoint(joint_name)
            dof_indices_torso.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            dof_limits_torso.append( (lim_lo[0], lim_up[0]) )

        dof_indices_left_arm = []
        dof_limits_left_arm = []
        for joint_name in dof_names_left_arm:
            joint = openrave.robot_rave.GetJoint(joint_name)
            dof_indices_left_arm.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            dof_limits_left_arm.append( (lim_lo[0], lim_up[0]) )

        # for planning torso movement disable arms links
        disabled_links_torso = [
        "calib_left_arm_base_link",
        "calib_right_arm_base_link",
        "left_HandFingerOneKnuckleOneLink",
        "left_HandFingerOneKnuckleThreeLink",
        "left_HandFingerOneKnuckleTwoLink",
        "left_HandFingerThreeKnuckleThreeLink",
        "left_HandFingerThreeKnuckleTwoLink",
        "left_HandFingerTwoKnuckleOneLink",
        "left_HandFingerTwoKnuckleThreeLink",
        "left_HandFingerTwoKnuckleTwoLink",
        "left_HandPalmLink",
        "left_arm_2_link",
        "left_arm_3_link",
        "left_arm_4_link",
        "left_arm_5_link",
        "left_arm_6_link",
        "left_arm_7_link",
        "left_gripper_calib_link",
        "left_gripper_calib_link1",
        "left_gripper_calib_link2",
        "left_gripper_mount_link",
        "left_hand_camera_link",
        "left_hand_camera_optical_frame",
        "right_HandFingerOneKnuckleOneLink",
        "right_HandFingerOneKnuckleThreeLink",
        "right_HandFingerOneKnuckleTwoLink",
        "right_HandFingerThreeKnuckleThreeLink",
        "right_HandFingerThreeKnuckleTwoLink",
        "right_HandFingerTwoKnuckleOneLink",
        "right_HandFingerTwoKnuckleThreeLink",
        "right_HandFingerTwoKnuckleTwoLink",
        "right_HandPalmLink",
        "right_arm_2_link",
        "right_arm_3_link",
        "right_arm_4_link",
        "right_arm_5_link",
        "right_arm_6_link",
        "right_arm_7_link",
        "right_gripper_calib_link",
        "right_gripper_calib_link1",
        "right_gripper_calib_link2",
        "right_gripper_mount_link",
        "torso_base",
        "torso_link0",
        ]

        # for planning left arm movement disable right arm links
        disabled_links_left_arm = [
        "calib_right_arm_base_link",
        "right_HandFingerOneKnuckleOneLink",
        "right_HandFingerOneKnuckleThreeLink",
        "right_HandFingerOneKnuckleTwoLink",
        "right_HandFingerThreeKnuckleThreeLink",
        "right_HandFingerThreeKnuckleTwoLink",
        "right_HandFingerTwoKnuckleOneLink",
        "right_HandFingerTwoKnuckleThreeLink",
        "right_HandFingerTwoKnuckleTwoLink",
        "right_HandPalmLink",
        "right_arm_2_link",
        "right_arm_3_link",
        "right_arm_4_link",
        "right_arm_5_link",
        "right_arm_6_link",
        "right_arm_7_link",
        "right_gripper_calib_link",
        "right_gripper_calib_link1",
        "right_gripper_calib_link2",
        "right_gripper_mount_link",
        ]

        # planning for torso
#        for link in openrave.robot_rave.GetLinks():
#            if link.GetName() in disabled_links_torso:
#                link.Enable(False)
#            else:
#                link.Enable(True)

        # TODO


#        return

        dof_names = [
        "head_pan_joint",
        "head_tilt_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
#        "left_arm_2_joint",
#        "left_arm_3_joint",
#        "left_arm_4_joint",
#        "left_arm_5_joint",
#        "left_arm_6_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
#        "right_arm_2_joint",
#        "right_arm_3_joint",
#        "right_arm_4_joint",
#        "right_arm_5_joint",
#        "right_arm_6_joint",
        "torso_0_joint",
#        "torso_1_joint",
        ]

        dof_indices = []
        dof_limits = []
        for joint_name in dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            dof_limits.append( (lim_lo[0], lim_up[0]) )

        print "planning for %s joints"%(len(dof_indices))

        self.collision_checks = 0

        def isStateValid(state):
            self.collision_checks += 1
#            return True
            q = []
            for i in range(len(dof_indices)):
                q.append(state[i])
            is_valid = True
            current_q = openrave.robot_rave.GetDOFValues(dof_indices)
            openrave.robot_rave.SetDOFValues(q, dof_indices)
            openrave.env.UpdatePublishedBodies()
            report1 = CollisionReport()
            report2 = CollisionReport()
 	    if openrave.robot_rave.CheckSelfCollision(report1) or openrave.env.CheckCollision(openrave.robot_rave, report2):
                is_valid = False
            openrave.robot_rave.SetDOFValues(current_q, dof_indices)
            openrave.env.UpdatePublishedBodies()
            return is_valid

        space = ob.RealVectorStateSpace()
        for i in range(len(dof_names)):
            space.addDimension(dof_names[i], dof_limits[i][0], dof_limits[i][1])

        # without SimpleSetup
        si = ob.SpaceInformation(space);
        si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

        print "si.getStateValidityCheckingResolution", si.getStateValidityCheckingResolution()
        si.setStateValidityCheckingResolution(0.03)

#        class InformedValidStateSampler(ob.ValidStateSampler):
#            def __init__(self, si):
#                ob.ValidStateSampler.__init__(self, si)
#                self.name_ = "my sampler"
#                print "InformedValidStateSampler: init"
#            def sample(state):
#                print "sample"
#                return True
#            def sampleNear(state_out, state_in, distance):
#                print "sampleNear"
#                return True

#        def allocValidStateSampler(si):
#            return InformedValidStateSampler(si)

        si.setup()
#        si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocValidStateSampler))
#        si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocOBValidStateSampler))

        # create a simple setup object
#        ss = og.SimpleSetup(space)
#        ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

        start = ob.State(space)
#        goal = ob.State(space)
        current_q = openrave.robot_rave.GetDOFValues(dof_indices)
        for i in range(len(current_q)):
            start[i] = current_q[i]
#            goal[i] = start[i] + 0.05

        class VisGoal(ob.GoalRegion):
            def __init__(self, si):
                ob.GoalRegion.__init__(self, si)
            def distanceGoal(self, state):
                q = []
                for i in range(len(dof_indices)):
                    q.append(state[i])
                rays_hit = getVisibility(openrave, vis_bodies, q=q, dof_indices=dof_indices)
                return len(vis_targets) - rays_hit

        class VisGoal2(ob.Goal):
            def __init__(self, si):
                ob.Goal.__init__(self, si)
            def isSatisfied(self, state):
                return True
#            def isSatisfied(state, distance):
#                print "bbb"
#                return True

        class VisOptimizationObjective(ob.OptimizationObjective):
            def __init__(self, si):
                ob.OptimizationObjective.__init__(self, si)
                self.si = si
            def stateCost(self, state):
#                q = []
#                for i in range(len(dof_indices)):
#                    q.append(state[i])
#                cost = getSightAngle(openrave, q=q, dof_indices=dof_indices)
#                return ob.Cost(cost)

                return ob.Cost(0.0)

                cost = 0.0
                for i in range(len(dof_indices)):
                    diff = (state[i]-start[i])
                    cost += diff*diff
                return ob.Cost(math.sqrt(cost))

            def motionCost(self, s1, s2):
#                q1 = []
#                q2 = []
#                for i in range(len(dof_indices)):
#                    q1.append(s1[i])
#                    q2.append(s2[i])
#                q_diff = np.array(q2) - np.array(q1)
#                cost = math.sqrt(np.dot(q_diff, q_diff)) + self.stateCost(s2) - self.stateCost(s1)
#                cost = math.sqrt(np.dot(q_diff, q_diff)) + self.stateCost(s2).value() - self.stateCost(s1).value()
#                cost = np.dot(q_diff, q_diff)

                cost1 = 0.0
                for i in range(len(dof_indices)):
                    diff = (s1[i]-start[i])
                    cost1 += diff*diff

                cost2 = 0.0
                for i in range(len(dof_indices)):
                    diff = (s2[i]-start[i])
                    cost2 += diff*diff

                cost = self.si.distance(s1,s2) * (math.sqrt(cost1) + math.sqrt(cost2))*0.5 # + (self.stateCost(s2).value() - self.stateCost(s1).value())
#                cost = 0.0
#                for i in range(len(dof_indices)):
#                    cost += abs(s1[i] - s2[i])
                return ob.Cost(cost)

        # without SimpleSetup
        pdef = ob.ProblemDefinition(si)
        pdef.clearStartStates()
        pdef.addStartState(start)
        goal = VisGoal(si)
        goal.setThreshold(1.0/len(vis_targets))
        pdef.setGoal(goal)
        opt_obj = VisOptimizationObjective(si)
        pdef.setOptimizationObjective(opt_obj)
#        pdef.setStartAndGoalStates(start, goal)
        planner = og.RRTstar(si)
        print planner
        planner.setProblemDefinition(pdef)
        planner.setDelayCC(True)
#        planner.setPrune(True)
#        print "pruning:", planner.getPruneStatesImprovementThreshold()
#        planner.setPruneStatesImprovementThreshold(0.99)
        planner.setRange(360.0/180.0*math.pi)
        print planner
        planner.setup()
        print planner

#        ss.setStartAndGoalStates(start, goal)

        # this will automatically choose a default planner with
        # default parameters
        solved = planner.solve(30.0 * 1.0)

        print "collision_checks", self.collision_checks

        if solved:
            # try to shorten the path
#            ss.simplifySolution()
            print "getSolutionCount: ", pdef.getSolutionCount()
            # print the simplified path
            sol_path = pdef.getSolutionPath()
            print sol_path
            print "cost", sol_path.cost(opt_obj).value()
            sol_path.interpolate(100)

            traj = []
            for state in sol_path.getStates():
                traj.append([])
                for i in range(len(dof_indices)):
                    traj[-1].append(state[i])

            final_state = sol_path.getStates()[-1]
            q = []
            for i in range(len(dof_indices)):
                q.append(final_state[i])
            rays_hit = getVisibility(openrave, vis_bodies, q=q, dof_indices=dof_indices)
            print "visibility:", rays_hit

            while True:
                raw_input(".")
                if rospy.is_shutdown():
                    break
                openrave.showTrajectory(dof_names, 5.0, traj)


    def spin(self):

        #
        # Initialise Openrave
        #

#        a1 = np.array([1,2,3])
#        a2 = np.array([3,2,1])
#        print a1*3
#        exit(0)

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/key/vis_test.env.xml'
        xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=env_file)
#        openrave.readRobot(xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro', srdf_uri=rospack.get_path('velma_description') + '/robots/velma.srdf')
        openrave.readRobot(xacro_uri=xacro_uri, srdf_path=srdf_path)


#        for link in openrave.robot_rave.GetLinks():
#            print link.GetName()
#        exit(0)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        rospy.sleep(0.5)
        self.velma.updateTransformations()

#        T_W_T = self.velma.T_W_E * PyKDL.Frame(PyKDL.Vector(0,0,0.17))
#        print T_W_T.M.GetQuaternion()
#        print T_W_T.p
#        exit(0)

        openrave.updateRobotConfigurationRos(self.velma.js_pos)

        self.planVis(openrave)

        raw_input("Press ENTER to exit...")
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
                        tr = self.KDLToOpenrave(PyKDL.Frame(PyKDL.Vector(x,y,z)))
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


