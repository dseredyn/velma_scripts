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
import tf

import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_msgs.msg import *
from barrett_hand_controller_msgs.srv import *
from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
from threading import Lock

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
from velma import Velma
import random
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import openraveinstance
import itertools
import operator
import rospkg

def KDLToOpenrave(T):
        ret = numpy.array([
        [T.M[0,0], T.M[0,1], T.M[0,2], T.p.x()],
        [T.M[1,0], T.M[1,1], T.M[1,2], T.p.y()],
        [T.M[2,0], T.M[2,1], T.M[2,2], T.p.z()],
        [0, 0, 0, 1]])
        return ret

def OpenraveToKDL(T):
        rot = PyKDL.Rotation(T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2])
        pos = PyKDL.Vector(T[0][3], T[1][3], T[2][3])
        return PyKDL.Frame(rot, pos)

class GraspingTask:
    """
Class for grasp learning.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = pub_marker
        self.listener = tf.TransformListener();

    def getMarkerPose(self, marker_id, wait = True, timeBack = None):
        try:
            marker_name = 'ar_marker_'+str(int(marker_id))
            if wait:
                self.listener.waitForTransform('torso_base', marker_name, rospy.Time.now(), rospy.Duration(4.0))
            if timeBack != None:
                time = rospy.Time.now() - rospy.Duration(timeBack)
            else:
                time = rospy.Time(0)
            jar_marker = self.listener.lookupTransform('torso_base', marker_name, time)
        except:
            return None
        return pm.fromTf(jar_marker)

    def getCameraPoseFake(self):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.6041471326857807, 0.7290707942893216, -0.24257326295862056, 0.21123501385869978), PyKDL.Vector(0.274115,-0.000762625,     1.67876) )  # romoco_02
#        return PyKDL.Frame(PyKDL.Rotation.RotY(135.0/180.0*math.pi), PyKDL.Vector(0.4, 0, 1.4)) * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi))

    def getMarkerPoseFake(self, marker_id, wait = True, timeBack = None):

        if True:  # romoco_02
            ar_track = [
            [5,0,0,0,0,-0.168262043609,0.0930374127131,1.01813819229,0.950324481028,-0.0289575235949,0.00394574675416,0.309885904275],
            [21,0,0,0,0,0.082505708292,0.110761122122,0.603730984018,-0.458504669821,-0.709444231601,-0.0397255592372,0.533745472997],
            [30,0,0,0,0,0.104154326153,0.0757350473769,0.587008320764,0.861332293804,0.391007810313,-0.113641433205,0.303817702879],
            [18,0,0,0,0,0.124413927856,0.111452725921,0.599775167445,0.696719708925,0.480876853947,0.5293727524,-0.0557098514612],
            [19,0,0,0,0,0.123152725863,0.198755505957,0.716141316519,0.700134532901,0.471859046812,0.532229610902,-0.0623884369125],
            [22,0,0,0,0,0.0797242701158,0.196635593195,0.713926653472,-0.448409835388,-0.710363705627,-0.044302175627,0.540693390462],
            ]
            for mk in ar_track:
                if mk[0] == marker_id:
                    
                    return self.getCameraPoseFake() * PyKDL.Frame(PyKDL.Rotation.Quaternion(mk[8], mk[9], mk[10], mk[11]), PyKDL.Vector(mk[5], mk[6], mk[7]) )

            return None

#        T_B_Tm = PyKDL.Frame( PyKDL.Rotation.EulerZYZ(0.1, -0.1, 0.0), PyKDL.Vector(0.55,-0.4,0.9) )
        T_B_Tm = PyKDL.Frame( PyKDL.Rotation.RotZ(90.0/180.0*math.pi), PyKDL.Vector(0.55,-0.2,0.9) )
        T_B_Tbb = PyKDL.Frame( PyKDL.Vector(0.5,-0.8,2.0) )
        T_Tm_Bm = PyKDL.Frame( PyKDL.Vector(-0.06, 0.3, 0.135) )
        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotZ(90.0/180.*math.pi) * PyKDL.Rotation.RotY(90.0/180.*math.pi), PyKDL.Vector(0.0,0.0,0.17) )    # the block standing on the table
#        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotZ(-30.0/180.*math.pi), PyKDL.Vector(0.1,-0.1,0.06) )    # the block lying on the table
#        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotX(90.0/180.*math.pi), PyKDL.Vector(0.1,-0.1,0.06) )
        if marker_id == 6:
            return T_B_Tm
        elif marker_id == 7:
            return T_B_Tm * T_Tm_Bm
        elif marker_id == 8:
            return T_B_Tbb
        elif marker_id == 19:
#            return T_B_Tm * T_Tm_Bm * T_Bm_Gm
            return T_B_Tm * T_Bm_Gm
        elif marker_id == 35:
            return T_B_Tm * T_Tm_Bm * T_Bm_Gm
        return None

    def getCameraPose(self):
        return pm.fromTf(self.listener.lookupTransform('torso_base', 'camera', rospy.Time(0)))

    def allowUpdateObjects(self):
        self.allow_update_objects_pose = True

    def disallowUpdateObjects(self):
        self.allow_update_objects_pose = False

    def waitForOpenraveInit(self):
        while not rospy.is_shutdown():
            if self.openrave.rolling:
                break
            rospy.sleep(0.5)

    def switchToJoint(self, robot):
        if robot.isCartesianImpedanceActive():
            raw_input("Press Enter to enable joint impedance...")
            if robot.checkStopCondition():
                exit(0)
            robot.switchToJoint()
        elif robot.isJointImpedanceActive():
            pass
        else:
            print "FATAL ERROR: impedance control in unknown state: %s %s"%(robot.joint_impedance_active, robot.cartesian_impedance_active)
            exit(0)

    def switchToCartesian(self, robot):
        if robot.isJointImpedanceActive():
            raw_input("Press Enter to enable cartesian impedance...")
            if robot.checkStopCondition():
                exit(0)
            robot.switchToCart()
        elif robot.isCartesianImpedanceActive():
            pass
        else:
            print "FATAL ERROR: impedance control in unknown state: %s %s"%(robot.joint_impedance_active, robot.cartesian_impedance_active)
            exit(0)

    def calculateWrenchesForTransportTask(self, ext_wrenches_W, transport_T_B_O):
        ext_wrenches_O = []
        for i in range(len(transport_T_B_O)-1):
            diff_B_O = PyKDL.diff(transport_T_B_O[i], transport_T_B_O[i+1])
            # simulate object motion and calculate expected wrenches
            for t in np.linspace(0.0, 1.0, 5):
                T_B_Osim = PyKDL.addDelta(transport_T_B_O[i], diff_B_O, t)
                T_Osim_B = T_B_Osim.Inverse()
                for ewr in ext_wrenches_W:
                    ext_wrenches_O.append(PyKDL.Frame(T_Osim_B.M) * ewr)
        return ext_wrenches_O

    def showPlan(self, plan):
        init_config = self.openrave.getRobotConfigurationRos()
        init_T_B_O = None
        grasped = False
        for stage in plan:
            if stage[0] == "move_joint":
                traj = stage[1]
                raw_input("Press Enter to visualize the joint trajectory...")
                duration = math.fsum(traj[3])
                self.openrave.showTrajectory(duration * 5.0, qar_list=traj[4])
                self.openrave.updateRobotConfiguration(qar=traj[0][-1])
            elif stage[0] == "grasp":
                graspable_object_name = stage[1]
#                grasp = stage[2]
                if init_T_B_O == None:
                    init_T_B_O = self.openrave.getPose(graspable_object_name)
                grasped = True
                self.openrave.grab(graspable_object_name)
            elif stage[0] == "move_cart":
                T_B_Wd_traj = stage[1]
                for idx in range(len(T_B_Wd_traj[0])):
                    init_js = self.openrave.getRobotConfigurationRos()
                    traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_Wd_traj[0][idx])
                    raw_input("Press Enter to visualize the cartesian trajectory...")
                    self.openrave.showTrajectory(T_B_Wd_traj[1][idx], qar_list=traj)
                    self.openrave.updateRobotConfiguration(qar=traj[-1])
            elif stage[0] == "move_gripper":
                g_shape = stage[1]
                self.openrave.updateRobotConfigurationRos(g_shape)

        if grasped:
            self.openrave.release()
        if init_T_B_O != None:
            self.openrave.updatePose(graspable_object_name, init_T_B_O)
        self.openrave.updateRobotConfigurationRos(init_config)

    def executePlan(self, plan, time_mult):
        for stage in plan:
            print stage[0]
            if stage[0] == "move_joint":
                traj = stage[1]
                print "switching to joint impedance..."
                self.velma.switchToJoint()
                print "done."
                duration = math.fsum(traj[3])
                print "trajectory len: %s"%(len(traj[0]))
                raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                if self.velma.checkStopCondition():
                    exit(0)
                self.velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if self.velma.checkStopCondition(duration * time_mult + 1.0):
                    exit(0)
            elif stage[0] == "grasp":
                pass
#                graspable_object_name = stage[1]
#                grasp = stage[2]
#                if init_T_B_O == None:
#                    init_T_B_O = self.openrave.getPose(graspable_object_name)
#                grasped = True
#                self.openrave.grab(graspable_object_name)
            elif stage[0] == "move_cart":
                self.velma.switchToCart()
                # move to the desired position
                self.velma.updateTransformations()
                traj = stage[1]
                print traj
#                T_B_Wd = stage[1]
#                duration = self.velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
                raw_input("Press Enter to move the robot in " + str(traj[1][-1]) + " s...")
                if self.velma.checkStopCondition():
                    exit(0)
                self.velma.moveWristTraj(traj[0], traj[1], Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if self.velma.checkStopCondition(traj[1][-1]):
                    break
            elif stage[0] == "move_gripper":
                g_shape = stage[1]
                q = [
                g_shape["right_HandFingerOneKnuckleTwoJoint"],
                g_shape["right_HandFingerTwoKnuckleTwoJoint"],
                g_shape["right_HandFingerThreeKnuckleTwoJoint"],
                g_shape["right_HandFingerOneKnuckleOneJoint"],
                ]
                raw_input("Press Enter to change the gripper configuration...")
                self.velma.move_hand_client(q, t=(3000.0, 3000.0, 3000.0, 3000.0))

    def makePlan(self, graspable_object_name, grasp, transport_T_B_O, penalty_threshold):

        if self.openrave.checkRobotCollision():
            print "makePlan failed: robot is in collision with environment"
            return None, None

        plan_ret = []

        config = self.openrave.getRobotConfigurationRos()
        init_T_B_O = self.openrave.getPose(graspable_object_name)

        T_B_Ed = self.openrave.getGraspTransform(graspable_object_name, grasp, collisionfree=True)

        # set the gripper preshape
        hand_config, contacts_ret_O, normals_O = self.openrave.getFinalConfig(graspable_object_name, grasp, show=False)
        pre_angle = 30.0/180.0*math.pi
        preshape = {
        "right_HandFingerOneKnuckleTwoJoint" : hand_config[0] - pre_angle,
        "right_HandFingerTwoKnuckleTwoJoint" : hand_config[1] - pre_angle,
        "right_HandFingerThreeKnuckleTwoJoint" : hand_config[2] - pre_angle,
        "right_HandFingerOneKnuckleOneJoint" : hand_config[3]}

        self.openrave.updateRobotConfigurationRos(preshape)

        if self.openrave.checkRobotCollision():
            print "makePlan failed: robot is in collision with environment after gripper preshape execution"
            return None, None

        plan_ret.append(["move_gripper", preshape])

        # plan first trajectory (in configuration space)
        self.openrave.extendAllObjects(0.02)
        traj = self.openrave.planMoveForRightArm(T_B_Ed, None, penalty_threshold=penalty_threshold)
        self.openrave.restoreExtendedObjects()
        if traj == None:
            print "colud not plan trajectory in configuration space"
            return None, None


        plan_ret.append(["move_joint", traj])

        penalty = traj[5]

        # start planning from the end of the previous trajectory
        self.openrave.updateRobotConfiguration(qar=traj[0][-1])

        # grab the body
        self.openrave.grab(graspable_object_name)

        g_angle = 10.0/180.0*math.pi
        g_shape = {
        "right_HandFingerOneKnuckleTwoJoint" : hand_config[0] + g_angle,
        "right_HandFingerTwoKnuckleTwoJoint" : hand_config[1] + g_angle,
        "right_HandFingerThreeKnuckleTwoJoint" : hand_config[2] + g_angle,
        "right_HandFingerOneKnuckleOneJoint" : hand_config[3]}

        plan_ret.append(["move_gripper", g_shape])
        plan_ret.append(["grasp", graspable_object_name])

        # calculate the destination pose of the end effector
        T_B_O = self.openrave.getPose(graspable_object_name)
        T_E_O = T_B_Ed.Inverse() * T_B_O


        cart_traj = []
        cart_times = []
        T_B_Wd1 = T_B_Ed * self.velma.T_E_W
        print "makePlan:"
        print T_B_O
#        print T_B_Wd1
        time = 0.0
        for idx in range(1, len(transport_T_B_O)):
            T_B_Ed = transport_T_B_O[idx] * T_E_O.Inverse()
            T_B_Wd2 = T_B_Ed * self.velma.T_E_W
#            print T_B_Wd2
            print transport_T_B_O[idx]
            cart_traj.append(T_B_Wd2)
            time += self.velma.getMovementTime2(T_B_Wd1, T_B_Wd2, max_v_l=0.1, max_v_r=0.2)
            cart_times.append(time)
            T_B_Wd1 = T_B_Wd2

        # calculate the destination pose of the end effector
#        T_B_O = self.openrave.getPose(graspable_object_name)
#        T_E_O = T_B_Ed.Inverse() * T_B_O
#        T_B_Ed = T_B_Od * T_E_O.Inverse()
#        T_B_Wd = T_B_Ed * self.velma.T_E_W

        # interpolate trajectory for the second motion (in the cartesian space)
        for idx in range(len(cart_traj)):
            init_js = self.openrave.getRobotConfigurationRos()
            traj = self.velma_solvers.getCartImpWristTraj(init_js, cart_traj[idx])
            if traj == None:
                print "could not plan trajectory in cartesian space: ", str(idx)
                self.openrave.release()
                self.openrave.updatePose(graspable_object_name, init_T_B_O)
                self.openrave.updateRobotConfigurationRos(config)
                return None, None
            # start planning from the end of the previous trajectory
            self.openrave.updateRobotConfiguration(qar=traj[-1])

        plan_ret.append(["move_cart", [cart_traj, cart_times]])

        self.openrave.release()
        self.openrave.updatePose(graspable_object_name, init_T_B_O)

        self.openrave.updateRobotConfigurationRos(config)

        return penalty, plan_ret

    def poseUpdaterThread(self, args, *args2):
        while not rospy.is_shutdown():
            self.pub_marker.publishConstantMeshMarker("package://velma_scripts/data/meshes/klucz_gerda_binary.stl", 0, r=1, g=0, b=0, scale=1.0, frame_id='right_HandPalmLink', namespace='key', T=self.T_E_O)
            self.pub_marker.publishSinglePointMarker(self.key_endpoint_O, 1, r=1, g=1, b=1, a=0.5, namespace='key', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=self.T_E_O)
            self.pub_marker.publishSinglePointMarker(self.T_O_H*PyKDL.Vector(), 2, r=1, g=1, b=0, a=0.5, namespace='key', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.015, 0.015, 0.015), T=self.T_E_O)
            self.pub_marker.publishVectorMarker(self.T_E_O*self.key_endpoint_O, self.T_E_O*(self.key_endpoint_O+self.key_up_O*0.05), 3, 1, 0, 0, frame='right_HandPalmLink', namespace='key', scale=0.001)
            self.pub_marker.publishVectorMarker(self.T_E_O*self.key_endpoint_O, self.T_E_O*(self.key_endpoint_O+self.key_axis_O*0.05), 4, 0, 1, 0, frame='right_HandPalmLink', namespace='key', scale=0.001)

            m_id = 0
            for pt in self.points:
#                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, a=1, namespace='hand', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, a=1, namespace='hand', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

            rospy.sleep(0.1)

    def pointsCollectorThread(self, point_list, frame_name, gripper_name):
        m_id = 0
        while not rospy.is_shutdown() and self.collect_points:
            points = self.velma.getContactPointsInFrame(300, frame_name, gripper_name)
            for pt in points:
#                point_list.append(pt)
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=1, b=0, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.CUBE, scale=Vector3(pt[1]*2, pt[2]*2, 0.001), T=pt[0])
            rospy.sleep(0.1)

    def spin(self):
        simulation_only = False

        print "creating interface for Velma..."
        # create the interface for Velma robot
        self.velma = Velma()
        print "done."

        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        # key and grasp parameters
        self.T_O_H = PyKDL.Frame(PyKDL.Vector(-0.0215,0,0))
        self.T_H_O = self.T_O_H.Inverse()
        self.T_E_H = PyKDL.Frame(PyKDL.Vector(0,-0.017,0.115))
        self.T_E_O = self.T_E_H * PyKDL.Frame(PyKDL.Rotation.RotZ(-130.0/180.0*math.pi) * PyKDL.Rotation.RotY(-20.0/180.0*math.pi) * PyKDL.Rotation.RotX(-30.0/180.0*math.pi)) * self.T_H_O
        self.key_axis_O = PyKDL.Vector(1,0,0)
        self.key_up_O = PyKDL.Vector(0,1,0)
        self.key_endpoint_O = PyKDL.Vector(0.039,0,0)

        # test
        self.points = []

        # start thread for updating key position in rviz
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        hv = [1.2, 1.2, 1.2, 1.2]
        ht = [3000, 3000, 3000, 3000]
        # set gripper configuration
        if False:
            self.velma.moveHand([0.0, 0.0, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([0.0, 2.3174461354903535, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 1.7038538203360971, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)

        # test
#        print "right hand"
#        for i in range(0, 100):
#            self.points = self.points + self.velma.getContactPointsInFrame(100, 'torso_base', "right")
#            rospy.sleep(0.1)

#        print "left hand"
#        for i in range(0, 100):
#            self.points = self.points + self.velma.getContactPointsInFrame(100, 'torso_base', "left")
#            rospy.sleep(0.1)

#        print "points: %s"%(len(self.points))
#        while not rospy.is_shutdown():
#            rospy.sleep(1)
#        exit(0)

        if False:
            print "collecting contact points with the door..."
            door_points = []

            if simulation_only:
                sim_contacts = [PyKDL.Vector(0.9,0.2, 1.0), PyKDL.Vector(0.9,-0.1, 1.1), PyKDL.Vector(0.9,0.0, 1.3)]
                for sim_c in sim_contacts:
                    for i in range(random.randint(3,20)):
                        door_points.append( sim_c + PyKDL.Vector(random.gauss(0.0, 0.001), random.gauss(0.0, 0.01), random.gauss(0.0, 0.01)) )
            else:
                self.collect_points = True
                thread.start_new_thread(self.pointsCollectorThread, (door_points,'torso_base', 'left'))
                raw_input("Press ENTER to stop collecting contacts...")
                self.collect_points = False
                rospy.sleep(0.5)

            for pt in door_points:
                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=1, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)

            print "done."
            rospy.sleep(0.1)

            # reduce the contact points set and visualise
            door_points = velmautils.reducePointsSet(door_points, 0.05)
            for pt in door_points:
                m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
            rospy.sleep(0.1)

            # estimate the door plane
            T_B_D = velmautils.estPlane(door_points)

            # visualise the door plane
            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=0.5, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(1.0, 1.0, 0.003), T=T_B_D)

        print "collecting contact points with the lock.."
        lock_points = []
        if simulation_only:
            sim_lock_hole = PyKDL.Vector(0.9-0.0036,0.2, 1.3)
            for i in range(400):
                    pt = PyKDL.Vector(random.gauss(0.0, 0.001), random.gauss(0.0, 0.01), random.gauss(0.0, 0.01))
                    if pt.Norm() > 0.008:
                        lock_points.append( sim_lock_hole + pt )
        else:
            self.collect_points = True
            thread.start_new_thread(self.pointsCollectorThread, (lock_points,'torso_link2', 'left'))
            raw_input("Press ENTER to stop collecting contacts...")
            self.collect_points = False
            rospy.sleep(0.5)
        print "done."
        for pt in lock_points:
            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=1, b=0, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.SPHERE, scale=Vector3(0.003, 0.003, 0.003), T=None)
        rospy.sleep(0.1)

        print "collecting contact points with the key end..."
        key_points_E = []
        key_points_B = []
        if simulation_only:
            sim_key_error_O = PyKDL.Vector(-0.002, 0.001, 0.001)
            for i in range(200):
                    pt = sim_key_error_O + PyKDL.Vector(random.gauss(0.0, 0.001), random.gauss(0.0, 0.002), random.gauss(0.0, 0.002))
                    key_points_E.append( self.T_E_O * (self.key_endpoint_O + pt) )
        else:
            self.collect_points = True
#            thread.start_new_thread(self.pointsCollectorThread, (key_points_E,'right_HandPalmLink', 'left'))
            thread.start_new_thread(self.pointsCollectorThread, (key_points_B,'torso_link2', 'left'))
            raw_input("Press ENTER to stop collecting contacts...")
            self.collect_points = False
            rospy.sleep(0.5)
        for pt in key_points_B:
            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=1, b=1, a=1, namespace='default', frame_id='torso_link2', m_type=Marker.SPHERE, scale=Vector3(0.001, 0.001, 0.001), T=None)
        print "done."
        rospy.sleep(0.1)
#        self.points = key_points_E

#        print len(contacts)
#        for pt in contacts:
#            m_id = self.pub_marker.publishSinglePointMarker(pt, m_id, r=0, g=0, b=1, a=1, namespace='default', frame_id='right_HandPalmLink', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)

        while not rospy.is_shutdown():
            rospy.sleep(1)
        exit(0)

        hv = [1.2, 1.2, 1.2, 1.2]
        ht = [3000, 3000, 3000, 3000]
        # set gripper configuration
        if True:
            self.velma.moveHand([0.0, 0.0, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([0.0, 2.3174461354903535, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 0.0, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)
            self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 1.7038538203360971, 1.1437360754475339], hv, ht, 500, True)
            rospy.sleep(3)

        exit(0)
        

        # change the tool - the safe way
        print "switching to joint impedance..."
        if not self.velma.switchToJoint():
            print "ERROR: switchToJoint"
            exit(0)

        rospy.sleep(0.5)

        print "updating tool..."
        self.velma.updateTransformations()
        self.velma.updateAndMoveToolOnly(PyKDL.Frame(self.velma.T_W_E.p+PyKDL.Vector(0.1,0,0)), 0.1)
        rospy.sleep(0.5)
        print "done."

        print "switching to cartesian impedance..."
        if not self.velma.switchToCart():
            print "ERROR: switchToCart"
            exit(0)

        rospy.sleep(0.5)

        # start with very low stiffness
        print "setting stiffness to very low value"
        k_low = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.velma.moveImpedance(k_low, 0.5)
        if self.velma.checkStopCondition(0.5):
            exit(0)
        print "done."

        # door normal
        n_door_B = PyKDL.Vector(-1,0,0)

        T_B_W_in_hole = None
#        T_B_W_in_hole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.361231179791, 0.0198304562193, 0.486979840032, 0.794965045241), PyKDL.Vector(0.60853551459, -0.220618900285, 1.30990416702))

        if T_B_W_in_hole == None:
            print "provide the pose of the key hole..."
            raw_input("Put the grasped key deep into key hole and press Enter to continue...")
            self.velma.updateTransformations()
            T_B_W_in_hole = self.velma.T_B_W
            print "T_B_W_in_hole"
            q = T_B_W_in_hole.M.GetQuaternion()
            p = T_B_W_in_hole.p
            print "T_B_W_in_hole = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s, %s, %s, %s), PyKDL.Vector(%s, %s, %s))"%(q[0], q[1], q[2], q[3], p[0], p[1], p[2])

        raw_input("put the gripper in the safe place near the key hole and press Enter to continue...")
        self.velma.updateTransformations()

        print "moving the desired pose to the current pose..."
        self.velma.moveWrist(self.velma.T_B_W, 1.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(1.0):
            exit(0)
        print "done"

        print "setting stiffness to bigger value"
        k_low_2 = Wrench(Vector3(10.0, 10.0, 10.0), Vector3(5, 5, 5))
        self.velma.moveImpedance(k_low_2, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")

        print "setting stiffness to bigger value"
        k_low_3 = Wrench(Vector3(50.0, 50.0, 50.0), Vector3(25, 25, 25))
        self.velma.moveImpedance(k_low_3, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")
        print "setting stiffness to bigger value"
        k_low_4 = Wrench(Vector3(500.0, 500.0, 500.0), Vector3(250, 250, 250))
        self.velma.moveImpedance(k_low_4, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to set bigger stiffness...")
        print "setting stiffness to bigger value"
        k_big = Wrench(Vector3(2000.0, 2000.0, 2000.0), Vector3(300, 300, 300))
        self.velma.moveImpedance(k_big, 2.0)
        if self.velma.checkStopCondition(2.1):
            exit(0)
        print "done."

        raw_input("Press Enter to move the wrist...")
        T_B_Wd = PyKDL.Frame(n_door_B*0.1) * T_B_W_in_hole
        print "moving the wrist..."
        self.velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(10.0):
            exit(0)
        print "done"

        raw_input("Press Enter to move the wrist...")
        T_B_Wd = PyKDL.Frame(n_door_B*0.0) * T_B_W_in_hole
        print "moving the wrist..."
        self.velma.moveWrist(T_B_Wd, 10.0, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(10.0):
            exit(0)
        print "done"

        # start with very low stiffness
        print "setting stiffness to very low value"
        k_low = Wrench(Vector3(1.0, 1.0, 1.0), Vector3(0.5, 0.5, 0.5))
        self.velma.moveImpedance(k_low, 0.5)
        if self.velma.checkStopCondition(0.5):
            exit(0)
        print "done."

        self.velma.moveHand([2.0177062895374993, 2.3174461354903535, 1.7038538203360971-5.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)
        self.velma.moveHand([2.0177062895374993-5.0/180.0*math.pi, 2.3174461354903535, 1.7038538203360971-5.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)
        self.velma.moveHand([2.0177062895374993-10.0/180.0*math.pi, 2.3174461354903535, 1.7038538203360971-10.0/180.0*math.pi, 1.1437360754475339], hv, ht, 10000, True)
        rospy.sleep(3)

        exit(0)


        
        graspable_object_name = "big_box"

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # list all packages, equivalent to rospack list
        #rospack.list_pkgs() 

        # get the file path for rospy_tutorials
        filename_environment = rospack.get_path('velma_scripts') + '/data/romoco/romoco.env.xml'
        filename_objectmarker = rospack.get_path('velma_scripts') + '/data/romoco/object_marker.txt'
        filename_wrenches = rospack.get_path('velma_scripts') + '/data/romoco/wrenches_' + graspable_object_name + '.txt'


        simulation_only = False
        if simulation_only:
            time_mult = 5.0
        else:
            time_mult = 20.0
        m_id = 0


        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        #
        # Initialise Openrave
        #
        self.openrave = openraveinstance.OpenraveInstance()
        self.openrave.startOpenrave(filename_environment)

        self.openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))

        #
        # Initialise dynamic objects and their marker information
        #
        dyn_objects_map = set()
        dyn_objects_map.add("table")
        dyn_objects_map.add("big_box")

        self.dyn_obj_markers = {}

        with open(filename_objectmarker, 'r') as f:
            for line in f:
                line_s = line.split()
                obj_name = line_s[0]
                markers_count = int(line_s[1])
                if obj_name in dyn_objects_map:
                     self.dyn_obj_markers[obj_name] = []
                     for i in range(markers_count):
                         marker_id = int(line_s[2+i*8+0])
                         frame = PyKDL.Frame(
                         PyKDL.Rotation.Quaternion(float(line_s[2+i*8+1]), float(line_s[2+i*8+2]), float(line_s[2+i*8+3]), float(line_s[2+i*8+4])),
                         PyKDL.Vector(float(line_s[2+i*8+5]), float(line_s[2+i*8+6]), float(line_s[2+i*8+7])))
                         self.dyn_obj_markers[obj_name].append([marker_id, frame])

        # simulation
        if simulation_only:
            self.getMarkerPose = self.getMarkerPoseFake
            self.getCameraPose = self.getCameraPoseFake

        self.T_World_Br = PyKDL.Frame(PyKDL.Vector(0,0,0.1))

        self.velma_solvers = velmautils.VelmaSolvers()

        self.velma = None

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

        self.openrave.updateRobotConfigurationRos(self.velma.js_pos)

        self.allowUpdateObjects()
        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        self.velma.updateTransformations()

        # TEST: moveWrist
#        T_B_Ed = PyKDL.Frame(PyKDL.Vector(0,0.0,0.4)) * self.openrave.getLinkPose("right_HandPalmLink")        
#        T_B_Wd = T_B_Ed * self.velma.T_E_W
#        init_js = self.openrave.getRobotConfigurationRos()
#        self.velma.switchToCart()
#        self.velma.moveTool(PyKDL.Frame(PyKDL.Vector(0,0,-0.3)), 2, stamp=None)
#        rospy.sleep(2)
#        self.velma.moveWrist(T_B_Wd, 4, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
#        exit(0)

        k_pregrasp = Wrench(Vector3(1000.0, 1000.0, 1000.0), Vector3(300.0, 300.0, 300.0))
        k_grasp = Wrench(Vector3(500.0, 500.0, 500.0), Vector3(150.0, 150.0, 150.0))

        # reset the gripper
        self.velma.resetFingers()
        self.velma.calibrateTactileSensors()
        self.velma.setMedianFilter(8)

        raw_input("Press Enter to enable cartesian impedance...")
        if self.velma.checkStopCondition():
            exit(0)
        self.velma.switchToCart()

        # start with very low stiffness
        print "setting stiffness to very low value"
        self.velma.moveImpedance(self.velma.k_error, 0.5)
        if self.velma.checkStopCondition(0.5):
            exit(0)

        raw_input("Press Enter to continue...")
        if self.velma.checkStopCondition():
            exit(0)

        self.velma.updateTransformations()
#        self.velma.updateAndMoveTool( PyKDL.Frame(), 5.0 )
        self.velma.updateAndMoveTool( self.velma.T_W_E * PyKDL.Frame(PyKDL.Vector(0,0,0.17)), 5.0 )
        if self.velma.checkStopCondition(6.0):
            exit(0)

        raw_input("Press Enter to continue...")
        print "setting stiffness to bigger value"
        self.velma.moveImpedance(k_pregrasp, 3.0)
        if self.velma.checkStopCondition(3.0):
            exit(0)

        self.velma.updateTransformations()

        # TEST: planning
        if False:
            self.openrave.updateRobotConfigurationRos(self.velma.js_pos)

            init_T_B_E = self.velma.T_B_W * self.velma.T_W_E
            T_B_Ed = init_T_B_E * PyKDL.Frame(PyKDL.Vector(0.1,0.1,0))

            # plan first trajectory (in configuration space)
            traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
            if traj == None:
                print "colud not plan trajectory in configuration space"
                return None, None

            plan = []
            plan.append(["move_joint", traj])

            # calculate the destination pose of the end effector
            T_B_Ed = init_T_B_E
            T_B_Wd = T_B_Ed * self.velma.T_E_W

            # interpolate trajectory for the second motion (in the cartesian space)
            plan.append(["move_cart", T_B_Wd])

            self.showPlan(plan)

            print "executing plan..."
            self.executePlan(plan, time_mult)

            exit(0)

        self.disallowUpdateObjects()

        self.openrave.prepareGraspingModule(graspable_object_name, force_load=False)

        try:
            print "trying to read wrenches for each grasp from file"
            self.openrave.loadWrenchesforAllGrasps(graspable_object_name, filename_wrenches)
            print "done."
        except IOError as e:
            print "could not read from file:"
            print e
            print "generating grapsing data..."
            self.openrave.generateWrenchesforAllGrasps(graspable_object_name)
            print "done."
            print "saving grasping data to file..."
            self.openrave.saveWrenchesforAllGrasps(graspable_object_name, filename_wrenches)
            print "done."

        # TEST
#        T_B_Ed = PyKDL.Frame(PyKDL.Vector(0,0,0.2)) * self.openrave.getLinkPose("right_HandPalmLink")

#        self.openrave.getGraspsForObjectTransport(graspable_object_name, [PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi), PyKDL.Vector(0.5,-0.1,1.0))])
#        traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
#        if traj == None:
#            print "colud not plan trajectory"
#            exit(0)
#        duration = math.fsum(traj[3])
#        raw_input("Press Enter to visualize the trajectory...")
#        if self.velma.checkStopCondition():
#            exit(0)
#        self.openrave.showTrajectory(duration * time_mult * 1, qar_list=traj[4])

#        raw_input(".")
#        exit(0)


        #
        # transport task specification
        #

#        task_variant = "liftup"
        task_variant = "rot"

        # current object pose
        current_T_B_O = self.openrave.getPose(graspable_object_name)

#        current_T_B_O = current_T_B_O * PyKDL.Frame(PyKDL.Rotation.RotX(45.0/180.0*math.pi))
        #self.openrave.updatePose(graspable_object_name, current_T_B_O)

        if task_variant == "liftup":
            # object destination poses
            T_B_O_trans = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * current_T_B_O

            transport_T_B_O = []
            transport_T_B_O.append(current_T_B_O)
            transport_T_B_O.append( T_B_O_trans )
        elif task_variant == "rot":
            # object destination poses
            T_B_O_trans = PyKDL.Frame(PyKDL.Vector(0,0,0.05)) * current_T_B_O
            TR_B_O_rot = (PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi)) * current_T_B_O).M
            TT_B_O_rot = current_T_B_O.p + PyKDL.Vector(0,0,0.1)
            T_B_O_rot = PyKDL.Frame(TR_B_O_rot, TT_B_O_rot)

            transport_T_B_O = []
            transport_T_B_O.append(current_T_B_O)
            transport_T_B_O.append( T_B_O_trans )
            transport_T_B_O.append( T_B_O_rot )
        else:
            print "wrong task: ", task_variant
            exit(0)

        print "transport_T_B_O:"
        print transport_T_B_O

        #
        # definition of the expected external wrenches for lift-up task for objects c.o.m. in the World frame
        #
        ext_wrenches_W = []
        # main force (downward)
        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(0,0,-1), PyKDL.Vector(0,0,0)))
        # disturbance forces
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(0,0,0.1), PyKDL.Vector()))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(0,0.1,0), PyKDL.Vector()))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(0,-0.1,0), PyKDL.Vector()))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(0.1,0,0), PyKDL.Vector()))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(-0.1,0,0), PyKDL.Vector()))
        # disturbance torques
        max_torque = 0.15
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(max_torque*0.1, 0, 0)))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(-max_torque*0.1, 0, 0)))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(0, max_torque*0.1, 0)))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(0, -max_torque*0.1, 0)))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(0, 0, max_torque*0.1)))
#        ext_wrenches_W.append(PyKDL.Wrench(PyKDL.Vector(), PyKDL.Vector(0, 0, -max_torque*0.1)))

        ext_wrenches_O = self.calculateWrenchesForTransportTask(ext_wrenches_W, transport_T_B_O)
        print ext_wrenches_O



        # TEST: visualise the quality of all grasps
        if False:
            m_id = 0
            m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0.2, g=0.2, b=0.2, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.177*2, 0.03*2, 0.03*2), T=current_T_B_O)
            print "generating GWS for all grasps..."
            self.openrave.generateGWSforAllGrasps(graspable_object_name)
            for grasp_idx in range(self.openrave.getGraspsCount(graspable_object_name)):
                gws = self.openrave.getGWSforGraspId(graspable_object_name, grasp_idx)
                q_min = None
                for wr_O in ext_wrenches_O:
                    q = self.openrave.getQualityMeasure2(gws, wr_O)
                    if q_min == None or q < q_min:
                        q_min = q
                grasp = self.openrave.getGrasp(graspable_object_name, grasp_idx)
                T_B_E = self.openrave.getGraspTransform(graspable_object_name, grasp)
                scale = q_min * 0.1
                m_id = self.pub_marker.publishSinglePointMarker(T_B_E * PyKDL.Vector(), m_id, r=0.6, g=0.6, b=0.6, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(scale, scale, scale), T=None)
            raw_input(".")
            exit(0)


        print "calculating set of possible grasps..."
        valid_indices = self.openrave.getGraspsForObjectTransport(graspable_object_name, transport_T_B_O)
        print "done"

        print "calculating quality measure..."
        evaluated_grasps = []
        for grasp_idx in valid_indices:
            gws = self.openrave.getGWSforGraspId(graspable_object_name, grasp_idx)
            q_min = None
            for wr_O in ext_wrenches_O:
                q = self.openrave.getQualityMeasure2(gws, wr_O)
                if q_min == None or q < q_min:
                    q_min = q
            evaluated_grasps.append([q_min, grasp_idx])
        print "done."

        evaluated_grasps_sorted = sorted(evaluated_grasps, key=operator.itemgetter(0), reverse=True)

        # show grasps sorted by their scores
#        for q, grasp_idx in evaluated_grasps_sorted:
#            print "showing the grasp..."
#            print q
#            grasp = self.openrave.getGrasp(graspable_object_name, grasp_idx)
#            self.openrave.getFinalConfig(graspable_object_name, grasp, show=True)

#        print "showing the grasp..."
#        self.openrave.getFinalConfig(graspable_object_name, grasp, show=True)

        q_max = evaluated_grasps_sorted[0][0]
        print "max quality: %s"%(q_max)
        print "len(evaluated_grasps_sorted)", len(evaluated_grasps_sorted)

        evaluated_plans = []
        penalty_threshold = 1000.0
        while len(evaluated_grasps_sorted) > 0:
            best_grasp_q, best_grasp_idx = evaluated_grasps_sorted.pop(0)
            if best_grasp_q < 0.9 * q_max:
                print best_grasp_q
                break

            grasp = self.openrave.getGrasp(graspable_object_name, best_grasp_idx)

            penalty, plan = self.makePlan(graspable_object_name, grasp, transport_T_B_O, penalty_threshold)

            print best_grasp_q, penalty
            if penalty != None:
                penalty_threshold = penalty
                evaluated_plans.append([penalty, best_grasp_idx, plan])
                if penalty < 0.000001:
                    break

        evaluated_plans_sorted = sorted(evaluated_plans, key=operator.itemgetter(0))
        print "best plan: %s"%(evaluated_plans_sorted[0][0])

        self.showPlan(evaluated_plans_sorted[0][2])

        self.executePlan(evaluated_plans_sorted[0][2], time_mult)
#        grasp = self.openrave.getGrasp(graspable_object_name, evaluated_plans_sorted[0][1])
#        print "showing the grasp..."
#        self.openrave.getFinalConfig(graspable_object_name, grasp, show=True)

        exit(0)

if __name__ == '__main__':

    rospy.init_node('door_key')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


