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

import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_srvs.msg import *
from barrett_hand_controller_srvs.srv import *
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

    def getMarkerPoseFake(self, marker_id, wait = True, timeBack = None):
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

    def getCameraPoseFake(self):
        return PyKDL.Frame(PyKDL.Rotation.RotY(135.0/180.0*math.pi), PyKDL.Vector(0.4, 0, 1.4)) * PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi))

    def poseUpdaterThread(self, args, *args2):
        index = 0
        z_limit = 0.3
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
#            self.velma.publishJointStates()
#            continue

            if self.allow_update_objects_pose == None or not self.allow_update_objects_pose:
                continue
            for obj_name in self.dyn_obj_markers:
                obj = self.dyn_obj_markers[obj_name]
#            for obj in self.objects:
                visible_markers_Br_Co = []
                visible_markers_weights_ori = []
                visible_markers_weights_pos = []
                visible_markers_idx = []
                for marker in obj:#.markers:
                    T_Br_M = self.getMarkerPose(marker[0], wait = False, timeBack = 0.3)
                    if T_Br_M != None and self.velma != None:
                        T_B_C = self.getCameraPose()
                        T_C_M = T_B_C.Inverse() * T_Br_M
                        v = T_C_M * PyKDL.Vector(0,0,1) - T_C_M * PyKDL.Vector()
                        if v.z() > -z_limit:
                            continue
                        # v.z() is in range (-1.0, -0.3)
                        weight = ((-v.z()) - z_limit)/(1.0-z_limit)
                        if weight > 1.0 or weight < 0.0:
                            print "error: weight==%s"%(weight)
                        T_Co_M = marker[1]
                        T_Br_Co = T_Br_M * T_Co_M.Inverse()
                        visible_markers_Br_Co.append(T_Br_Co)
                        visible_markers_weights_ori.append(1.0-weight)
                        visible_markers_weights_pos.append(weight)
                        visible_markers_idx.append(marker[0])
                if len(visible_markers_Br_Co) > 0:
#                    if obj.name == "object":
#                        print "vis: %s"%(visible_markers_idx)
#                        print "w_o: %s"%(visible_markers_weights_ori)
#                        print "w_p: %s"%(visible_markers_weights_pos)

                    # first calculate mean pose without weights
                    R_B_Co = velmautils.meanOrientation(visible_markers_Br_Co)[1]
                    p_B_Co = velmautils.meanPosition(visible_markers_Br_Co)
                    T_B_Co = PyKDL.Frame(copy.deepcopy(R_B_Co.M), copy.deepcopy(p_B_Co))
                    distances = []
                    for m_idx in range(0, len(visible_markers_Br_Co)):
                        diff = PyKDL.diff(T_B_Co, visible_markers_Br_Co[m_idx])
                        distances.append( [diff, m_idx] )
                    Br_Co_list = []
                    weights_ori = []
                    weights_pos = []
                    for d in distances:
                        if d[0].vel.Norm() > 0.04 or d[0].rot.Norm() > 15.0/180.0*math.pi:
                            continue
                        Br_Co_list.append( visible_markers_Br_Co[d[1]] )
                        weights_ori.append( visible_markers_weights_ori[d[1]] )
                        weights_pos.append( visible_markers_weights_pos[d[1]] )

                    if len(Br_Co_list) > 0:
                        R_B_Co = velmautils.meanOrientation(Br_Co_list, weights=weights_ori)[1]
                        p_B_Co = velmautils.meanPosition(Br_Co_list, weights=weights_pos)
#                        obj.updatePose( PyKDL.Frame(copy.deepcopy(R_B_Co.M), copy.deepcopy(p_B_Co)) )
                        self.openrave.updatePose(obj_name, T_Br_Co)

            index += 1
            if index >= 100:
                index = 0

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
                T_B_Wd = stage[1]
                init_js = self.openrave.getRobotConfigurationRos()
                traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_Wd)
                raw_input("Press Enter to visualize the cartesian trajectory...")
                self.openrave.showTrajectory(3.0, qar_list=traj)
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
                T_B_Wd = stage[1]
                duration = self.velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
                raw_input("Press Enter to move the robot in " + str(duration) + " s...")
                if self.velma.checkStopCondition():
                    exit(0)
                self.velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if self.velma.checkStopCondition(duration):
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
                self.velma.move_hand_client(q)

    def makePlan(self, graspable_object_name, grasp, T_B_Od, penalty_threshold):

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
        traj = self.openrave.planMoveForRightArm(T_B_Ed, None, penalty_threshold=penalty_threshold)
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
        T_B_Ed = T_B_Od * T_E_O.Inverse()
        T_B_Wd = T_B_Ed * self.velma.T_E_W

        # interpolate trajectory for the second motion (in the cartesian space)
        init_js = self.openrave.getRobotConfigurationRos()
        traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_Wd)
        if traj == None:
            print "could not plan trajectory in cartesian space"
            self.openrave.release()
            self.openrave.updatePose(graspable_object_name, init_T_B_O)
            self.openrave.updateRobotConfigurationRos(config)
            return None, None

        plan_ret.append(["move_cart", T_B_Wd])

        self.openrave.release()
        self.openrave.updatePose(graspable_object_name, init_T_B_O)

        self.openrave.updateRobotConfigurationRos(config)

        return penalty, plan_ret

    def spin(self):

        graspable_object_name = "big_box"

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # list all packages, equivalent to rospack list
        #rospack.list_pkgs() 

        # get the file path for rospy_tutorials
        filname_environment = rospack.get_path('velma_scripts') + '/data/romoco/romoco.env.xml'
        filname_objectmarker = rospack.get_path('velma_scripts') + '/data/romoco/object_marker.txt'
        filename_wrenches = rospack.get_path('velma_scripts') + '/data/romoco/wrenches_' + graspable_object_name + '.txt'


        simulation_only = True
        if simulation_only:
            time_mult = 5.0
        else:
            time_mult = 8.0
        m_id = 0


        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        #
        # Initialise Openrave
        #
        self.openrave = openraveinstance.OpenraveInstance()
        self.openrave.startOpenrave(filname_environment)

        self.openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))

        #
        # Initialise dynamic objects and their marker information
        #
        dyn_objects_map = set()
        dyn_objects_map.add("table")
        dyn_objects_map.add("big_box")

        self.dyn_obj_markers = {}

        with open(filname_objectmarker, 'r') as f:
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
        self.velma.updateAndMoveTool( self.velma.T_W_E, 1.0 )
        if self.velma.checkStopCondition(2.0):
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

        # current object pose
        current_T_B_O = self.openrave.getPose(graspable_object_name)

#        current_T_B_O = current_T_B_O * PyKDL.Frame(PyKDL.Rotation.RotX(45.0/180.0*math.pi))
        #self.openrave.updatePose(graspable_object_name, current_T_B_O)

        # object destination poses
        T_B_O_trans = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * current_T_B_O
        T_B_O_rot = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * current_T_B_O * PyKDL.Frame(PyKDL.Rotation.RotY(-90.0/180.0*math.pi))

        transport_T_B_O = []
        transport_T_B_O.append(current_T_B_O)

        transport_T_B_O.append( T_B_O_rot )   # first variant (lying)
#        transport_T_B_O.append( T_B_O_trans )   # second variant (standing)

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
#        m_id = 0
#        self.openrave.generateGWSforAllGrasps(graspable_object_name)
#        for grasp_idx in range(self.openrave.getGraspsCount(graspable_object_name)):
#            gws = self.openrave.getGWSforGraspId(graspable_object_name, grasp_idx)
#            q_min = None
#            for wr_O in ext_wrenches_O:
#                q = self.openrave.getQualityMeasure2(gws, wr_O)
#                if q_min == None or q < q_min:
#                    q_min = q
#            grasp = self.openrave.getGrasp(graspable_object_name, grasp_idx)
#            T_B_E = self.openrave.getGraspTransform(graspable_object_name, grasp)
#            scale = q_min * 0.1
#            m_id = self.pub_marker.publishSinglePointMarker(T_B_E * PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, scale=Vector3(scale, scale, scale), T=None)
#        raw_input(".")
#        exit(0)


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

        evaluated_plans = []
        penalty_threshold = 1000.0
        while len(evaluated_grasps_sorted) > 0:
            best_grasp_q, best_grasp_idx = evaluated_grasps_sorted.pop(0)
            if best_grasp_q < 0.95 * q_max:
                break

            grasp = self.openrave.getGrasp(graspable_object_name, best_grasp_idx)

            penalty, plan = self.makePlan(graspable_object_name, grasp, transport_T_B_O[1], penalty_threshold)

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

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


