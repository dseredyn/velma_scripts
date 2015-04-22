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

            if self.allow_update_objects_pose == None or not self.allow_update_objects_pose:
                continue
            for obj_name in self.dyn_obj_markers:
                obj = self.dyn_obj_markers[obj_name]
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

    def spin(self):

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

        rospy.sleep(1.0)
        self.velma.updateTransformations()

        self.openrave.updateRobotConfigurationRos(self.velma.js_pos)

        self.allowUpdateObjects()
        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

        self.velma.updateTransformations()


        init_pos1 = [1.2521458864212036, -1.6721032451629638, 1.526840851020813, 1.6542659255981444, 0.3262618441104889, -1.830127703857422, 0.06183132529258728]
        init_pos2 = [1.4523228406906128, -1.5725950788497924, -1.7778874185562135, -1.7074530866622926, 0.40205069966316226, -1.3939709541320802, 0.053711459040641785]

        self.velma.switchToJoint()

        self.openrave.extendAllObjects(0.02)
        traj = self.openrave.planMoveForRightArm( None, init_pos1)
        self.openrave.restoreExtendedObjects()
        if traj == None:
            print "colud not plan trajectory in configuration space"
            return None, None

        duration = math.fsum(traj[3])
        self.openrave.showTrajectory(duration * 5.0, qar_list=traj[4])

        raw_input("Press Enter to execute the trajectory in " + str(duration * time_mult) + "s...")
        if self.velma.checkStopCondition():
            exit(0)

        self.velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
        if self.velma.checkStopCondition(duration * time_mult + 1.0):
            exit(0)

if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


