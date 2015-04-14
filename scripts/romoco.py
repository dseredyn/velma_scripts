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
from numpy import *
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma import Velma
from velmasim import VelmaSim
import random
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import openraveinstance
import itertools
import grip
import operator

import velma_fk_ik

import rospkg

from sklearn import manifold, datasets
#from sklearn.cluster import AgglomerativeClustering

# reference frames:
# B - robot's base
# R - camera
# W - wrist
# E - gripper
# F - finger distal link
# T - tool
# C - current contact point
# N - the end point of finger's nail
# J - jar marker frame (jar cap)

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
        T_Bm_Gm = PyKDL.Frame( PyKDL.Rotation.RotZ(-30.0/180.*math.pi), PyKDL.Vector(0.1,-0.1,0.06) )
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

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(KDLToOpenrave(self.T_World_Br*T_Br_Bo))
            else:
#                print "openrave: could not find body: %s"%(name)
                pass
            self.env.UpdatePublishedBodies()

    def poseUpdaterThread(self, args, *args2):
        index = 0
        z_limit = 0.3
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.velma.publishJointStates()
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
                        T_B_C = self.velma.T_B_C
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
                        self.updatePose(obj_name, T_Br_Co)

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

    def spin(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # list all packages, equivalent to rospack list
        #rospack.list_pkgs() 

        # get the file path for rospy_tutorials
        filneme_environment = rospack.get_path('velma_scripts') + '/data/romoco/romoco.env.xml'
        filneme_objectmarker = rospack.get_path('velma_scripts') + '/data/romoco/object_marker.txt'

        simulation_only = True
        if simulation_only:
            time_mult = 5.0
        else:
            time_mult = 8.0
        m_id = 0


        m_id = 0
        self.pub_marker.eraseMarkers(0,3000, frame_id='world')

        #
        # Init Openrave
        #
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args()
        self.env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)

        self.env.Load(filneme_environment)#'romoco/romoco.env.xml')

        # set the camera in openrave
        if True:
            cam_pos = PyKDL.Vector(2.0, 0.0, 2.0)
            target_pos = PyKDL.Vector(0.60, 0.0, 1.10)

            cam_z = target_pos - cam_pos
            focalDistance = cam_z.Norm()
            cam_y = PyKDL.Vector(0,0,-1)
            cam_x = cam_y * cam_z
            cam_y = cam_z * cam_x
            cam_x.Normalize()
            cam_y.Normalize()
            cam_z.Normalize()
            cam_T = PyKDL.Frame(PyKDL.Rotation(cam_x,cam_y,cam_z), cam_pos)
            
            self.env.GetViewer().SetCamera(KDLToOpenrave(cam_T), focalDistance)

        self.openrave_robot = self.env.GetRobots()[0]

        dof_values = self.openrave_robot.GetDOFValues()
        dof_values[1] = 0.0
        self.openrave_robot.SetDOFValues(dof_values)

        dyn_objects_map = set()
        dyn_objects_map.add("table")
        dyn_objects_map.add("big_box")

        self.dyn_obj_markers = {}

        with open(filneme_objectmarker, 'r') as f:
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

        # load and init ik solver for right hand
        velma_ikr = velmautils.VelmaIkSolver()
        velma_ikr.initIkSolver()

        # simulation
        if simulation_only:
            self.getMarkerPose = self.getMarkerPoseFake

        self.T_World_Br = PyKDL.Frame(PyKDL.Vector(0,0,0.1))

        # create Openrave interface
#        self.openrave = openraveinstance.OpenraveInstance(PyKDL.Frame(PyKDL.Vector(0,0,0.1)))
#        self.openrave.startNewThread()

#        self.waitForOpenraveInit()

        self.velma = None

        if simulation_only:
            # create the robot interface for simulation
#            self.velma = VelmaSim(self.openrave, velma_ikr)
            self.velma = VelmaSim(None, velma_ikr)
        else:
            # create the robot interface for real hardware
            self.velma = Velma()

        self.velma.updateTransformations()

        self.allowUpdateObjects()
        # start thread for updating objects' positions in openrave
        thread.start_new_thread(self.poseUpdaterThread, (None,1))

#        self.openrave.addRobotInterface(self.velma)

        self.velma.updateTransformations()

#        camera_fov_x = 50.0/180.0*math.pi
#        camera_fov_y = 40.0/180.0*math.pi

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
        self.velma_init_T_B_W = copy.deepcopy(self.velma.T_B_W)

        if simulation_only:
#            self.velma.qar[1] += 10.0/180.0*math.pi
            self.velma.qar[6] = 90.0/180.0*math.pi
            rospy.sleep(1.0)

        sim_grips = []

        self.disallowUpdateObjects()

        vertices, faces = self.openrave.getMesh("object")

        print vertices
        print faces

#        com_pt = velmautils.generateComSamples(vertices, faces, 2000)
#        obj_grasp.addComPoints(com_pt)

        self.openrave.prepareGraspingModule("object", force_load=False)

        raw_input(".")
        exit(0)

        try:
            print "trying to read grasping data from file"
            with open('sim_grips_' + obj_model + '.txt', 'r') as f:
                for line in f:
                    if line == 'None' or line == 'None\n':
                        sim_grips.append(None)
                    else:
                        gr = grip.Grip(obj_grasp)
                        gr.fromStr(line)
                        sim_grips.append(gr)
        except IOError as e:
            print "could not read from file:"
            print e
            print "generating grapsing data..."
            self.disallowUpdateObjects()
            T_B_Oorig = self.openrave.getPose("object")
            # move the object 10 meters above
            self.openrave.updatePose("object", PyKDL.Frame(PyKDL.Vector(0,0,10)))
            T_B_O = self.openrave.getPose("object")
            T_O_B = T_B_O.Inverse()
            vertices, faces = self.openrave.getMesh("object")
            # get the possible grasps for the current scene

            valid_grasps = 0
            grips = []
            for idx in range(0, self.openrave.getGraspsCount("object")):
                if idx % 100 == 0:
                    print "index: %s"%(idx)
                sim_grips.append(None)
                grasp = self.openrave.getGrasp("object", idx)
                q, contacts, normals = self.openrave.getFinalConfig("object", grasp)
                if contacts == None:
                    contacts = []
#                print "grasp_idx: %s   contacts: %s"%(idx, len(contacts))
                if len(contacts) == 0:
                    continue

                T_B_E = self.openrave.getGraspTransform("object", grasp)

                gr = grip.Grip(obj_grasp)
                self.pub_marker.eraseMarkers(2, m_id)
                m_id = 2

                checked_contacts = []
                contacts_groups = []
                while len(checked_contacts) < len(contacts):
                    pos = None
                    # group the contacts
                    for c_idx in range(0, len(contacts)):
                        if c_idx in checked_contacts:
                            continue
                        c_O = T_B_O.Inverse() * contacts[c_idx]
                        if pos == None:
                            pos = c_O
                            checked_contacts.append(c_idx)
                            contacts_groups.append([])
                            contacts_groups[-1].append(c_idx)
                        else:
                            dist = (pos - c_O).Norm()
                            if dist < 0.02:
                                checked_contacts.append(c_idx)
                                contacts_groups[-1].append(c_idx)

                if len(contacts_groups) < 3:
                    continue

                centers = []
#                center_normals = []
                for g in contacts_groups:
                    center = PyKDL.Vector()
#                    normal = PyKDL.Vector()
                    for c in g:
                        center += T_B_O.Inverse() * contacts[c]
#                        normal += T_B_O.Inverse() * normals[c]
                    center *= 1.0/len(g)
                    centers.append(center)
#                    normal.Normalize()
#                    center_normals.append(normal)

#                for i in range(0, len(centers)):
#                        c = centers[i]
#                        nz = center_normals[i]
#                        # create frame for contact
#                        if abs(nz.z()) < 0.8:
#                            nx = PyKDL.Vector(0,0,1)
#                        elif abs(nz.y()) < 0.8:
#                            nx = PyKDL.Vector(0,1,0)
#                        else:
#                            nx = PyKDL.Vector(1,0,0)
#                        ny = nz * nx
#                        nx = ny * nz
#                        fr = PyKDL.Frame( PyKDL.Rotation(nx, ny, nz), c)
#                        # get the finger in contact index
#                        P_F = PyKDL.Vector(0.0510813, -0.0071884, 0.0)
#                        finger_idx_min = -1
#                        d_min = 1000000.0
#                        fr_B_p = T_B_O * fr * PyKDL.Vector(0,0,0)
#                        for finger_idx in range(0,3):
#                            pt_B = T_B_E * velma.get_T_E_Fd( finger_idx, q[finger_idx], q[3]) * P_F
#                            d = (fr_B_p - pt_B).Norm()
#                            if d < d_min:
#                                d_min = d
#                                finger_idx_min = finger_idx
#                        gr.addContact(fr, finger_idx_min)
                    

                for c in centers:
                        points = velmautils.sampleMesh(vertices, faces, 0.003, [c], 0.005)
                        if len(points) < 3:
                            continue
                        # get the contact surface normal
                        fr = velmautils.estPlane(points)
                        # set the proper direction of the contact surface normal (fr.z axis)
                        fr_B_p = T_B_O * fr * PyKDL.Vector(0,0,0)
                        fr_B_z = PyKDL.Frame( copy.deepcopy((T_B_O * fr).M) ) * PyKDL.Vector(0,0,1)
                        # get the finger in contact index
                        P_F = PyKDL.Vector(0.0510813, -0.0071884, 0.0)
                        finger_idx_min = -1
                        d_min = 1000000.0
                        for finger_idx in range(0,3):
                            pt_B = T_B_E * velma.get_T_E_Fd( finger_idx, q[finger_idx], q[3]) * P_F
                            d = (fr_B_p - pt_B).Norm()
                            if d < d_min:
                                d_min = d
                                finger_idx_min = finger_idx
                        n_B = PyKDL.Frame( copy.deepcopy((T_B_E * velma.get_T_E_Fd( finger_idx_min, q[finger_idx_min], q[3])).M) ) * PyKDL.Vector(1,-1,0)
                        if PyKDL.dot(n_B, fr_B_z) < 0:
                            fr = fr * PyKDL.Frame(PyKDL.Rotation.RotX(math.pi))
                        # add the contact to the grip description
                        gr.addContact(fr, finger_idx_min)

                valid_grasps += 1
                if len(gr.contacts) == 3:
                    sim_grips[-1] = copy.deepcopy(gr)

            print "added grasps: %s / %s"%(valid_grasps, len(sim_grips))
            print "writing grasping data to file"
            with open('sim_grips_' + obj_model + '.txt', 'w') as f:
                for gr in sim_grips:
                    if gr == None:
                        f.write('None\n')
                    else:
#                        print "%s"%(len(gr.contacts))
                        f.write(gr.toStr() + '\n')
            self.openrave.updatePose("object", T_B_Oorig)
            self.allowUpdateObjects()

        if False:
            velmautils.comSamplesUnitTest(self.openrave, self.pub_marker, "cbeam")
            exit(0)

        if False:
            velmautils.updateComUnitTest(self.openrave, self.pub_marker, "object")
            exit(0)

        if False:
            print "grasps: %s  %s"%(len(sim_grips), self.openrave.getGraspsCount("object"))

            # verify the grasps
            for i in range(0, len(sim_grips)):
                if sim_grips[i] == None:
                    continue
                if len(sim_grips[i].contacts) != 3:
                    print "ERROR: len(sim_grips[i].contacts) != 3"
                    grasp = self.openrave.getGrasp("object", i)
                    self.openrave.getFinalConfig("object", grasp, show=True)
                    exit(0)
                fingers = [sim_grips[i].contacts[0][0], sim_grips[i].contacts[1][0], sim_grips[i].contacts[2][0]]
                if (not 0 in fingers) or (not 1 in fingers) or (not 2 in fingers):
                    print "ERROR: grasp %s: not all fingers have contacts"%(i)
                    sim_grips[i] = None
#                    grasp = self.openrave.getGrasp("object", i)
#                    self.openrave.getFinalConfig("object", grasp, show=True)
                    continue


            valid_indices = self.openrave.generateGrasps("object", show=False, checkcollision=True, checkik=False, checkgrasper=True)
            print "valid grasps: %s"%(len(valid_indices))

            for i in range(0, len(sim_grips)):
                if not i in valid_indices:
                    sim_grips[i] = None

            max_valid_grasps = 300
            valid_grasps = 0
            for i in range(0, len(sim_grips)):
                if sim_grips[i] != None:
                    valid_grasps += 1

            print "valid grasps: %d"%(valid_grasps)
            print "reducing to %s"%(max_valid_grasps)
            g_idx = 0
            while valid_grasps > max_valid_grasps:
                if sim_grips[g_idx] != None and random.random() > 0.8:
                    sim_grips[g_idx] = None
                    valid_grasps -= 1
                g_idx = (g_idx + 1) % len(sim_grips)

            print "valid grasps: %d"%(valid_grasps)
            sim_grips_valid = []
            sim_grips_valid_idx = []
            valid_grasps = 0
            for i in range(0, len(sim_grips)):
                if sim_grips[i] != None:
                    valid_grasps += 1
                    sim_grips_valid.append(copy.deepcopy(sim_grips[i]))
                    sim_grips_valid_idx.append(i)
            print "valid grasps: %d"%(valid_grasps)

            print "showing 10 random grips"
            random_idx = -1
            random_grasps = []
            # get random grasp
            while True:
                random_idx = random.randint(0, len(sim_grips_valid)-1)
                if sim_grips_valid[random_idx] != None:
                    grasp = self.openrave.getGrasp("object", sim_grips_valid_idx[random_idx])
                    random_grasps.append(grasp)
                    self.openrave.getFinalConfig("object", grasp, show=True)
                if len(random_grasps) >= 10:
                    break

#            print "random grasp: %s"%(random_idx)
#            grasp = self.openrave.getGrasp("object", random_idx)
#            self.openrave.getFinalConfig("object", grasp, show=True)

#            self.openrave.showFinalConfigs("object", random_grasps)
#            exit(0)






            print "searching for grasps..."
            checked_idx = []

            dist_cache = numpy.zeros( (len(sim_grips_valid), len(sim_grips_valid)) )
            for i in range(0, len(sim_grips_valid)):
                for j in range(0, len(sim_grips_valid)):
                    dist_cache[i][j] = -1.0

            axis_B = PyKDL.Vector(0,0,1)
            T_B_O = self.openrave.getPose("object")
            T_O_B = T_B_O.Inverse()
            axis_O = PyKDL.Frame(T_O_B.M) * axis_B

            max_dist = None
            # calculate distance between all grips
            for i in range(0, len(sim_grips_valid)):
                if sim_grips_valid[i] == None:
                    continue
                print "calculating grip distances: %s%%"%(100.0 * float(i)/len(sim_grips_valid))
                for j in range(0, len(sim_grips_valid)):
                    if sim_grips_valid[j] == None:
                        continue
                    if i == j:
                        dist_cache[j][i] = 0.0
                    elif dist_cache[i][j] < 0.0:
#                        dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist5(sim_grips_valid[i], sim_grips_valid[j])
                        dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist6(sim_grips_valid[i], sim_grips_valid[j], axis_O)
                        dist_cache[i][j] = dist
                        dist_cache[j][i] = dist
                        if max_dist == None or dist > max_dist:
                            max_dist = dist

#            print "max pair-wise distance: %s"%(max_dist)
#            hist_count = 10
#            hist_delta = max_dist / hist_count
#            for v in np.linspace(0, max_dist-hist_delta, hist_count):
#                samples_count = 0
#                for i in range(0, len(sim_grips_valid)):
#                    for j in range(0, i):
#                        if dist_cache[i][j] >= v and dist_cache[i][j] <= v + hist_delta:
#                            samples_count += 1
#                print "hist: (%s ; %s): %s"%(v, v + hist_delta, samples_count)

            n_clusters = 5
            print "dist_cache shape:"
            print dist_cache.shape
            clustering = AgglomerativeClustering(linkage='average', affinity='precomputed', n_clusters=n_clusters, compute_full_tree=True)
            clustering.fit(dist_cache)

            print "clustering.n_leaves_: %s"%(clustering.n_leaves_)
            print "clustering.n_components_: %s"%(clustering.n_components_)
            index = 0
            for lab in clustering.labels_:
                print "%s:  %s"%(index, lab)
                index += 1

            index = 0
            for ch in clustering.children_:
                print "%s:  %s"%(index, ch)
                index += 1

#            for cl in range(0, n_clusters):
#                print "showing cluster %s"%(cl)
#                grasps = []
#                for i in range(0, len(clustering.labels_)):
#                    if clustering.labels_[i] == cl:
#                        grasp = self.openrave.getGrasp("object", sim_grips_valid_idx[i])
#                        grasps.append(grasp)
#                print "number of grasps: %s"%(len(grasps))
#                self.openrave.showFinalConfigs("object", grasps)
#            print "end"

#            exit(0)









#                        dist2, angles2, all_scores3, all_angles2, all_scores4, n1_s_list2, pos1_s_list2, n2_s_list2, pos2_s_list2 = grip.gripDist5(sim_grips[i], sim_grips[c_idx])
#                        if abs(dist-dist2) > 0.01:
#                            print "%s %s:  %s  %s"%(idx, i, dist, dist2)



#            for i in range(0, len(sim_grips)):
#                if sim_grips[i] == None:
#                    continue
#                print i
#                grasp = self.openrave.getGrasp("object", i)
#                self.openrave.getFinalConfig("object", grasp, show=True)

            idx = random_idx
            grasp = self.openrave.getGrasp("object", sim_grips_valid_idx[idx])
            self.openrave.getFinalConfig("object", grasp, show=True)

            for tries in range(0, 10):
                checked_idx.append(idx)
                next_idx = None
                max_dist = None
                for i in range(0, len(sim_grips_valid)):
                    if sim_grips_valid[i] == None:
                        continue
                    if i in checked_idx:
                        continue
                    min_dist = None
                    for c_idx in checked_idx:
#                        if dist_cache[c_idx][i] < 0.0:
#                            dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist5(sim_grips_valid[c_idx], sim_grips_valid[i])
#                            dist_cache[c_idx][i] = dist
#                            dist_cache[i][c_idx] = dist

#                            dist2, angles2, all_scores3, all_angles2, all_scores4, n1_s_list2, pos1_s_list2, n2_s_list2, pos2_s_list2 = grip.gripDist5(sim_grips_valid[i], sim_grips_valid[c_idx])
#                            if abs(dist-dist2) > 0.01:
#                                print "%s %s:  %s  %s"%(idx, i, dist, dist2)
#                        else:
                        dist = dist_cache[c_idx][i]

                        if min_dist == None or dist < min_dist:
                            min_dist = dist

                    if max_dist == None or min_dist > max_dist:
                        max_dist = min_dist
                        next_idx = i

                print "next grip: %s  dist: %s"%(next_idx, max_dist)

                grasp = self.openrave.getGrasp("object", sim_grips_valid_idx[next_idx])
                self.openrave.getFinalConfig("object", grasp, show=True)
#                self.openrave.showGrasp("object", grasp)
                idx = next_idx

            exit(0)





















        ################
        # the main loop
        ################
        T_W_T_orig = copy.deepcopy(velma.T_W_T)
        T_W_T_new = copy.deepcopy(velma.T_W_T)
        base_qar = copy.deepcopy(velma.qar)
        T_B_O_prev = None
        checked_grasps_idx = []
        grips_db = []
        while True:

            self.allowUpdateObjects()

            self.switchToJoint(velma)

            velma.updateTransformations()
            velma.updateAndMoveToolOnly(T_W_T_orig, 0.5)
            if velma.checkStopCondition(0.5):
                exit(0)
#            raw_input("Press Enter to set impedance to bigger value...")
            print "setting stiffness to bigger value"
            velma.moveImpedance(k_pregrasp, 0.5)
            if velma.checkStopCondition(0.5):
                exit(0)

#            rospy.sleep(1.0)
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            print "pose of the object is from %s seconds ago"%(dur.to_sec())

            # move to base pose
            dist = 0.0
            for q_idx in range(0,7):
                dist += (velma.qar[q_idx] - base_qar[q_idx])*(velma.qar[q_idx] - base_qar[q_idx])
            if math.sqrt(dist) > 10.0/180.0*math.pi:
                print "moving to base pose..."
                traj = self.openrave.planMoveForRightArm(None, base_qar)
                if traj == None:
                    print "FATAL ERROR: colud not plan trajectory to base pose"
                    return

                duration = math.fsum(traj[3])

                raw_input("Press Enter to visualize the trajectory...")
                if velma.checkStopCondition():
                    exit(0)
                self.openrave.showTrajectory(duration * time_mult * 0.3, qar_list=traj[4])

                self.switchToJoint(velma)

                print "trajectory len: %s"%(len(traj[0]))
                raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                if velma.checkStopCondition():
                    exit(0)
                velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                if velma.checkStopCondition(duration * time_mult + 1.0):
                    exit(0)

            velma.calibrateTactileSensors()

            rospy.sleep(2.0)
            self.disallowUpdateObjects()
            rospy.sleep(0.2)

            print "generating possible grasps..."
            # check if we need to generate new set of possible grasps
            generate_new_grasps = True
            T_B_O = self.openrave.getPose("object")
            if T_B_O_prev != None:
                T_B_O_diff = PyKDL.diff(T_B_O_prev, T_B_O)
                if T_B_O_diff.vel.Norm() < 0.02 and T_B_O_diff.rot.Norm() < 5.0/180.0*math.pi:
                    generate_new_grasps = False
            T_B_O_prev = T_B_O
            if generate_new_grasps:
                # get the possible grasps for the current scene
                indices = self.openrave.generateGrasps("object", checkcollision=True, checkgrasper=True)

            if len(indices) == 0:
                print "FATAL ERROR: could not generate any grasps for current configuration"
                exit(0)

            print "number of possible grasps for current pose: %s"%(len(indices))

            # show all possible grasps
#            for idx in indices:
#                self.openrave.showGrasp("object", self.openrave.getGrasp("object", idx))

            max_dist = -1000000.0
            min_score = None
            max_idx = None
            # TODO
            first_ok_idx = None
            indices_scored = []
            # iterate through all available grasps
            for idx in indices:
                if sim_grips[idx] == None:
                    continue
                # ignore grasps which failed due to planning error for pose close to the current pose
                close_to_failed = False
                for T_Br_O_failed in sim_grips[idx].planning_failure_poses:
                    diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_O_failed)
                    if diff.vel.Norm() < 0.02 and diff.rot.Norm() < 5.0/180.0*math.pi:
                        close_to_failed = True
                        break
                if close_to_failed:
                    continue

                # ignore grasps which failed due to visibility error for pose close to the current pose
                close_to_failed = False
                for T_Br_O_failed in sim_grips[idx].visibility_problem_poses:
                    diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_O_failed)
                    if diff.vel.Norm() < 0.02 and diff.rot.Norm() < 5.0/180.0*math.pi:
                        close_to_failed = True
                        break
                if close_to_failed:
                    continue

                # ignore the grasps with:
                # count_no_contact > 0
                # count_too_little_contacts > 0
                # count_moved_on_grip > 0
                # count_unstable > 0
                # count_stable > 0
                if sim_grips[idx].count_no_contact > 0 or sim_grips[idx].count_too_little_contacts > 0 or sim_grips[idx].count_moved_on_grip > 0 or sim_grips[idx].count_unstable > 0 or sim_grips[idx].count_stable > 0:
                    continue

                # save the index of the first possibly good grasp
                if first_ok_idx == None:
                    first_ok_idx = idx

                # search for grasp most distant from other grasps with:
                # count_no_contact > 0
                # count_too_little_contacts > 0
                # count_unstable > 0

                # iterate through all checked grasps and calculate the total score
                score = 0.0
                for idx_2 in range(0, self.openrave.getGraspsCount("object")):
                    if sim_grips[idx_2] == None:
                        continue
                    sc_mul = 1.0
                    if sim_grips[idx_2].count_no_contact == 0 and sim_grips[idx_2].count_too_little_contacts == 0 and sim_grips[idx_2].count_unstable == 0 and sim_grips[idx_2].count_stable == 0:
                        continue
                    dist, angles, all_scores, all_angles, all_scores2, n1_s_list, pos1_s_list, n2_s_list, pos2_s_list = grip.gripDist3(sim_grips[idx], sim_grips[idx_2])

                    penalty_no_contact = 4.0 * sim_grips[idx_2].count_no_contact * max(5.0-dist, 0.0)
                    penalty_too_little_contacts = 4.0 * sim_grips[idx_2].count_too_little_contacts * max(5.0-dist, 0.0)
                    penalty_unstable = sim_grips[idx_2].count_unstable * max(5.0-dist, 0.0)
                    reward_stable = sim_grips[idx_2].count_stable * max(5.0-dist, 0.0)

                    score += penalty_no_contact + penalty_too_little_contacts + penalty_unstable - reward_stable

                indices_scored.append( [score, idx] )
#                if min_score == None or min_score > score:
                    # check collisions
#                    if not self.openrave.checkGripperCollision("object", idx):
#                        min_score = score
#                        max_idx = idx
            print "chosen and scored candidate grasps"

            indices_scored_sorted = sorted(indices_scored, key=operator.itemgetter(0))
            for score_idx in indices_scored_sorted:
                score, idx = score_idx
                if self.openrave.checkGripperCollision("object", idx):
                    sim_grips[idx].setPlanningFailure(obj_grasp.T_Br_Co)
                else:
                    grasp = copy.deepcopy(self.openrave.getGrasp("object", idx))
                    T_B_Ed = self.openrave.getGraspTransform("object", grasp, collisionfree=True)
                    traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
                    if traj == None:
                        sim_grips[idx].setPlanningFailure(obj_grasp.T_Br_Co)
                    else:
                        final_config, contacts = self.openrave.getFinalConfig("object", grasp)
                        if final_config == None:
                            sim_grips[idx].setPlanningFailure(obj_grasp.T_Br_Co)
                        else:
                            min_score = score
                            max_idx = idx
                            break

            print "found grasp that has the best score: %s"%(min_score)

            if max_idx != None:
                grasp_idx = max_idx
            elif first_ok_idx != None:
                grasp_idx = first_ok_idx
            else:
                print "FATAL ERROR: max_idx == None and first_ok_idx == None"
                exit(0)

            current_sim_grip = sim_grips[grasp_idx]

            print "choosed grasp no. %s"%(grasp_idx)

            print "found grasp"

            grasp = self.openrave.getGrasp("object", grasp_idx)

            T_B_Ed = self.openrave.getGraspTransform("object", grasp, collisionfree=True)
            self.openrave.showGrasp("object", grasp)

            T_Br_O_init = obj_grasp.T_Br_Co
            traj = self.openrave.planMoveForRightArm(T_B_Ed, None)
            if traj == None:
                print "colud not plan trajectory"
                current_sim_grip.setPlanningFailure(obj_grasp.T_Br_Co)
                continue

            final_config, contacts_ret, normals = self.openrave.getFinalConfig("object", grasp)
            if final_config == None:
                print "colud not plan trajectory"
                current_sim_grip.setPlanningFailure(obj_grasp.T_Br_Co)
                continue

            print "final_config:"
            print final_config
            print "contacts (sim): %s"%(len(contacts_ret))
            print "standoff: %s"%(self.openrave.getGraspStandoff("object", grasp))

#            print "q_start: %s"%(traj[0][0])
#            print "q_end:   %s"%(traj[0][-1])

            duration = math.fsum(traj[3])

            raw_input("Press Enter to visualize the trajectory...")
            if velma.checkStopCondition():
                exit(0)
            self.openrave.showTrajectory(duration * time_mult * 0.3, qar_list=traj[4])

            self.switchToJoint(velma)

            print "trajectory len: %s"%(len(traj[0]))
            raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
            if velma.checkStopCondition(duration * time_mult + 1.0):
                exit(0)

            self.switchToCartesian(velma)

            # move to the desired position
            velma.updateTransformations()
            T_B_Wd = T_B_Ed * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            raw_input("Press Enter to close fingers for pre-grasp...")
            # close the fingers for pre-grasp
            ad = 5.0/180.0*math.pi
            velma.move_hand_client([final_config[0]-ad, final_config[1]-ad, final_config[2]-ad, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))

            print "setting stiffness to lower value"
            velma.moveImpedance(k_grasp, 3.0)
            if velma.checkStopCondition(3.0):
                break

            # close the fingers
            ad2 = 20.0/180.0*math.pi
            velma.move_hand_client([final_config[0]+ad2, final_config[1]+ad2, final_config[2]+ad2, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(2000.0, 2000.0, 2000.0, 2000.0))
            if velma.checkStopCondition(3.0):
                break

            if not simulation_only:
                # get contact points and forces for each finger
                velma.updateTransformations()
                contacts = [[],[],[]]
                forces = [[],[],[]]
                contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                fingers_in_contact = 0
                print "f1: %s   %s    %s"%((final_config[0]+ad2), velma.qhr[1], len(contacts[0]))
                print "f2: %s   %s    %s"%((final_config[1]+ad2), velma.qhr[2], len(contacts[1]))
                print "f3: %s   %s    %s"%((final_config[2]+ad2), velma.qhr[3], len(contacts[2]))
                if abs((final_config[0]+ad2) - velma.qhr[1]) > 1.0/180.0*math.pi or len(contacts[0]) > 0:
                    fingers_in_contact += 1
                    f1_contact = True
                if abs((final_config[1]+ad2) - velma.qhr[2]) > 1.0/180.0*math.pi or len(contacts[1]) > 0:
                    fingers_in_contact += 1
                    f2_contact = True
                if abs((final_config[3]+ad2) - velma.qhr[3]) > 1.0/180.0*math.pi or len(contacts[2]) > 0:
                    fingers_in_contact += 1
                    f3_contact = True
            else:
                fingers_in_contact = 3

            if fingers_in_contact == 0:
                current_sim_grip.setNoContact()
            elif fingers_in_contact < 3:
                current_sim_grip.setTooLittleContacts()

            if fingers_in_contact < 3:
                print "only %s fingers have contact"%(fingers_in_contact)
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                continue

            self.allowUpdateObjects()

            # start with very low stiffness
#            print "setting stiffness to very low value"
#            velma.moveImpedance(velma.k_error, 0.5)
#            if velma.checkStopCondition(0.5):
#                exit(0)

            print "checking the object pose after grip..."
            rospy.sleep(1.0)

            # grab the body
            self.openrave.grab("object")

            pose_tolerance = [0.02, 5.0/180.0*math.pi]
            # check the object pose before lift-up
#            dur = rospy.Time.now() - obj_grasp.pose_update_time
#            if abs(dur.to_sec()) < 1.0:
#                print "fresh pose available: %s"%(dur.to_sec())
#                fresh_pose = True
#                T_Br_Co_sim = self.openrave.getPose("object")
#                pose_diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_Co_sim)
#                if pose_diff.vel.Norm() > pose_tolerance[0] or pose_diff.rot.Norm() > pose_tolerance[1]:
#                    print "object pose is different after the grip - diff: %s > %s     %s > %s deg. but it is okay..."%(pose_diff.vel.Norm(), pose_tolerance[0], pose_diff.rot.Norm()/math.pi*180.0, pose_tolerance[1]/math.pi*180.0)
#            else:
#                print "fresh pose not available: %s"%(dur.to_sec())
#                fresh_pose = False

            self.switchToJoint(velma)

            # set the new orientation of tool
            velma.updateTransformations()

            # get the current tool transformation
            T_B_T = velma.T_B_W * T_W_T_orig

            T_B_T_new = velmautils.alignRotationToVerticalAxis(T_B_T)
            R_B_T_new = PyKDL.Frame(copy.deepcopy(T_B_T_new.M))
            T_W_T_new = velma.T_B_W.Inverse() * T_B_T_new
            velma.updateAndMoveToolOnly(T_W_T_new, 1.0)

            if abs((R_B_T_new * PyKDL.Vector(1,0,0)).z()) > 0.9:
                k_lift = Wrench(Vector3(800.0, 5.0, 5.0), Vector3(2.0, 200.0, 200.0))
                k_move = Wrench(Vector3(800.0, 5.0, 5.0), Vector3(200.0, 200.0, 200.0))
            elif abs((R_B_T_new * PyKDL.Vector(0,1,0)).z()) > 0.9:
                k_lift = Wrench(Vector3(5.0, 800.0, 5.0), Vector3(200.0, 2.0, 200.0))
                k_move = Wrench(Vector3(5.0, 800.0, 5.0), Vector3(200.0, 200.0, 200.0))
            else:
                k_lift = Wrench(Vector3(5.0, 5.0, 800.0), Vector3(200.0, 200.0, 2.0))
                k_move = Wrench(Vector3(5.0, 5.0, 800.0), Vector3(200.0, 200.0, 200.0))
            print "new impedance:"
            print k_lift
            raw_input("Press Enter to set impedance to bigger value...")
            print "setting stiffness to bigger value"
            velma.moveImpedance(k_lift, 3.0)
            if velma.checkStopCondition(3.0):
                exit(0)

            # lift the object up
            velma.updateTransformations()

            T_B_Ebeforelift = velma.T_B_W * velma.T_W_E
            T_B_Wd = PyKDL.Frame(PyKDL.Vector(0,0,0.08)) * velma.T_B_W
            # save the initial position after lift up
            T_B_Eafterlift = T_B_Wd * velma.T_W_E

            self.switchToCartesian(velma)

            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            qar_after_lift = copy.deepcopy(velma.qar)

            # get contact points and forces for each finger
            if not simulation_only:
                velma.updateTransformations()
                contacts = [[],[],[]]
                forces = [[],[],[]]
                contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                fingers_in_contact = 0
                if len(contacts[0]) > 0:
                    fingers_in_contact += 1
                if len(contacts[1]) > 0:
                    fingers_in_contact += 1
                if len(contacts[2]) > 0:
                    fingers_in_contact += 1
            else:
                fingers_in_contact = 3

            if fingers_in_contact < 2:
                print "only %s fingers have contact"%(fingers_in_contact)
                current_sim_grip.setUnstable()
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                T_B_Wd = T_B_Ebeforelift * velma.T_E_W
                duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
                raw_input("Press Enter to move the robot in " + str(duration) + " s...")
                if velma.checkStopCondition():
                    exit(0)
                velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if velma.checkStopCondition(duration):
                    break
                continue

            self.allowUpdateObjects()
            print "checking the object pose after the lift-up..."
            stiffness_changed = False
            for angle_deg in (5.0, -5.0, 10.0, -10.0, 15.0, -15.0):
                rospy.sleep(2.0)
                # check the object pose after lift-up
                dur = rospy.Time.now() - obj_grasp.pose_update_time
                if abs(dur.to_sec()) < 2.0:
                    break

                if not stiffness_changed:
                    velma.updateTransformations()
                    T_B_Wd = velma.T_B_W
                    duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
                    raw_input("Press Enter to move the robot to the currrent pose in " + str(duration) + " s...")
                    velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                    if velma.checkStopCondition(duration):
                        break

                    stiffness_changed = True
                    print k_move
                    raw_input("Press Enter to set impedance to bigger value...")
                    print "setting stiffness to bigger value"
                    velma.moveImpedance(k_move, 3.0)
                    if velma.checkStopCondition(3.0):
                        exit(0)

                R_B = PyKDL.Frame(PyKDL.Rotation.RotZ(angle_deg/180.0*math.pi))

                T_B_E = velma.T_B_W * velma.T_W_E
                T_B_Ed = PyKDL.Frame(copy.deepcopy((R_B * T_B_E).M), copy.deepcopy(T_B_E.p))
                T_B_Wd = T_B_Ed * velma.T_E_W
                duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
                raw_input("Press Enter to rotate the gripper in " + str(duration) + " s...")
                if velma.checkStopCondition():
                    exit(0)
                velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if velma.checkStopCondition(duration):
                    break

            # check the object pose after lift-up
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            if abs(dur.to_sec()) < 2.0:
                print "fresh pose available: %s"%(dur.to_sec())
                fresh_pose = True
                T_Br_Co_sim = self.openrave.getPose("object")
                pose_diff = PyKDL.diff(obj_grasp.T_Br_Co, T_Br_Co_sim)
                if pose_diff.vel.Norm() > pose_tolerance[0] or pose_diff.rot.Norm() > pose_tolerance[1]:
                    print "object pose is different after the lift-up - diff: %s > %s     %s > %s deg."%(pose_diff.vel.Norm(), pose_tolerance[0], pose_diff.rot.Norm()/math.pi*180.0, pose_tolerance[1]/math.pi*180.0)
                    print "adding experience for determination of COM"
                    # calculate the contacts in object frame
                    contacts_O = []
                    for c in list(contacts[0]) + list(contacts[1]) + list(contacts[2]):
                        contacts_O.append(obj_grasp.T_Br_Co.Inverse() * c)

                    m_id = 0
                    T_B_O = T_Br_O_init
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O)
                    m_id = self.pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O)

                    T_B_O_2 = obj_grasp.T_Br_Co
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O_2)
                    m_id = self.pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O_2)

                    m_id = obj_grasp.updateCom(T_Br_O_init ,obj_grasp.T_Br_Co, contacts_O, m_id=m_id, pub_marker=self.pub_marker)
                    # get max value
                    max_com = None
                    for com_value in obj_grasp.com_weights:
                        if max_com == None or max_com < com_value:
                            max_com = com_value
                    good_com_count = 0
                    for idx in range(0, len(obj_grasp.com_pt)):
                        if obj_grasp.com_weights[idx] == max_com:
                            good_com_count += 1
                            m_id = pub_marker.publishSinglePointMarker(obj_grasp.com_pt[idx], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=obj_grasp.T_Br_Co)
                        else:
                            m_id = pub_marker.publishSinglePointMarker(obj_grasp.com_pt[idx], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=obj_grasp.T_Br_Co)
                        if idx % 10 == 0:
                            rospy.sleep(0.01)
                    print "COM estimation: %s  com: %s"%(float(good_com_count)/float(len(obj_grasp.com_pt)), obj_grasp.com)

                    current_sim_grip.setUnstable()
                    self.openrave.release("object")
                    velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                    if velma.checkStopCondition(3.0):
                        break
                    T_B_Wd = T_B_Ebeforelift * velma.T_E_W
                    duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
                    raw_input("Press Enter to move the robot in " + str(duration) + " s...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                    if velma.checkStopCondition(duration):
                        break
                    continue
            else:
                print "fresh pose not available: %s"%(dur.to_sec())
                fresh_pose = False
                current_sim_grip.setVisibilityProblem(T_Br_O_init)
                self.openrave.release("object")
                velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
                if velma.checkStopCondition(3.0):
                    break
                T_B_Wd = T_B_Ebeforelift * velma.T_E_W
                duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
                raw_input("Press Enter to move the robot in " + str(duration) + " s...")
                if velma.checkStopCondition():
                    exit(0)
                velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if velma.checkStopCondition(duration):
                    break
                continue

            print "success"
            current_sim_grip.setStable()
            self.openrave.release("object")
            velma.move_hand_client([0, 0, 0, final_config[3]], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
            if velma.checkStopCondition(3.0):
                break
            T_B_Wd = T_B_Ebeforelift * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.02, max_v_r=0.04)
            raw_input("Press Enter to move the robot in " + str(duration) + " s...")
            if velma.checkStopCondition():
                exit(0)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break
            rospy.sleep(1.0)

            continue












#            raw_input("Press Enter to exit...")
#            exit(0)


            print "checking the orientation of the object..."
            # TODO
            stable_grasp = True

            # basic orientations of the gripper, we can rotate them in global z axis and move them around
            main_R_Br_E2 = [
            PyKDL.Frame(),                                           # gripper points up
            PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi)),    # gripper points down
            PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)),     # gripper points right (E.y points up)
            PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)),    # gripper points left (E.y points down)
            PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi)),     # gripper points to front (E.x points down)
            PyKDL.Frame(PyKDL.Rotation.RotY(-90.0/180.0*math.pi)),    # gripper points to back (E.x points up)
            ]
            main_R_Br_E = [
            [PyKDL.Frame(),1],                                           # gripper points up
            [PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi)),0],    # gripper points down
            [PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi)),3],     # gripper points right (E.y points up)
            [PyKDL.Frame(PyKDL.Rotation.RotX(-90.0/180.0*math.pi)),2],    # gripper points left (E.y points down)
            [PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi)),5],     # gripper points to front (E.x points down)
            [PyKDL.Frame(PyKDL.Rotation.RotY(-90.0/180.0*math.pi)),4],    # gripper points to back (E.x points up)
            ]

            vis = self.openrave.getVisibility("object", velma.T_B_C, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
            print "vis: %s"%(vis)

            self.openrave.printCollisions()
            wrist_col = velmautils.WristCollisionAvoidance("right", None, 5.0/180.0*math.pi)
            velma.updateTransformations()
            list_T_B_Ed = []
            singularity_angle = 20.0/180.0*math.pi
            for basic_orientation_idx in range(0, len(main_R_Br_E)):
                if not stable_grasp:
                    break
                R_Br_E = main_R_Br_E[basic_orientation_idx][0]
                print "generate reasonable destinations for one basic orientation no. %s"%(basic_orientation_idx)
                # generate reasonable destinations for one basic orientation
                list_T_B_Ed.append([])
                angle_steps = 10
                found_solution = False
                for r in np.linspace(0.0, 0.2, 5):
                    density = 10.0    # per meter
#                    L = 2.0 * math.pi * r
                    if r == 0.0 or 1.0/(r*density) > math.pi:
                        v_sphere = velmautils.generateNormalsSphere(math.pi)
                    else:
                        v_sphere = velmautils.generateNormalsSphere(1.0/(r*density))
                    print "normals: %s"%(len(v_sphere))
                    for angle_deg in np.linspace(0.0, 360.0*(float(angle_steps-1)/angle_steps), angle_steps):
                        # calculate orientation (rotate along global z axis)
                        R_Br_Ed = PyKDL.Frame(PyKDL.Rotation.RotZ(angle_deg/180.0*math.pi)) * R_Br_E
                        # set the position
                        T_E_G = PyKDL.Frame(PyKDL.Vector(0,0,0.2))
                        T_G_E = T_E_G.Inverse()
                        # iterate gripper position in camera frame:

                        A = 4.0 * math.pi * r * r
                        density = 100.0    # per square meter
                        steps = int(A * density) + 1
                        for i in range(0, steps):
                            v = r * v_sphere[random.randint(0, len(v_sphere)-1)]#PyKDL.Frame( PyKDL.Rotation.RotX(random.uniform(-math.pi, math.pi)) * PyKDL.Rotation.RotY(random.uniform(-math.pi, math.pi))) * PyKDL.Vector(0,0,r)
                            pt_G_in_B = velma.T_B_C * PyKDL.Vector(0, 0, 0.5) + v
                            T_B_Gd = PyKDL.Frame(copy.deepcopy(R_Br_Ed.M), pt_G_in_B)
                            T_B_Ed = T_B_Gd * T_G_E
                            q_out = self.openrave.findIkSolution(T_B_Ed)
                            if q_out != None:
                                if len(wrist_col.getQ5Q6SpaceSectors(q_out[5],q_out[6])) > 0:
                                    vis = self.openrave.getVisibility("object", velma.T_B_C, qar=q_out, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
#                                    print "vis: %s   i: %s   angle: %s    r: %s    o: %s"%(vis, i, angle_deg, r, basic_orientation_idx)
                                    if vis > 0.8:
                                        found_solution = True
                                        list_T_B_Ed[-1].append( [T_B_Ed, q_out] )
                                        print "i: %s"%(i)
                                        break
#                            else:
#                                print "q_out == None"
                        if found_solution:
                            print "angle_deg: %s"%(angle_deg)
                            break
                    if found_solution:
                        print "r: %s"%(r)
                        break
                print "generated %s poses"%(len(list_T_B_Ed[-1]))

            checked_orientations = []

            raw_input("Press Enter to enable joint impedance...")
            if velma.checkStopCondition():
                exit(0)
            velma.switchToJoint()

            while len(checked_orientations) < len(main_R_Br_E):
                if not stable_grasp:
                    break

                velma.updateTransformations()

                # iterate through all poses and get the closest with reasonable visibility
                print "looking for reachable and visible poses..."
                min_cost = 1000000.0
                best_basic_orientation_idx = -1
                best_pose_idx = -1
                best_q_dest = None
                best_traj = None
                for basic_orientation_idx in range(0, len(main_R_Br_E)):
                    if basic_orientation_idx in checked_orientations:
                        continue
                    for pose_idx in range(0, len(list_T_B_Ed[basic_orientation_idx])):
                            T_B_Ed = list_T_B_Ed[basic_orientation_idx][pose_idx][0]
                            q_dest = list_T_B_Ed[basic_orientation_idx][pose_idx][1]

                            cost = 0.0
                            for q_idx in range(0, 7):
                                cost += (velma.qar[q_idx]-q_dest[q_idx])*(velma.qar[q_idx]-q_dest[q_idx])
                            if velma.qar[3]*q_dest[3] < 0:
                                cost += 100.0
                            if cost < min_cost:
                                min_cost = cost
                                best_basic_orientation_idx = basic_orientation_idx
                                best_pose_idx = pose_idx
                                best_q_dest = q_dest
                                best_traj = traj

                print "best_basic_orientation_idx: %s best_pose_idx: %s   min_cost: %s"%(best_basic_orientation_idx, best_pose_idx, min_cost)

                if best_pose_idx < 0:
                    print "could not find next pose"
                    break

                T_B_Ed = list_T_B_Ed[best_basic_orientation_idx][best_pose_idx][0]
                q_dest = list_T_B_Ed[best_basic_orientation_idx][best_pose_idx][1]
                traj = self.openrave.planMoveForRightArm(None, q_dest)
                if traj != None:
                    self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])
                    duration = math.fsum(traj[3])
                    raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                    if velma.checkStopCondition(duration * time_mult + 1.0):
                        exit(0)
                else:
                    print "colud not plan trajectory"
                    rospy.sleep(4.0)
                    exit(0)

                vis = self.openrave.getVisibility("object", velma.T_B_C, pub_marker=None, fov_x=camera_fov_x, fov_y=camera_fov_y, min_dist=0.2)
                print "reached the desired pose. Visibility: %s"%(vis)

                checked_orientations.append(best_basic_orientation_idx)
                if best_basic_orientation_idx == 0:
                    checked_orientations.append(1)
                if best_basic_orientation_idx == 1:
                    checked_orientations.append(0)
                if best_basic_orientation_idx == 2:
                    checked_orientations.append(3)
                if best_basic_orientation_idx == 3:
                    checked_orientations.append(2)
                if best_basic_orientation_idx == 4:
                    checked_orientations.append(5)
                if best_basic_orientation_idx == 5:
                    checked_orientations.append(4)
                print "checked_orientations: %s"%(checked_orientations)

                print "checking the orientation of the object..."
                # TODO
                stable_grasp = True

            if stable_grasp:
                print "moving to initial pose"
                traj = self.openrave.planMoveForRightArm(None, qar_after_lift)
                if traj != None:
                    self.openrave.showTrajectory(duration * time_mult * 0.5, qar_list=traj[4])
                    duration = math.fsum(traj[3])
                    raw_input("Press Enter to execute the trajectory on real robot in " + str(duration * time_mult) + "s ...")
                    if velma.checkStopCondition():
                        exit(0)
                    velma.moveWristTrajJoint(traj, time_mult, Wrench(Vector3(20,20,20), Vector3(4,4,4)))
                    if velma.checkStopCondition(duration * time_mult + 1.0):
                        exit(0)
                else:
                    print "colud not plan trajectory"
                    rospy.sleep(4.0)
                    exit(0)

                raw_input("Press Enter to enable cartesian impedance...")
                if velma.checkStopCondition():
                    exit(0)
                velma.switchToCart()

                T_B_Wd = T_B_Ebeforelift * velma.T_E_W
                duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
                velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
                if velma.checkStopCondition(duration):
                    break
            else:
                # TODO: check the position of the object and decide what to do
                pass

            velma.move_hand_client([0, 0, 0, 0], v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))

            print "releasing the body"
            # release the body
            if simulation_only: 
                self.openrave.release("object")
            else:
                pass

            self.allowUpdateObjects()

            raw_input("Press Enter to enable cartesian impedance...")
            if velma.checkStopCondition():
                exit(0)
            velma.switchToCart()

            print "moving the gripper up"
            T_B_Wd = T_B_Eafterlift * velma.T_E_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break


#            rospy.sleep(2.0)

#            raw_input("Press Enter to exit...")
#            exit(0)
            continue
















            raw_input("Press Enter to close fingers for grasp...")

            # close the fingers for grasp
            velma.move_hand_client((120.0/180.0*math.pi,120.0/180.0*math.pi,120.0/180.0*math.pi,0), v=(1.2, 1.2, 1.2, 1.2), t=(1500.0, 1500.0, 1500.0, 1500.0))
            m_id = 0
            if True:
                if velma.checkStopCondition(3.0):
                    break
            else:
                time_end = rospy.Time.now() + rospy.Duration(3.0)
                all_contacts = []
                all_forces = []
                while rospy.Time.now() < time_end:
                    contacts = [[],[],[]]
                    forces = [[],[],[]]
                    contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
                    contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
                    contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)
                    if len(contacts) > 0:
                        all_contacts.append(contacts)
                        all_forces.append(forces)
                    rospy.sleep(0.01)
                    if velma.checkStopCondition():
                        break
                for c in all_contacts:
                    m_id = self.pub_marker.publishMultiPointsMarker(c[0], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                    rospy.sleep(0.01)
                    m_id = self.pub_marker.publishMultiPointsMarker(c[1], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                    rospy.sleep(0.01)
                    m_id = self.pub_marker.publishMultiPointsMarker(c[2], m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
                    rospy.sleep(0.01)

            # get contact points and forces for each finger
            velma.updateTransformations()
            contacts = [[],[],[]]
            forces = [[],[],[]]
            contacts[0], forces[0] = velma.getContactPoints(100, f1=True, f2=False, f3=False, palm=False)
            contacts[1], forces[1] = velma.getContactPoints(100, f1=False, f2=True, f3=False, palm=False)
            contacts[2], forces[2] = velma.getContactPoints(100, f1=False, f2=False, f3=True, palm=False)

            m_id = self.pub_marker.publishMultiPointsMarker(contacts[0], m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            rospy.sleep(0.01)
            m_id = self.pub_marker.publishMultiPointsMarker(contacts[1], m_id, r=0, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            rospy.sleep(0.01)
            m_id = self.pub_marker.publishMultiPointsMarker(contacts[2], m_id, r=0, g=0, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
            rospy.sleep(0.01)

            # calculate force-weighted center of contacts for each finger
            centers = []
            number_of_fingers_in_contact = 0
            forces_sum = []
            for finger in range(0, len(contacts)):
                center = PyKDL.Vector()
                force_sum = 0.0
                for i in range(0, len(contacts[finger])):
                    center += contacts[finger][i] * forces[finger][i]
                    force_sum += forces[finger][i]
                forces_sum.append(force_sum)
                if force_sum > 0.0:
                    center *= (1.0/force_sum)
                    centers.append(center)
                    number_of_fingers_in_contact += 1
                else:
                    centers.append(None)

            print "fingers in contact: %s"%(number_of_fingers_in_contact)
            if number_of_fingers_in_contact < 2:
                print "could not grasp the object with more than 1 finger"
                break

            gr = grip.Grip(obj_grasp)

            for c in centers:
                if c != None:
                    m_id = self.pub_marker.publishSinglePointMarker(c, m_id, r=1, g=1, b=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))

            T_Br_O = obj_grasp.T_Br_Co

            T_E_Co_before = velma.T_E_W * velma.T_B_W.Inverse() * T_Br_O

            T_O_Br = T_Br_O.Inverse()
            vertices, indices = self.openrave.getMesh("object")
            finger = 0
            T_E_Fi3 = [velma.T_E_F13, velma.T_E_F23, velma.T_E_F33]
            T_Fi3_E = [velma.T_F13_E, velma.T_F23_E, velma.T_F33_E]
            actual_angles = [velma.q_rf[1], velma.q_rf[4], velma.q_rf[6]]
            for c in centers:
                if c != None:
                    c_Fi3 = T_Fi3_E[finger] * velma.T_E_W * velma.T_B_W.Inverse() * c
                    pt_list = []
                    for angle in np.linspace(actual_angles[finger]-10.0/180.0*math.pi, actual_angles[finger]+10.0/180.0*math.pi, 20):
                        T_E_F = velma.get_T_E_Fd(finger, angle, 0)
                        cn_B = velma.T_B_W * velma.T_W_E * T_E_F * c_Fi3
                        cn_O = T_O_Br * cn_B
                        pt_list.append(cn_O)
                    m_id = self.pub_marker.publishMultiPointsMarker(pt_list, m_id, r=1, g=1, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004), T=T_Br_O)
                    points = velmautils.sampleMesh(vertices, indices, 0.002, pt_list, 0.01)
                    print len(points)
                    m_id = self.pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.004, 0.004, 0.004), T=T_Br_O)
                    rospy.sleep(1.0)
                    # get the contact surface normal
                    fr = velmautils.estPlane(points)
                    # set the proper direction of the contact surface normal (fr.z axis)
                    if PyKDL.dot( T_Br_O * fr * PyKDL.Vector(0,0,1), velma.T_B_W * velma.T_W_E * T_E_Fi3[finger] * PyKDL.Vector(1,-1,0) ) > 0:
                        fr = fr * PyKDL.Frame(PyKDL.Rotation.RotX(180.0/180.0*math.pi))
                    # add the contact to the grip description
                    gr.addContact(fr)
                    m_id = self.pub_marker.publishFrameMarker(T_Br_O*fr, m_id)
                    rospy.sleep(1.0)
                finger += 1

            self.allowUpdateObjects()

            # lift the object up
            velma.updateTransformations()

            T_B_Wd = PyKDL.Frame(PyKDL.Vector(0,0,0.05)) * velma.T_B_W
            duration = velma.getMovementTime(T_B_Wd, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist2(T_B_Wd*velma.T_W_T)
            raw_input("Press Enter to lift the object up in " + str(duration) + " s...")
            if velma.checkStopCondition():
                break
            velma.moveWrist(T_B_Wd, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

            if velma.checkStopCondition(2.0):
                break

            contacts, forces = velma.getContactPoints(200, f1=True, f2=True, f3=True, palm=False)
            if len(contacts) > 0:
                print "Still holding the object. Contacts: %s"%(len(contacts))
                holding = True
            else:
                holding = False

            # try to get fresh object pose
            dur = rospy.Time.now() - obj_grasp.pose_update_time
            if dur.to_sec() < 1.0:
                fresh_pose = True
            else:
                fresh_pose = False

            velma.updateTransformations()
            if fresh_pose:
                print "we can still see the object!"
                T_E_Co_after = velma.T_E_W * velma.T_B_W.Inverse() * obj_grasp.T_Br_Co
                T_E_Co_diff = PyKDL.diff(T_E_Co_before, T_E_Co_after)
                print "T_E_Co_diff: %s"%(T_E_Co_diff)
            else:
                print "we can't see the object!"

            gr.success()

            grips_db.append( gr )

            raw_input("Press Enter to open fingers...")

            velma.move_hand_client((0,0,0,0), v=(1.2, 1.2, 1.2, 1.2), t=(3000.0, 3000.0, 3000.0, 3000.0))
            if velma.checkStopCondition(3.0):
                break

            duration = velma.getMovementTime(velma_init_T_B_W, max_v_l=0.1, max_v_r=0.2)
            velma.moveWrist2(velma_init_T_B_W*velma.T_W_T)
            raw_input("Press Enter to move back to initial position in " + str(duration) + " s...")
            if velma.checkStopCondition():
                break
            velma.moveWrist(velma_init_T_B_W, duration, Wrench(Vector3(20,20,20), Vector3(4,4,4)), abort_on_q5_singularity=True, abort_on_q5_q6_self_collision=True)
            if velma.checkStopCondition(duration):
                break

        # grasping loop end

        for g in grips_db:
            g.serializePrint()

        print "setting stiffness to very low value"
        velma.moveImpedance(velma.k_error, 0.5)
        if velma.checkStopCondition(0.5):
            exit(0)

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

        exit(0)

if __name__ == '__main__':

    rospy.init_node('grasp_leanring')

    global br
    pub_marker = velmautils.MarkerPublisher()
    task = GraspingTask(pub_marker)
    rospy.sleep(1)
    br = tf.TransformBroadcaster()

    task.spin()


