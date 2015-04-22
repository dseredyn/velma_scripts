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
roslib.load_manifest('barrett_hand_controller')

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
from threading import RLock

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy
import thread
from openravepy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import velmautils
import operator
import random

class OpenraveInstance:

    def __init__(self):#, T_World_Br):
        self.joint_dof_idx_map = None

    def startOpenrave(self, filename_environment):
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args()
        self.env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
        self.env.Load(filename_environment)
        self.robot_rave = self.env.GetRobot("velma")
        base_link = self.robot_rave.GetLink("torso_base")
        self.T_World_Br = self.OpenraveToKDL(base_link.GetTransform())
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot_rave,iktype=IkParameterizationType.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

        self.manipulator_dof_indices_map = {}
        self.lower_lim = {}
        self.upper_lim = {}
        manipulators = self.robot_rave.GetManipulators()
        for man in manipulators:
            man_name = man.GetName()
            self.manipulator_dof_indices_map[man_name] = man.GetArmIndices()
            self.lower_lim[man_name], self.upper_lim[man_name] = self.robot_rave.GetDOFLimits(self.manipulator_dof_indices_map[man_name])

        self.minimumgoalpaths = 1
        plannername = "BiRRT"
        self.basemanip = interfaces.BaseManipulation(self.robot_rave,plannername=plannername)
        self.basemanip.prob.SendCommand('SetMinimumGoalPaths %d'%self.minimumgoalpaths)

    def setCamera(self, cam_pos, target_pos):
            cam_z = target_pos - cam_pos
            focalDistance = cam_z.Norm()
            cam_y = PyKDL.Vector(0,0,-1)
            cam_x = cam_y * cam_z
            cam_y = cam_z * cam_x
            cam_x.Normalize()
            cam_y.Normalize()
            cam_z.Normalize()
            cam_T = PyKDL.Frame(PyKDL.Rotation(cam_x,cam_y,cam_z), cam_pos)
            
            self.env.GetViewer().SetCamera(self.KDLToOpenrave(cam_T), focalDistance)

    def updateRobotConfigurationRos(self, js_pos):

        dof_values = self.robot_rave.GetDOFValues()
        for dof_idx in range(self.robot_rave.GetDOF()):
            joint = self.robot_rave.GetJointFromDOFIndex(dof_idx)
            joint_name = joint.GetName()
            if joint_name in js_pos:
                dof_values[dof_idx] = js_pos[joint_name]

        self.robot_rave.SetDOFValues(dof_values)

    def getRobotConfigurationRos(self):
        js_pos = {}
        joints = self.robot_rave.GetJoints() + self.robot_rave.GetPassiveJoints()
        for j in joints:
            if j.IsStatic():
                continue
            js_pos[j.GetName()] = j.GetValue(0)

        return js_pos

    def updateRobotConfiguration(self, qt=None, qal=None, qhl=None, qar=None, qhr=None):
        dof_values = self.robot_rave.GetDOFValues()
        if qt == None:
            qt = dof_values[0:2]
        if qal == None:
            qal = dof_values[2:9]
        if qhl == None:
            qhl = dof_values[9:13]
        if qar == None:
            qar = dof_values[13:20]
        if qhr == None:
            qhr = dof_values[20:24]
        dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
        self.robot_rave.SetDOFValues(dof_values)

    def addRobotInterface(self, robot):
        self.robot = robot

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

    def addBox(self, name, x_size, y_size, z_size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([[0,0,0,0.5*x_size,0.5*y_size,0.5*z_size]]),True)
        self.env.Add(body,True)

    def addCBeam(self, name, w, h, l, t):
#          h
#       _______
#    w  |
#       |______
#        
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromBoxes(numpy.array([
        [0.0, w/2.0 - t/2.0, 0.0, 0.5 * h, 0.5 * t, 0.5 * l],
        [-h/2.0 + t/2.0, 0.0, 0.0, 0.5 * t, 0.5 * (w - 2.0 * t), 0.5 * l],
        [0.0, -(w/2.0 - t/2.0), 0.0, 0.5 * h, 0.5 * t, 0.5 * l],
        ]),True)
        self.env.Add(body,True)

    def removeObject(self, name):
        with self.env:
            body = self.env.GetKinBody(name)
            self.env.Remove(body)

    def addSphere(self, name, size):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        body.InitFromSpheres(numpy.array([[0,0,0,0.5*size]]),True)
        self.env.Add(body,True)
        return body

    def addCamera(self, name, fov_x, fov_y, dist):
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        boxes = [
        [0,0,0,0.02,0.02,0.02],
        [dist*math.tan(fov_x/2.0),dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [-dist*math.tan(fov_x/2.0),dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [-dist*math.tan(fov_x/2.0),-dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        [dist*math.tan(fov_x/2.0),-dist*math.tan(fov_y/2.0),dist,0.01,0.01,0.01],
        ]
        body.InitFromBoxes(numpy.array(boxes),True)
        self.env.Add(body,True)
#        self.env.CheckCollision(self.robot_rave,body)

    def updatePose(self, name, T_Br_Bo):
        with self.env:
            body = self.env.GetKinBody(name)
            if body != None:
                body.SetTransform(self.KDLToOpenrave(self.T_World_Br*T_Br_Bo))
            else:
#                print "openrave: could not find body: %s"%(name)
                pass
            self.env.UpdatePublishedBodies()

    def getPose(self, name):
        body = self.env.GetKinBody(name)
        if body != None:
            return self.T_World_Br.Inverse() * self.OpenraveToKDL(body.GetTransform())
        return None

    def getLinkPose(self, name, qt=None, qar=None, qal=None, qhr=None, qhl=None):
        if qt != None or qar != None or qal != None or qhr != None or qhl != None:
            self.robot_rave_update_lock.acquire()
            with self.robot_rave.CreateRobotStateSaver():
                with self.env:
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    if qt == None:
                        qt = dof_values[0:2]
                    if qal == None:
                        qal = dof_values[2:9]
                    if qhl == None:
                        qhl = dof_values[9:13]
                    if qar == None:
                        qar = dof_values[13:20]
                    if qhr == None:
                        qhr = dof_values[20:24]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    self.env.UpdatePublishedBodies()
                    link = self.robot_rave.GetLink(name)
                    if link != None:
                        T_World_L = self.OpenraveToKDL(link.GetTransform())
                        self.robot_rave_update_lock.release()
                        return self.T_World_Br.Inverse() * T_World_L
                    else:
                        self.robot_rave_update_lock.release()
                        return None
        else:
            link = self.robot_rave.GetLink(name)
            if link != None:
                return self.T_World_Br.Inverse() * self.OpenraveToKDL(link.GetTransform())
        return None

    def changeColor(self, name, r, g, b, a):
        body = self.env.GetKinBody(name)
        if body != None:
            for link in body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetDiffuseColor([r,g,b])
                    geom.SetTransparency(a)

    def run(self, args, *args2):
        parser = OptionParser(description='Openrave Velma interface')
        OpenRAVEGlobalArguments.addOptions(parser)
        (options, leftargs) = parser.parse_args(args=args)
        OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,self.main,defaultviewer=True)

    def startNewThread(self):
        # start thread for jar tf publishing and for visualization
        thread.start_new_thread(self.run, (None,1))

    def prepareGraspingModule(self, target_name, force_load=False):
        if not hasattr(self, 'gmodel') or self.gmodel == None:
            self.gmodel = {}

        if not target_name in self.gmodel.keys():
            target = self.env.GetKinBody(target_name)
            self.gmodel[target_name] = databases.grasping.GraspingModel(self.robot_rave,target)
            self.gmodel[target_name].numthreads = 4
            if force_load or not self.gmodel[target_name].load():
              with self.env:
                print "removing all other objects"
                all_bodies = self.env.GetBodies()
                for body in all_bodies:
                    name = body.GetName()
                    if name != target_name and name != "velma":
                        self.env.Remove(body)

                print 'generating grasping model (one time computation)'
                self.gmodel[target_name].init(friction=0.6,avoidlinks=[])
                print 'grasping model initialised'
                print 'computing approach rays...'
                approachrays3 = self.gmodel[target_name].computeBoxApproachRays(delta=0.03,normalanglerange=0.0) #201, directiondelta=0.2)
#                print approachrays3.shape
#                print approachrays3[0]
#                exit(0)
#                approachrays3 = self.gmodel[target_name].computeBoxApproachRays(delta=0.03, normalanglerange=15.0/180.0*math.pi, directiondelta=14.0/180.0*math.pi)
#                approachrays3 = np.concatenate((approachrays, approachrays2), axis=0)
#                print approachrays3.shape
                print 'generating grasps...'
# possible arguments for generate:
# preshapes=None, standoffs=None, rolls=None, approachrays=None, graspingnoise=None, forceclosure=True, forceclosurethreshold=1.0000000000000001e-09, checkgraspfn=None, manipulatordirections=None, translationstepmult=None, finestep=None, friction=None, avoidlinks=None, plannername=None, boxdelta=None, spheredelta=None, normalanglerange=None
# http://openrave.org/docs/latest_stable/openravepy/databases.grasping/#openravepy.databases.grasping.GraspingModel.generatepcg
#                self.gmodel[target_name].generate(approachrays=approachrays3, forceclosure=False, standoffs=[0.025, 0.05, 0.075])
#                self.gmodel[target_name].generate(approachrays=approachrays3, friction=0.6, forceclosure=True, standoffs=[0.04, 0.06, 0.07])
                self.gmodel[target_name].generate(approachrays=approachrays3, friction=0.6, forceclosure=False, standoffs=np.array([0.04, 0.06])) #, 0.07
                self.gmodel[target_name].save()

                for body in all_bodies:
                    name = body.GetName()
                    if name != target_name and name != "velma":
                        self.env.Add(body)

    def getGraspsCount(self, target_name):
        self.prepareGraspingModule(target_name)
        return len(self.gmodel[target_name].grasps)

    def getGrasp(self, target_name, grasp_idx):
        self.prepareGraspingModule(target_name)
        return self.gmodel[target_name].grasps[grasp_idx]

    def computeValidGrasps(self, target_name, grasp_indices, checkcollision=True,checkik=True,checkgrasper=True,backupdist=0.0,returnnum=np.inf):
        """Returns the set of grasps that satisfy conditions like collision-free and reachable.

        :param returnnum: If set, will also return once that many number of grasps are found.
        :param backupdist: If > 0, then will move the hand along negative approach direction and check for validity.
        :param checkgrasper: If True, will execute the grasp and check if gripper only contacts target.
        :param checkik: If True will check that the grasp is reachable by the arm.
        :param checkcollision: If true will return only collision-free grasps. If checkik is also True, will return grasps that have collision-free arm solutions.
        """
        with self.gmodel[target_name].robot:
            validgrasps = []
            validindices = []
            self.gmodel[target_name].robot.SetActiveManipulator(self.gmodel[target_name].manip)
            report = CollisionReport()
            for i in grasp_indices:
                grasp = self.gmodel[target_name].grasps[i]
                self.gmodel[target_name].setPreshape(grasp)
                Tglobalgrasp = self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=True)
                if checkik:
                    if self.gmodel[target_name].manip.GetIkSolver().Supports(IkParameterization.Type.Transform6D):
                        if self.gmodel[target_name].manip.FindIKSolution(Tglobalgrasp,checkcollision) is None:
                            continue
                    elif self.gmodel[target_name].manip.GetIkSolver().Supports(IkParameterization.Type.TranslationDirection5D):
                        ikparam = IkParameterization(Ray(Tglobalgrasp[0:3,3],dot(Tglobalgrasp[0:3,0:3],self.gmodel[target_name].manip.GetDirection())),IkParameterization.Type.TranslationDirection5D)
                        solution = self.gmodel[target_name].manip.FindIKSolution(ikparam,checkcollision)
                        if solution is None:
                            continue
                        with RobotStateSaver(self.gmodel[target_name].robot):
                            self.gmodel[target_name].robot.SetDOFValues(solution, self.gmodel[target_name].manip.GetArmIndices())
                            Tglobalgrasp = self.gmodel[target_name].manip.GetEndEffectorTransform()
                            grasp = array(grasp)
                            grasp[self.gmodel[target_name].graspindices['grasptrans_nocol']] = Tglobalgrasp[0:3,0:4].flatten()
                    else:
                        return ValueError('manipulator iktype not correct')
                elif checkcollision:
                    if self.gmodel[target_name].manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
                if backupdist > 0:
                    Tnewgrasp = array(Tglobalgrasp)
                    Tnewgrasp[0:3,3] -= backupdist * self.gmodel[target_name].getGlobalApproachDir(grasp)
                    if checkik:
                        if self.gmodel[target_name].manip.FindIKSolution(Tnewgrasp,checkcollision) is None:
                            continue
                    elif checkcollision:
                        if self.gmodel[target_name].manip.CheckEndEffectorCollision(Tnewgrasp):
                            continue
                if checkcollision and checkgrasper:
                    try:
                        contacts2,finalconfig2,mindist2,volume2 = self.gmodel[target_name].runGraspFromTrans(grasp)
                    except planning_error, e:
                        continue
                validgrasps.append(grasp)
                validindices.append(i)
                if len(validgrasps) == returnnum:
                    return validgrasps,validindices
            return validgrasps,validindices

    def generateGrasps(self, target_name, show=False, checkcollision=True, checkik=True, checkgrasper=True, grasp_indices=None):
        self.prepareGraspingModule(target_name)

        if grasp_indices == None:
            grasp_indices = range(self.getGraspsCount(target_name))
#        validgrasps,validindices = self.gmodel[target_name].computeValidGrasps(checkcollision=checkcollision, checkik=checkik, checkgrasper=checkgrasper)
        validgrasps,validindices = self.computeValidGrasps(target_name, grasp_indices, checkcollision=checkcollision, checkik=checkik, checkgrasper=checkgrasper)
#        print "all valid grasps: %s"%(len(validgrasps))

#        print "done."
        return validindices

    def getGraspTransform(self, target_name, grasp, collisionfree=False):
        return self.T_World_Br.Inverse()*self.OpenraveToKDL( self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=collisionfree) )

    def showGrasp(self, target_name, grasp):
        self.robot_rave_update_lock.acquire()
        self.gmodel[target_name].showgrasp(grasp, collisionfree=False, useik=False)
        self.robot_rave_update_lock.release()

    def getGraspStandoff(self, target_name, grasp):
        return grasp[self.gmodel[target_name].graspindices.get('igraspstandoff')]

    def showTrajectory(self, time, qt_list=None, qar_list=None, qal_list=None, qhr_list=None, qhl_list=None):
        length = 0
        if qt_list != None:
            length = len(qt_list)
        elif qar_list != None:
            length = len(qar_list)
        elif qal_list != None:
            length = len(qal_list)
        elif qhr_list != None:
            length = len(qhr_list)
        elif qhl_list != None:
            length = len(qhl_list)
        if length < 1:
            return None
        time_d = time / length
        report = CollisionReport()
        first_collision = None
#        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.env:
                for i in range(0, length):
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    if qt_list == None:
                        qt = dof_values[0:2]
                    else:
                        qt = qt_list[i]
                    if qal_list == None:
                        qal = dof_values[2:9]
                    else:
                        qal = qal_list[i]
                    if qhl_list == None:
                        qhl = dof_values[9:13]
                    else:
                        qhl = qhl_list[i]
                    if qar_list == None:
                        qar = dof_values[13:20]
                    else:
                        qar = qar_list[i]
                    if qhr_list == None:
                        qhr = dof_values[20:24]
                    else:
                        qhr = qhr_list[i]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    if time_d > 0.0:
                        self.env.UpdatePublishedBodies()
                    check = self.env.CheckCollision(self.robot_rave, report)
                    if first_collision == None and report.numCols > 0:
                        first_collision = i
                        print "first collision at step %s"%(i)
                    elif report.numCols > 0:
                        print "collision at step %s"%(i)
                    if time_d > 0.0:
                        rospy.sleep(time_d)
#        self.robot_rave_update_lock.release()
        return first_collision

    def getMesh(self, name):
        body = self.env.GetKinBody(name)
        if body == None:
            return None
        link = body.GetLinks()[0]
        col = link.GetCollisionData()
        return col.vertices, col.indices

    def runGraspFromTrans(self, gmodel, grasp):
        """squeeze the fingers to test whether the completed grasp only collides with the target, throws an exception if it fails. Otherwise returns the Grasp parameters. Uses the grasp transformation directly."""
        with gmodel.robot:
            gmodel.robot.SetActiveManipulator(gmodel.manip)
            gmodel.robot.SetDOFValues(grasp[gmodel.graspindices.get('igrasppreshape')],gmodel.manip.GetGripperIndices())
            gmodel.robot.SetTransform(np.dot(gmodel.getGlobalGraspTransform(grasp),np.dot(np.linalg.inv(gmodel.manip.GetEndEffectorTransform()),gmodel.robot.GetTransform())))
            gmodel.robot.SetActiveDOFs(gmodel.manip.GetGripperIndices())
            if len(gmodel.manip.GetGripperIndices()) == 0:
                return [],[[],gmodel.robot.GetTransform()],None,None
            return gmodel.grasper.Grasp(transformrobot=False,target=gmodel.target,onlycontacttarget=True, forceclosure=True, execute=False, outputfinal=True,translationstepmult=gmodel.translationstepmult,finestep=gmodel.finestep)

    def getGraspQHull(self, target_name, points):
#        print "getGraspQHull points: %s"%(len(points))
        try:
            planes, faces, triangles = self.gmodel[target_name].grasper.ConvexHull(np.array(points), returnplanes=True,returnfaces=False,returntriangles=False)
        except:
            return None
#        print "getGraspQHull planes: %s"%(len(planes))
        return planes

    def getGraspQuality(self, target_name, grasp):
        return grasp[self.gmodel[target_name].graspindices.get('forceclosure')]

    def generateGWSwrenches(self, target_name, wrenches):
        qhullpoints = []
        for wr in wrenches:
            qhullpoints.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])

        qhullplanes = self.getGraspQHull(target_name, qhullpoints)
        return qhullplanes

    def generateGWS(self, target_name, contacts):
        friction = 1.0
        Nconepoints = 8
        fdeltaang = 2.0*math.pi/float(Nconepoints)
        qhullpoints = []
        for c in contacts:
            p = PyKDL.Vector(c[0], c[1], c[2])
            nz = PyKDL.Vector(c[3], c[4], c[5])
            if abs(nz.z()) < 0.7:
                nx = PyKDL.Vector(0,0,1)
            elif abs(nz.y()) < 0.7:
                nx = PyKDL.Vector(0,1,0)
            else:
                nx = PyKDL.Vector(1,0,0)
            ny = nz * nx
            nx = ny * nz
            ny.Normalize()
            nz.Normalize()
            R_n = PyKDL.Frame(PyKDL.Rotation(nx,ny,nz))
            fangle = 0.0
            for cp in range(Nconepoints):
                nn = R_n * PyKDL.Frame(PyKDL.Rotation.RotZ(fangle)) * (PyKDL.Vector(friction,0,1))
                fangle += fdeltaang
                tr = p * nn
                wr = PyKDL.Wrench(nn,tr)
                qhullpoints.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])

        qhullplanes = self.getGraspQHull(target_name, qhullpoints)
        return qhullplanes

    def getQualityMeasure2(self, qhull, wr):
        if qhull == None:
            return 0.0

        wr6 = [wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]]
        mindist = None
        for qp in qhull:
            n = np.array([qp[0],qp[1],qp[2],qp[3],qp[4],qp[5]])
            if np.dot(n,n) > 1.00001 or np.dot(n,n) < 0.9999:
                print "ERROR: getQualityMeasure2: np.dot(n,n): %s"%(np.dot(n,n))
                exit(0)
            dot = np.dot(np.array(wr6), n)
            if dot > 0:
                dqp = -qp[6]/dot
                if mindist == None or mindist > dqp:
                    mindist = dqp
        return mindist

    def getFinalConfig(self, target_name, grasp, show=False, sim_grip=None, remove_other_objects=False):
        hand_config = None
        contacts = None
        normals_O = None
#        mindist_grav = None
#        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.gmodel[target_name].GripperVisibility(self.robot_rave.GetActiveManipulator()):
                if remove_other_objects:
                    all_bodies = self.env.GetBodies()
                    for body in all_bodies:
                        name = body.GetName()
                        if name != target_name and name != "velma":
                            self.env.Remove(body)
                try:
                    contacts,finalconfig,mindist,volume = self.runGraspFromTrans(self.gmodel[target_name], grasp)
                    hand_config = [
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerTwoKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerThreeKnuckleTwoJoint")],
                    finalconfig[0][self.robot_rave.GetJointIndex("right_HandFingerOneKnuckleOneJoint")],
                    ]

                    ind = self.robot_rave.GetActiveManipulator().GetGripperIndices()
                    hand_config2 = [
                    finalconfig[0][ind[0]],
                    finalconfig[0][ind[1]],
                    finalconfig[0][ind[2]],
                    finalconfig[0][ind[3]],
                    ]

                    if show:
                        self.robot_rave.SetTransform(np.dot(self.gmodel[target_name].getGlobalGraspTransform(grasp),np.dot(np.linalg.inv(self.robot_rave.GetActiveManipulator().GetEndEffectorTransform()),self.robot_rave.GetTransform())))

                        self.robot_rave.SetDOFValues([hand_config2[0], hand_config2[1], hand_config2[2], hand_config2[3]],self.robot_rave.GetActiveManipulator().GetGripperIndices())

                        if sim_grip != None:
                            T_B_O = self.getPose(target_name)
                            contacts = []
                            for c in sim_grip.contacts:
                                T_World_C = self.T_World_Br * T_B_O * c[1]
                                p = T_World_C * PyKDL.Vector()
                                n = PyKDL.Frame(T_World_C.M) * PyKDL.Vector(0,0,1)
                                contacts.append([p[0],p[1],p[2],n[0],n[1],n[2]])

                        self.gmodel[target_name].contactgraph = self.gmodel[target_name].drawContacts(contacts)
                        self.env.UpdatePublishedBodies()
                        raw_input('press any key to continue: ')

                        self.gmodel[target_name].contactgraph = None

                except planning_error,e:
                    print "getFinalConfig: planning error:"
                    print e
                if remove_other_objects:
                    for body in all_bodies:
                        name = body.GetName()
                        if name != target_name and name != "velma":
                            self.env.Add(body)
#        self.robot_rave_update_lock.release()
        if contacts == None:
            contacts_ret_O = None
        else:
            body = self.env.GetKinBody(target_name)
            T_World_O = self.OpenraveToKDL(body.GetTransform())
            T_O_World = T_World_O.Inverse()
            contacts_ret_O = []
            normals_O = []
            for c in contacts:
                contacts_ret_O.append(T_O_World * PyKDL.Vector(c[0], c[1], c[2]))
                normals_O.append(PyKDL.Frame(T_O_World.M) * PyKDL.Vector(c[3], c[4], c[5]))
        return hand_config, contacts_ret_O, normals_O

    def generateWrenchesforAllGrasps(self, graspable_object_name):
            if not hasattr(self, 'wrenches_map') or self.wrenches_map == None:
                self.wrenches_map = {}

            self.wrenches_map[graspable_object_name] = []

            # calculate the scaling factor for torque for wrench reduction
            vertices, faces = self.getMesh(graspable_object_name)
            max_radius = None
            for v in vertices:
                radius = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
                if max_radius == None or radius > max_radius:
                    max_radius = radius
            max_radius2 = max_radius * max_radius

            # calculate GWS for each grasp
            for idx in range(0, self.getGraspsCount(graspable_object_name)):
                self.wrenches_map[graspable_object_name].append(None)

                if idx % 100 == 0:
                    print "generateWrenchesforAllGrasps: index: %s"%(idx)
                grasp = self.getGrasp(graspable_object_name, idx)
                q, contacts_O, normals_O = self.getFinalConfig(graspable_object_name, grasp, remove_other_objects=True)
                if contacts_O == None:
                    contacts_O = []
#                print "grasp_idx: %s   contacts: %s"%(idx, len(contacts_O))
                if len(contacts_O) == 0:
                    continue

                wrenches = []
                for c_idx in range(len(contacts_O)):
                    wrenches += velmautils.contactToWrenches(contacts_O[c_idx], normals_O[c_idx], 1.0, 8)

                wrenches_reduced = []
                for wr in wrenches:
                    add_wr = True
                    for wr_r in wrenches_reduced:
                        dist = math.sqrt(
                        (wr[0]-wr_r[0])*(wr[0]-wr_r[0]) +
                        (wr[1]-wr_r[1])*(wr[1]-wr_r[1]) +
                        (wr[2]-wr_r[2])*(wr[2]-wr_r[2]) +
                        (wr[3]-wr_r[3])*(wr[3]-wr_r[3])/max_radius2 +
                        (wr[4]-wr_r[4])*(wr[4]-wr_r[4])/max_radius2 +
                        (wr[5]-wr_r[5])*(wr[5]-wr_r[5])/max_radius2 )
                        if dist < 0.05:
                            add_wr = False
                            break
                    if add_wr:
                        wrenches_reduced.append(wr)
#                print "wrenches:   %s   %s   planes: %s"%(len(wrenches), len(wrenches_reduced), len(qhullplanes))
                self.wrenches_map[graspable_object_name][-1] = wrenches_reduced

    def generateGWSforAllGrasps(self, graspable_object_name):
        if not hasattr(self, 'gws_map') or self.gws_map == None:
            self.gws_map = {}

        self.gws_map[graspable_object_name] = []

        for wrenches in self.wrenches_map[graspable_object_name]:
            if wrenches == None:
                self.gws_map[graspable_object_name].append(None)
            else:
                qhullplanes = self.generateGWSwrenches(graspable_object_name, wrenches)
                self.gws_map[graspable_object_name].append(qhullplanes)

    def getGWSforGraspId(self, graspable_object_name, grasp_idx):
        if not hasattr(self, 'gws_map') or self.gws_map == None:
            self.gws_map = {}

        if not graspable_object_name in self.gws_map:
            self.gws_map[graspable_object_name] = []
            for i in range(self.getGraspsCount(graspable_object_name)):
                self.gws_map[graspable_object_name].append(None)

        if self.gws_map[graspable_object_name][grasp_idx] == None:
            wrenches = self.wrenches_map[graspable_object_name][grasp_idx]
            if wrenches == None:
                self.gws_map[graspable_object_name][grasp_idx] = []
            else:
                qhullplanes = self.generateGWSwrenches(graspable_object_name, wrenches)
                self.gws_map[graspable_object_name][grasp_idx] = qhullplanes

        return self.gws_map[graspable_object_name][grasp_idx]


    def saveWrenchesforAllGrasps(self, graspable_object_name, filename_wrenches):
        with open(filename_wrenches, 'w') as f:
            for wrenches in self.wrenches_map[graspable_object_name]:
                if wrenches == None:
                    f.write('None\n')
                else:
                    line = ""
                    for wr in wrenches:
                        line += str(wr[0]) + " " + str(wr[1]) + " " + str(wr[2]) + " " + str(wr[3]) + " " + str(wr[4]) + " " + str(wr[5]) + " "
                    f.write(line + '\n')

    def loadWrenchesforAllGrasps(self, graspable_object_name, filename_wrenches):
        if not hasattr(self, 'wrenches_map') or self.wrenches_map == None:
            self.wrenches_map = {}

        self.wrenches_map[graspable_object_name] = []
        with open(filename_wrenches, 'r') as f:
            for line in f:
                line_s = line.split()
                if len(line_s) == 0:
                    break
                if line_s[0] == 'None':
                    self.wrenches_map[graspable_object_name].append(None)
                    continue

                self.wrenches_map[graspable_object_name].append([])
                for i in range(0, len(line_s), 6):
                    self.wrenches_map[graspable_object_name][-1].append(
                    PyKDL.Wrench(PyKDL.Vector(float(line_s[i+0]), float(line_s[i+1]), float(line_s[i+2])), PyKDL.Vector(float(line_s[i+3]), float(line_s[i+4]), float(line_s[i+5]))))

    def showFinalConfigs(self, target_name, grasps):
       self.robot_rave_update_lock.acquire()
       with self.robot_rave.CreateRobotStateSaver():
           with self.gmodel[target_name].GripperVisibility(self.robot_rave.GetActiveManipulator()):
                try:

                    hand_configs = []
                    ind = self.robot_rave.GetActiveManipulator().GetGripperIndices()
                    for grasp in grasps:
                        contacts,finalconfig,mindist,volume = self.gmodel[target_name].runGraspFromTrans(grasp)
                        hand_configs.append([
                        finalconfig[0][ind[0]],
                        finalconfig[0][ind[1]],
                        finalconfig[0][ind[2]],
                        finalconfig[0][ind[3]],
                        ])

                    added_robots = []
                    index = 0
                    for hand_config2 in hand_configs:
                        if index == 0:
                            self.robot_rave.SetTransform(np.dot(self.gmodel[target_name].getGlobalGraspTransform(grasps[index]),np.dot(np.linalg.inv(self.robot_rave.GetActiveManipulator().GetEndEffectorTransform()),self.robot_rave.GetTransform())))

                            self.robot_rave.SetDOFValues([hand_config2[0], hand_config2[1], hand_config2[2], hand_config2[3]],self.robot_rave.GetActiveManipulator().GetGripperIndices())

#                            self.gmodel[target_name].contactgraph = self.gmodel[target_name].drawContacts(contacts)
                            self.env.UpdatePublishedBodies()

#                            self.gmodel[target_name].contactgraph = None
                        elif index <= self.max_robots and self.newrobots != None:

                            self.env.Add(self.newrobots[index-1],True)
                            self.newrobots[index-1].SetTransform(np.dot(self.gmodel[target_name].getGlobalGraspTransform(grasps[index]),np.dot(np.linalg.inv(self.newrobots[index-1].GetActiveManipulator().GetEndEffectorTransform()),self.newrobots[index-1].GetTransform())))
                            self.newrobots[index-1].SetDOFValues([hand_config2[0], hand_config2[1], hand_config2[2], hand_config2[3]],ind)
                            added_robots.append(self.newrobots[index-1])

#                            self.gmodel[target_name].contactgraph = self.gmodel[target_name].drawContacts(contacts)
                            self.env.UpdatePublishedBodies()
#                            raw_input('press any key to continue: ')

#                            self.gmodel[target_name].contactgraph = None

                        index += 1
                    raw_input('press any key to continue: ')

                    for newrobot in added_robots:
                        self.env.Remove(newrobot)

                except planning_error,e:
                    print "getFinalConfig: planning error:"
                    print e
       self.robot_rave_update_lock.release()

    def grab(self, name):
        body = self.env.GetKinBody(name)
        with self.env:
            self.robot_rave.Grab( body )

    def release(self):
        with self.env:
            self.robot_rave.ReleaseAllGrabbed()

    def getVisibility(self, name, T_Br_C, qt=None, qar=None, qal=None, qhr=None, qhl=None, pub_marker=None, fov_x=None, fov_y=None, min_dist=0.01):
        # remove the head sphere from environment
        self.env.Remove(self.obj_head_sphere)
        if name in self.visibility_surface_samples_dict:
            points = self.visibility_surface_samples_dict[name]
        else:
            vertices, indices = self.getMesh(name)
            points = velmautils.sampleMesh(vertices, indices, 0.025, [PyKDL.Vector()], 1.0)
            print "points for visibility test: %s"%(len(points))
            self.visibility_surface_samples_dict[name] = points
        T_World_C = self.T_World_Br * T_Br_C
        T_C_World = T_World_C.Inverse()
        R_C_World = PyKDL.Frame(T_C_World.M)
        cam_pt_in_World = T_World_C * PyKDL.Vector()
        if fov_x != None and fov_y != None:
            tg_fov_x = math.tan(fov_x/2.0)
            tg_fov_y = math.tan(fov_y/2.0)
        m_id = 0
        if qt != None or qar != None or qal != None or qhr != None or qhl != None:
            self.robot_rave_update_lock.acquire()
            with self.env:
                with self.robot_rave.CreateRobotStateSaver():
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    if qt == None:
                        qt = dof_values[0:2]
                    if qal == None:
                        qal = dof_values[2:9]
                    if qhl == None:
                        qhl = dof_values[9:13]
                    if qar == None:
                        qar = dof_values[13:20]
                    if qhr == None:
                        qhr = dof_values[20:24]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)
                    body = self.env.GetKinBody(name)
                    T_World_O = self.OpenraveToKDL(body.GetTransform())
                    hits = 0
                    all_hits = 0
                    for p in points:
                        p_in_World = T_World_O * p
                        d = p_in_World - cam_pt_in_World
                        d_len = d.Norm()
                        # elongate the ray by 1 cm
                        d = d * (d_len + 0.01)/d_len
                        check_collision = True
                        if fov_x != None and fov_y != None:
                            # check the ray in camera frame
                            d_cam = R_C_World * d
                            if d_cam.z() < min_dist or math.fabs(d_cam.x()/d_cam.z()) > tg_fov_x or math.fabs(d_cam.y()/d_cam.z()) > tg_fov_y:
                                check_collision = False
                        if check_collision:
                            report = CollisionReport()
                            ret = self.env.CheckCollision(Ray([cam_pt_in_World.x(),cam_pt_in_World.y(),cam_pt_in_World.z()],[d.x(),d.y(),d.z()]), report)
                            if pub_marker != None:
                                m_id = pub_marker.publishVectorMarker(cam_pt_in_World, cam_pt_in_World+d, m_id, 1, 1, 1, frame='world', namespace='default', scale=0.001)
                            if report.numCols > 0:
                                all_hits += 1
                                parent = report.plink1.GetParent()
                                if parent.GetName() == name:
                                    hits += 1
#                    print "all_rays: %s   all_hits: %s   hits: %s"%(len(points), all_hits, hits)
            self.robot_rave_update_lock.release()
        else:
            body = self.env.GetKinBody(name)

            T_World_O = self.OpenraveToKDL(body.GetTransform())
            hits = 0
            all_hits = 0
            for p in points:
                p_in_World = T_World_O * p
                d = p_in_World - cam_pt_in_World
                d_len = d.Norm()
                d = d * (d_len + 0.01)/d_len
                check_collision = True
                if fov_x != None and fov_y != None:
                    # check the ray in camera frame
                    d_cam = R_C_World * d
                    if d_cam.z() < min_dist or math.fabs(d_cam.x()/d_cam.z()) > tg_fov_x or math.fabs(d_cam.y()/d_cam.z()) > tg_fov_y:
                        check_collision = False
                if check_collision:
                    report = CollisionReport()
                    ret = self.env.CheckCollision(Ray([cam_pt_in_World.x(),cam_pt_in_World.y(),cam_pt_in_World.z()],[d.x(),d.y(),d.z()]), report)
                    if pub_marker != None:
                        m_id = pub_marker.publishVectorMarker(cam_pt_in_World, cam_pt_in_World+d, m_id, 1, 1, 1, frame='world', namespace='default', scale=0.001)
                    if report.numCols > 0:
                        all_hits += 1
                        parent = report.plink1.GetParent()
                        if parent.GetName() == name:
                            hits += 1

#            print "all_rays: %s   all_hits: %s   hits: %s"%(len(points), all_hits, hits)
        # add the head sphere from environment
        self.env.Add(self.obj_head_sphere, True)

        return float(hits)/float(len(points))

    def findIkSolution(self, T_Br_E):
        return self.ikmodel.manip.FindIKSolution(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def findIkSolutions(self, T_Br_E):
        return self.ikmodel.manip.FindIKSolutions(self.KDLToOpenrave(self.T_World_Br * T_Br_E), IkFilterOptions.CheckEnvCollisions)

    def getConfigurationPenalty(self, q):
        # punish for singularities in end configuration
        if abs(q[1]) < 30.0/180.0*math.pi:
            return 100000.0
        if abs(q[3]) < 30.0/180.0*math.pi:
            return 100000.0
        if abs(q[5]) < 30.0/180.0*math.pi:
            return 100000.0

        penalty = 0.0
        if abs(q[1]) < 40.0/180.0*math.pi:
            penalty += 40.0/180.0*math.pi - abs(q[1])
        if abs(q[3]) < 40.0/180.0*math.pi:
            penalty += 40.0/180.0*math.pi - abs(q[3])
        if abs(q[5]) < 40.0/180.0*math.pi:
            penalty += 40.0/180.0*math.pi - abs(q[5])
        penalty *= 10.0

        for i in range(0, 7):
            if abs(q[i]-self.lower_lim["right_arm"][i]) < 40.0/180.0*math.pi:
                penalty += 40.0/180.0*math.pi - abs(q[i]-self.lower_lim["right_arm"][i])
            if abs(q[i]-self.upper_lim["right_arm"][i]) < 40.0/180.0*math.pi:
                penalty += 40.0/180.0*math.pi - abs(q[i]-self.upper_lim["right_arm"][i])

        return penalty
        
    def getBestFinalConfigs(self, T_Br_E):
        q_list = self.findIkSolutions(T_Br_E)
#        print "ik solutions: %s"%(len(q_list))
        q_penalty = []
        for q in q_list:
            q_penalty.append([self.getConfigurationPenalty(q), q])
        q_sorted = sorted(q_penalty, key=operator.itemgetter(0))
        return q_sorted

    def findFreeSpaceSphere(self, radius, dist_from_shoulder):
        self.robot_rave_update_lock.acquire()

        # first, find the free space in the best area for manipulation
        with self.robot_rave.CreateRobotStateSaver():
                    self.robot_rave.GetController().Reset(0)
                    dof_values = self.robot_rave.GetDOFValues()
                    qt = dof_values[0:2]
                    qal = dof_values[2:9]
                    qhl = dof_values[9:13]
                    qar = [0.0, -90.0/180*math.pi, 0.0, 0.0, 0.0, 0.0, 0.0]
                    qhr = dof_values[20:24]
                    dof_values = list(qt) + list(qal) + list(qhl) + list(qar) + list(qhr)
                    self.robot_rave.SetDOFValues(dof_values)

                    with self.env:
                        wrist_sphere = self.addSphere("wrist_sphere", 2.0 * radius)
                        T_Br_T2 = self.getLinkPose("torso_link2")
                        T_Br_L2r = self.getLinkPose("right_arm_2_link")
                        Pc_T2 = T_Br_T2.Inverse() * T_Br_L2r * PyKDL.Vector()
                        it = 0
                        while True:
                            v = self.normals_sphere_5_deg[random.randint(0, len(self.normals_sphere_5_deg)-1)]
                            P_Br = T_Br_T2 * (Pc_T2 + v * dist_from_shoulder)
                            wrist_sphere.SetTransform(self.KDLToOpenrave(self.T_World_Br*PyKDL.Frame(PyKDL.Vector(P_Br))))
                            report = CollisionReport()
                            self.env.CheckCollision(wrist_sphere, report)

                            if report.numCols == 0:
                                break
                            it += 1
                            if it > 200:
                                P_Br = None
                                break
                        self.env.Remove(wrist_sphere)

        self.robot_rave_update_lock.release()

        return P_Br

    def planMove(self, goal):
        traj = None
        try:
            with self.robot_rave:
                self.robot_rave.SetActiveDOFs(self.manipulator_dof_indices_map["right_arm"])
                traj = self.basemanip.MoveActiveJoints(goal=goal,execute=False,outputtrajobj=True, steplength=0.02, postprocessingplanner="LinearSmoother")
#, postprocessingparameters="<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>100</_nmaxiterations></_postprocessing>")#, steplength=0.002, maxiter=100000, maxtries=100)
#                print "waypoints: %s"%(traj.GetNumWaypoints())
        except planning_error,e:
            print "planMoveThroughGoals: planning error"
        return traj

    def extendAllObjects(self, ext_value):
        if not hasattr(self, 'removed_bodies'):
            self.removed_bodies = None

        if self.removed_bodies != None:
            print "error: extendAllObjects"
            exit(0)

        all_bodies = self.env.GetBodies()
        self.removed_bodies = []
        for body in all_bodies:
            name = body.GetName()
            if name != "velma" and len(body.GetLinks()) == 1:
                self.removed_bodies.append(body)
                self.env.Remove(body)

        self.added_bodies = []
        ext_idx = 0
        for body in self.removed_bodies:
            for link in body.GetLinks():
                mesh = link.GetCollisionData()

#            mesh_v_f_map = {}
#            for face_idx in range(len(mesh.indices)):
#                face = mesh.indices[face_idx]
#                for v_idx in face:
#                    if not v_idx in mesh_v_f_map:
#                        mesh_v_f_map[v_idx] = [face_idx]
#                    else:
#                        mesh_v_f_map[v_idx].append(face_idx)

#            normals = []
#            for face in mesh.indices:
#                v0 = mesh.vertices[face[0]]
#                v1 = mesh.vertices[face[1]]
#                v2 = mesh.vertices[face[2]]
#                a = PyKDL.Vector(v1[0], v1[1], v1[2]) - PyKDL.Vector(v0[0], v0[1], v0[2])
#                b = PyKDL.Vector(v2[0], v2[1], v2[2]) - PyKDL.Vector(v0[0], v0[1], v0[2])
#                n = a * b
#                n.Normalize()
#                normals.append(n)

            for v_idx in range(len(mesh.vertices)):
                # assume boxes only
                for dof_idx in range(3):
                    if mesh.vertices[v_idx][dof_idx] > 0:
                        mesh.vertices[v_idx][dof_idx] += ext_value
                    else:
                        mesh.vertices[v_idx][dof_idx] -= ext_value

            body_ext = RaveCreateKinBody(self.env,'')
            body_ext.SetName("extended_body_"+str(ext_idx))
            body_ext.InitFromTrimesh(mesh, True)
            body_ext.SetTransform(body.GetTransform())
            self.env.Add(body_ext,True)
            self.added_bodies.append(body_ext)
            ext_idx += 1

    def restoreExtendedObjects(self):
        for body in self.added_bodies:
            self.env.Remove(body)

        for body in self.removed_bodies:
            self.env.Add(body,True)

        self.removed_bodies = None
        self.added_bodies = None

    def planMoveForRightArm(self, T_Br_E, q_dest, maxiter=500, verbose_print=False, penalty_threshold=None, extend_objects=False):

        if T_Br_E != None and q_dest == None:
            q_list = self.getBestFinalConfigs(T_Br_E)
            if len(q_list) < 10:
                print "planMoveForRightArm: strange pose - a few ik solutions"
                return None
        elif T_Br_E == None and q_dest != None:
            q_list = [[0.0, q_dest]]
        else:
            print "planMoveForRightArm: wrong arguments: %s %s"%(T_Br_E, q_dest)
            return None

        init_q = self.robot_rave.GetDOFValues(self.manipulator_dof_indices_map["right_arm"])
        for q_s in q_list:

            traj = None
            goal = q_s[1]

            penalty = q_s[0]

            if penalty_threshold != None and penalty > penalty_threshold:
                return None

            if penalty > 10000.0:
                break

            traj = self.planMove(goal)
            if traj == None:
                print "error: planMoveThroughGoals"
                continue

            # verify the trajectory
            conf = traj.GetConfigurationSpecification()
            q_traj = []
            steps2 = max(2, int(traj.GetDuration()*200.0))
            for t in np.linspace(0.0, traj.GetDuration(), steps2):
                q = conf.ExtractJointValues(traj.Sample(t), self.robot_rave, self.manipulator_dof_indices_map["right_arm"])
                q_traj.append(list(q))
            break

        if traj == None:
            print "planMoveForRightArm: planning error"
            return None

        if verbose_print:
            print "all groups:"
            for gr in conf.GetGroups():
                print gr.name

        def printGroup(gr):
            print "offset: %s   dof: %s   name: %s   interpolation: %s"%(gr.offset, gr.dof, gr.name, gr.interpolation)

        try:
            gr_tim = conf.GetGroupFromName("deltatime")
            tim = []
            if verbose_print:
                print "gr_tim:"
                printGroup(gr_pos)
        except openrave_exception:
            gr_tim = None
            tim = None
            if verbose_print:
                print "gr_tim == None"
        try:
            gr_pos = conf.GetGroupFromName("joint_values")
            pos = []
            if verbose_print:
                print "gr_pos:"
                printGroup(gr_pos)
        except openrave_exception:
            gr_pos = None
            pos = None
            if verbose_print:
                print "gr_pos == None"
        try:
            gr_vel = conf.GetGroupFromName("joint_velocities")
            vel = []
            if verbose_print:
                print "gr_vel:"
                printGroup(gr_vel)
        except openrave_exception:
            gr_vel = None
            vel = None
            if verbose_print:
                print "gr_vel == None"
        try:
            gr_acc = conf.GetGroupFromName("joint_accelerations")
            acc = []
            if verbose_print:
                print "gr_acc:"
                printGroup(gr_acc)
        except openrave_exception:
            gr_acc = None
            acc = None
            if verbose_print:
                print "gr_acc == None"

        if verbose_print:
            print "waypoints: %s"%(traj.GetNumWaypoints())

        for idx in range(0, traj.GetNumWaypoints()):
            w = traj.GetWaypoint(idx)
            if pos != None:
                pos.append( [w[gr_pos.offset], w[gr_pos.offset + 1], w[gr_pos.offset + 2], w[gr_pos.offset + 3], w[gr_pos.offset + 4], w[gr_pos.offset + 5], w[gr_pos.offset + 6]] )
            if vel != None:
               vel.append( [w[gr_vel.offset], w[gr_vel.offset + 1], w[gr_vel.offset + 2], w[gr_vel.offset + 3], w[gr_vel.offset + 4], w[gr_vel.offset + 5], w[gr_vel.offset + 6]] )
            if acc != None:
               acc.append( [w[gr_acc.offset], w[gr_acc.offset + 1], w[gr_acc.offset + 2], w[gr_acc.offset + 3], w[gr_acc.offset + 4], w[gr_acc.offset + 5], w[gr_acc.offset + 6]] )
            if tim != None:
               tim.append( w[gr_tim.offset] )

        if verbose_print:
            print "pos"
            print pos
            print "tim"
            print tim
            if tim != None:
                print "tim sum: %s"%(math.fsum(tim))
            print "duration: %s"%(traj.GetDuration())
        return pos, vel, acc, tim, q_traj, penalty

    def checkRobotCollision(self):
        report = CollisionReport()
        if self.env.CheckCollision(self.robot_rave, report):
            return True

        return False

    def printCollisions(self):
        report = CollisionReport()
        if self.env.CheckCollision(self.robot_rave, report):
            print 'robot in collision:'
            if report.plink1 != None:
                print report.plink1
            if report.plink2 != None:
                print report.plink2
            if report.numCols != None:
                print report.numCols
        else:
            print "no collisions"

    def checkGripperCollision(self, target_name, grasp_idx):
        collision = False
        grasp = self.gmodel[target_name].grasps[grasp_idx]
        self.robot_rave_update_lock.acquire()
        with self.robot_rave.CreateRobotStateSaver():
            with self.gmodel[target_name].GripperVisibility(self.gmodel[target_name].manip):
                with self.env:
                    self.gmodel[target_name].setPreshape(grasp)
                    Tgrasp = self.gmodel[target_name].getGlobalGraspTransform(grasp,collisionfree=False)
                    Tdelta = np.dot(Tgrasp,np.linalg.inv(self.gmodel[target_name].manip.GetEndEffectorTransform()))
                    for link in self.gmodel[target_name].manip.GetChildLinks():
                        link.SetTransform(np.dot(Tdelta,link.GetTransform()))
                    report = CollisionReport()
                    if self.env.CheckCollision(self.robot_rave, report):
                        collision = True
        self.robot_rave_update_lock.release()
        return collision

    def getGraspsForObjectTransport(self, target_name, transport_T_B_O):
        current_T_B_O = self.getPose(target_name)

        valid_indices = range(self.getGraspsCount(target_name))

        # eliminate the colliding grasps
#        valid_indices = self.generateGrasps(target_name, show=False, checkcollision=True, checkik=False, checkgrasper=False)
#        print len(valid_indices)
        for T_B_Od in transport_T_B_O:
            self.updatePose(target_name, T_B_Od)
            valid_indices = self.generateGrasps(target_name, show=False, checkcollision=True, checkik=False, checkgrasper=False, grasp_indices=valid_indices)
            print len(valid_indices)
#        self.updatePose(target_name, current_T_B_O)

        # eliminate non-reachable grasps (ik)
#        valid_indices = self.generateGrasps(target_name, show=False, checkcollision=True, checkik=True, checkgrasper=False, grasp_indices=valid_indices)
#        print len(valid_indices)
        for T_B_Od in transport_T_B_O:
            self.updatePose(target_name, T_B_Od)
            valid_indices = self.generateGrasps(target_name, show=False, checkcollision=True, checkik=True, checkgrasper=False, grasp_indices=valid_indices)
            print len(valid_indices)

        self.updatePose(target_name, current_T_B_O)

        return valid_indices

