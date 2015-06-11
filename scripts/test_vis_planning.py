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

class TestOrOctomap:
    """

"""

    def uniformInBall(self, r, n):
        while True:
            vec = np.empty(n)
            for i in range(n):
                vec[i] = random.uniform(-r, r)
            break
#            if np.linalg.norm(vec) <= r:
#                break

        return vec
        
    class ProlateHyperspheroid:

        def updateRotation(self):
            # Mark the transform as out of date
            #self.isTransformUpToDate_ = False;
            # If the minTransverseDiameter_ is too close to 0, we treat this as a circle.
            circleTol = 1E-9;
            if self.minTransverseDiameter_ < circleTol:
                #rotationWorldFromEllipse_.setIdentity(dim_, dim_);
                np.identity(self.dim_)
            else:
                # Variables
                # The transverse axis of the PHS expressed in the world frame.
                #Eigen::VectorXd transverseAxis(dim_);
                # The matrix representation of the Wahba problem
                #Eigen::MatrixXd wahbaProb(dim_, dim_);
                # The middle diagonal matrix in the SVD solution to the Wahba problem
                #Eigen::VectorXd middleM(dim_);

                # Calculate the major axis, storing as the first eigenvector
                transverseAxis = (self.xFocus2_ - self.xFocus1_ )/self.minTransverseDiameter_;

                # Calculate the rotation that will allow us to generate the remaining eigenvectors
                # Formulate as a Wahba problem, first forming the matrix a_j*a_i' where a_j is the transverse axis if the ellipse in the world frame, and a_i is the first basis vector of the world frame (i.e., [1 0 .... 0])
                #wahbaProb = transverseAxis*Eigen::MatrixXd::Identity(dim_, dim_).col(0).transpose();
                id_col0_inv = np.zeros(self.dim_)
                id_col0_inv[0] = 1
                wahbaProb = np.transpose(np.matrix(transverseAxis)) * id_col0_inv;
#                print "transverseAxis"
#                print transverseAxis
#                print "wahbaProb"
#                print wahbaProb

                # Then run it through the  SVD solver
                #Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd(wahbaProb, Eigen::ComputeFullV | Eigen::ComputeFullU);
                U, s, V = np.linalg.svd(wahbaProb, full_matrices=1, compute_uv=1)


                # Then calculate the rotation matrix from the U and V components of SVD
                # Calculate the middle diagonal matrix
                #middleM = Eigen::VectorXd::Ones(dim_);
                middleM = np.ones(self.dim_)
                # Make the last value equal to det(U)*det(V) (zero-based indexing remember)
                #middleM(dim_ - 1) = svd.matrixU().determinant()*svd.matrixV().determinant();
                middleM[self.dim_ - 1] = np.linalg.det(U) * np.linalg.det(V)

                # Calculate the rotation
                #rotationWorldFromEllipse_ = svd.matrixU()*middleM.asDiagonal()*svd.matrixV().transpose();
                self.rotationWorldFromEllipse_ = U * np.diag(middleM) * np.transpose(V)

        def __init__(self, n, focus1, focus2, minTransverseDiameter):
            self.transverseDiameter_ = 0.0
            self.dim_ = n
            self.xFocus1_ = focus1
            self.xFocus2_ = focus2
            self.minTransverseDiameter_ = minTransverseDiameter #np.linalg.norm(self.xFocus1_ - self.xFocus2_)
            self.xCentre_ = 0.5*(self.xFocus1_ + self.xFocus2_);
            self.updateRotation()

        def unitNBallMeasure(self, N):
#            return std::pow(std::sqrt(boost::math::constants::pi<double>()), static_cast<double>(N)) / boost::math::tgamma(static_cast<double>(N)/2.0 + 1.0);
            return pow(math.sqrt(math.pi), float(N)) / math.gamma(float(N)/2.0 + 1.0)

        def calcPhsMeasure(self, N, minTransverseDiameter, transverseDiameter):
            if transverseDiameter < minTransverseDiameter:
                print "Transverse diameter cannot be less than the minimum transverse diameter."
                exit(0)
            # Variable
            # The conjugate diameter:
            #double conjugateDiameter;
            # The Lebesgue measure return value
            #double lmeas;

            # Calculate the conjugate diameter:
            #conjugateDiameter = std::sqrt(transverseDiameter*transverseDiameter - minTransverseDiameter*minTransverseDiameter);
            conjugateDiameter = math.sqrt(transverseDiameter*transverseDiameter - minTransverseDiameter*minTransverseDiameter)

            # Calculate as a product series of the radii, noting that one is the transverse diameter/2.0, and the other N-1 are the conjugate diameter/2.0
            lmeas = transverseDiameter/2.0;
            for i in range(1, N): #(unsigned int i = 1u; i < N; ++i)
                lmeas = lmeas * conjugateDiameter/2.0;

            # Then multiplied by the volume of the unit n-ball.
            lmeas = lmeas * self.unitNBallMeasure(N);

            # Return:
            return lmeas;

        def updateTransformation(self):
            # Variables
            # The radii of the ellipse
            #Eigen::VectorXd diagAsVector(dim_);
            diagAsVector = np.empty(self.dim_)
            # The conjugate diameters:
            #double conjugateDiamater;

            # Calculate the conjugate radius
            #conjugateDiamater = std::sqrt(transverseDiameter_*transverseDiameter_ - minTransverseDiameter_*minTransverseDiameter_);
            conjugateDiamater = math.sqrt(self.transverseDiameter_*self.transverseDiameter_ - self.minTransverseDiameter_*self.minTransverseDiameter_)

            # Store into the diagonal matrix
            # All the elements but one are the conjugate radius
            diagAsVector.fill(conjugateDiamater/2.0);

            # The first element in diagonal is the transverse radius
            #diagAsVector(0) = 0.5*transverseDiameter_;
            diagAsVector[0] = 0.5*self.transverseDiameter_

            # Calculate the transformation matrix
            #transformationWorldFromEllipse_ = rotationWorldFromEllipse_*diagAsVector.asDiagonal();
            self.transformationWorldFromEllipse_ = self.rotationWorldFromEllipse_*np.diag(diagAsVector);

            # Calculate the measure:
            #phsMeasure_ = calcPhsMeasure(dim_, minTransverseDiameter_, transverseDiameter_);
            self.phsMeasure_ = self.calcPhsMeasure(self.dim_, self.minTransverseDiameter_, self.transverseDiameter_);

            # Mark as up to date
            #isTransformUpToDate_ = true;

        def transform(self, sphere):
            #if (isTransformUpToDate_ == false)
            #    throw Exception("The transformation is not up to date in the PHS class. Has the transverse diameter been set?");

            # Calculate the tranformation and offset, using Eigen::Map views of the data
            #Eigen::Map<Eigen::VectorXd>(phs, n) = transformationWorldFromEllipse_*Eigen::Map<const Eigen::VectorXd>(sphere, n) + xCentre_;
#            print "self.transformationWorldFromEllipse_"
#            print self.transformationWorldFromEllipse_
#            print "sphere"
#            print sphere
#            print "self.xCentre_"
#            print self.xCentre_
            return self.transformationWorldFromEllipse_ * np.transpose(np.matrix(sphere)) + np.transpose(np.matrix(self.xCentre_));

        def setTransverseDiameter(self, transverseDiameter):
            if transverseDiameter+0.001 < self.minTransverseDiameter_:
                print "ERROR: setTransverseDiameter:", transverseDiameter, " <", self.minTransverseDiameter_
            #    std::cout << transverseDiameter << " < " << minTransverseDiameter_ << std::endl;
            #    throw Exception("Transverse diameter cannot be less than the distance between the foci.");

            # Store and update if changed
#            if self.transverseDiameter_ != transverseDiameter:
                # Mark as out of date
                #isTransformUpToDate_ = false;

            # Store
            self.transverseDiameter_ = transverseDiameter+0.001

            # Update the transform
            self.updateTransformation()
            # No else, the diameter didn't change

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

    def inputThread(self, arg):
        raw_input("Press ENTER to stop planning...")
        self.stop_planning = True

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

        def isStateValid(openrave, q, dof_indices):
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

        # test visibility
#        if True:
#            print getVisibility(openrave)

        dof_names = [
        "head_pan_joint",
        "head_tilt_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
#        "left_arm_4_joint",
#        "left_arm_5_joint",
#        "left_arm_6_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
#        "right_arm_4_joint",
#        "right_arm_5_joint",
#        "right_arm_6_joint",
        "torso_0_joint",
#        "torso_1_joint",
        ]
        arms_dof = [
        [2,3,4,5],
        [6,7,8,9],
        ]

        dof_indices = []
        dof_limits = []
        for joint_name in dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            dof_limits.append( (lim_lo[0], lim_up[0]) )

        print "planning for %s joints"%(len(dof_indices))

        #
        # RRT
        #
        def RRT(openrave, dof_limits, dof_indices):
            V_vis = []
            V = []
            E = {}
            q_init = openrave.robot_rave.GetDOFValues(dof_indices)
            V.append(np.array(q_init))
            E[0] = []
            for k in range(10000):
                # RAND_FREE_CONF
                while True:
                    q_rand = []
                    for i in range(len(dof_indices)):
                        q_rand.append( random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01) )
                    break
#                print k
                q_rand_arr = np.array( q_rand )
                q_near = None
                # NEAREST_VERTEX
                for vi in range(len(V)):
                    q_diff = q_rand_arr - V[vi]
                    dist = math.sqrt( np.dot(q_diff, q_diff) )
                    if q_near == None or dist < q_near[0]:
                        q_near = (dist, vi, q_diff)
#                print "   nearest   %s"%(q_near[1])
                # NEW_CONF
                max_dist = 10.0/180.0*math.pi
                if q_near[0] > max_dist:
                    factor = max_dist / q_near[0]
                else:
                    factor = 1.0
                q_new = V[q_near[1]] + q_near[2] * factor
                if isStateValid(openrave, q_new, dof_indices):
                    V.append(q_new)
                    v_idx = len(V) - 1
                    E[v_idx] = [q_near[1]]
                    E[q_near[1]].append(v_idx)
#                    m_id = self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_near[1]][0], V[q_near[1]][1], V[q_near[1]][2]), PyKDL.Vector(q_new[0], q_new[1], q_new[2]), m_id, 0, 1, 0, frame='world', namespace='kinect_head_rays', scale=0.01)
                    vis = getVisibility(openrave, vis_bodies, q=q_new, dof_indices=dof_indices)
                    if vis > 0:
                        V_vis.append(q_new)
                        print " %s  vis"%(k)
#                    raw_input(".")
            for q in V_vis:
                openrave.robot_rave.SetDOFValues(q, dof_indices)
                openrave.env.UpdatePublishedBodies()
                raw_input(".")

        #
        # RRT*
        #
        def RRTstar(openrave, dof_limits, dof_indices):
            ETA = 60.0/180.0*math.pi
            gamma = 0.5
            d = len(dof_indices)
            
            def Distance(q1, q2):
                return np.linalg.norm(q1-q2)

            def GetPath(E, q_idx):
                if not q_idx in E:
                    return [q_idx]
                parent_idx = E[q_idx]
                q_list = GetPath(E, parent_idx)
                q_list.append(q_idx)
                return q_list

            def SampleFree(openrave, dof_indices, dof_limits, best_q, start_q, shortest_path_len, best_q_idx, V, E):
                search_near_path = False
                search_for_one_arm = False

                random_0_1 = random.uniform(0,1)
                if best_q_idx != None and random_0_1 < 0.05:
                    path = GetPath(E, best_q_idx)
                    search_near_path = True
                elif random_0_1 < 0.2:
                    arm_id = random.randint(0,1)
                    search_for_one_arm = True
                    # get one node
                    v_idx = random.choice(V.keys())#random.randint(0, len(V)-1)

                search_for_one_arm = False

                tries = 0
                while True:
                    tries += 1
                    if search_near_path:
                        q_rand_list = []
                        p_idx = random.randint(0, len(path)-1)
                        search_near_q = V[path[p_idx]]
                        for i in range(len(dof_limits)):
                            lim_lo = max(dof_limits[i][0], search_near_q[i]-10.0/180.0*math.pi)
                            lim_up = min(dof_limits[i][1], search_near_q[i]+10.0/180.0*math.pi)
                            q_rand_list.append( random.uniform(lim_lo, lim_up) )
#                            q_rand_list.append( search_near_q[i] )
#                        q_rand_list[random.randint(0, len(dof_limits)-1)] += random.uniform(-20.0/180.0*math.pi, 20.0/180.0*math.pi)
                        q_rand = np.array(q_rand_list)
                    elif search_for_one_arm:
                        q_rand = V[v_idx]
                        for dof_idx in arms_dof[arm_id]:
                            lim_lo = max(dof_limits[dof_idx][0], q_rand[dof_idx]-5.0/180.0*math.pi)
                            lim_up = min(dof_limits[dof_idx][1], q_rand[dof_idx]+5.0/180.0*math.pi)
                            q_rand[dof_idx] = random.uniform(lim_lo, lim_up)
                    elif best_q == None or self.phs == None:
                        q_rand_list = []
                        for i in range(len(dof_limits)):
                            q_rand_list.append( random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01) )
                        q_rand = np.array(q_rand_list)
                    else:
                        sphere = self.uniformInBall(1.0, len(dof_indices))
                        # Transform to the PHS
                        vec = self.phs.transform(sphere)
                        q_rand = np.empty(len(dof_indices))
                        outside_bounds = False
                        for i in range(len(dof_indices)):
                            q_rand[i] = vec[i]
                            if vec[i] < dof_limits[i][0] + 0.01 or vec[i] > dof_limits[i][1] - 0.01:
                                outside_bounds = True
                                break

                        if outside_bounds:
                            if tries > 2000:
                                print "disabling phs..."
#                                raw_input("Press ENTER to continue...")
                                rrt_switch = 0.9 * shortest_path_len
                                self.phs = None
                            continue

                    if isStateValid(openrave, q_rand, dof_indices):
#                        print "SampleFree:", tries
                        return q_rand

            def CostLine(q1, q2):
                cost = Distance(q1, q2)# * (np.linalg.norm(q1-self.q_init) + np.linalg.norm(q2-self.q_init)) * 0.5
                return cost

            def Nearest(V, q):
                q_near = None
                for vi in V:
                    dist = Distance(q, V[vi])
#                    dist = CostLine(q, V[vi])
                    if q_near == None or dist < q_near[0]:
                        q_near = (dist, vi)
                return q_near[1]

            def Steer(q_nearest, q_rand):
                dist = Distance(q_nearest, q_rand)
                if dist > ETA:
                    factor = ETA / dist
                else:
                    factor = 1.0
                q_diff = q_rand - q_nearest
                q_new = q_nearest + q_diff * factor
                return q_new

            def Near(V, q, near_dist):
                result = []
                for vi in V:
                    if Distance(q, V[vi]) < near_dist:
                        result.append(vi)
                return result

            def Cost(V, E, q_idx):
                if not q_idx in E:
                    return 0.0
                parent_idx = E[q_idx]
                if not parent_idx in V:
                    print "not parent_idx in V: %s"%(parent_idx)
                if not q_idx in V:
                    print "not q_idx in V: %s"%(q_idx)
                cost = CostLine(V[parent_idx], V[q_idx]) + Cost(V, E, parent_idx)
                return cost

            def CollisionFree(openrave, q1, q2, dof_indices, return_closest_q=False):
                dist = max(abs(q1-q2))
#                dist = Distance(q1,q2)
                steps = int(dist / (10.0/180.0*math.pi))
                if steps < 2:
                    steps = 2
                last_valid_q = None
                for i in range(1, steps):
#                    print "CollisionFree i: %s / %s"%(i, steps-1)
                    t = float(i)/float(steps-1)
                    current_q = q1 * (1.0-t) + q2 * t
                    if not isStateValid(openrave, current_q, dof_indices):
                        if return_closest_q:
                            return last_valid_q 
                        else:
                            return False
                    last_valid_q = current_q
                if return_closest_q:
                    return q2
                else:
                    return True

            def DrawPath(V, E, q_idx):
                if not q_idx in E:
                    return 0
                parent_idx = E[q_idx]
                m_id = DrawPath(V, E, parent_idx)
                m_id = self.pub_marker.publishVectorMarker(PyKDL.Vector(V[parent_idx][0], V[parent_idx][1], V[parent_idx][2]), PyKDL.Vector(V[q_idx][0], V[q_idx][1], V[q_idx][2]), m_id, 1, 0, 0, frame='world', namespace='shortest_path', scale=0.02)
                return m_id

            if debug:
                edge_ids = {}
                edge_id = 0

            rrt_switch = 0.0
            for lim in dof_limits:
                rrt_switch += (lim[1] - lim[0]) * (lim[1] - lim[0]) / 4.0

            rrt_switch = math.sqrt(rrt_switch)
#            rrt_switch = 0.0
            print "rrt_switch", rrt_switch

            self.phs = None

            shortest_path_len = None
            shortest_path_len_prev = None
            best_vis = 0
            best_q = None
            best_q_idx = None
            V_vis = []
            V = {}
            E = {}
            goal_V_ids = []
            q_init = openrave.robot_rave.GetDOFValues(dof_indices)
            self.q_init = np.array(q_init)
            q_new_idx = 0
            V[0] = np.array(q_init)
            for k in range(100000):
                time = []
                time.append( rospy.Time.now() )
                q_rand = SampleFree(openrave, dof_indices, dof_limits, best_q, q_init, shortest_path_len, best_q_idx, V, E)
                time.append( rospy.Time.now() )
                q_nearest_idx = Nearest(V, q_rand)
                time.append( rospy.Time.now() )
                q_nearest = V[q_nearest_idx]
                q_new = Steer(q_nearest, q_rand)
                col_free = CollisionFree(openrave, q_nearest, q_new, dof_indices)
#                q_new = CollisionFree(openrave, q_nearest, q_new, dof_indices, return_closest_q=True)
#                col_free = (q_new != None)
                time.append( rospy.Time.now() )
                if col_free:

                    if not isStateValid(openrave, q_new, dof_indices):
                        print "ERROR: invalid q_new"
                        exit(0)
                        
                    if shortest_path_len != None and CostLine(self.q_init, q_new) > shortest_path_len:
                        continue

#                    near_dist = gamma*math.log(len(V))/len(V)
                    near_dist = 120.0/180.0*math.pi#min(gamma*math.pow(math.log(len(V))/len(V), 1.0/d), ETA)
#                    print k, near_dist
#                    near_dist = min(gamma*math.log(len(V))/len(V), ETA)
                    q_near_idx_list = Near(V, q_new, near_dist)
#                    print "q_near_idx_list", len(q_near_idx_list)

                    time.append( rospy.Time.now() )

                    # sort the neighbours
                    cost_q_near_idx_list = []
                    q_min_idx = q_nearest_idx
                    c_min = Cost(V, E, q_nearest_idx) + CostLine(q_nearest, q_new)
                    for q_near_idx in q_near_idx_list:
                        if q_nearest_idx == q_near_idx:
                            continue
                        q_near = V[q_near_idx]
                        cost_q_near_idx_list.append( (Cost(V, E, q_near_idx) + CostLine(q_near, q_new), q_near_idx) ) 

                    sorted_cost_q_near_idx_list = sorted(cost_q_near_idx_list, key=operator.itemgetter(0))

                    time.append( rospy.Time.now() )

                    for (new_cost, q_near_idx) in sorted_cost_q_near_idx_list:
                        q_near = V[q_near_idx]
                        if CollisionFree(openrave, q_near, q_new, dof_indices):
                            q_min_idx = q_near_idx
                            c_min = new_cost
                            break

                    time.append( rospy.Time.now() )

                    # without sorting
#                    q_min_idx = q_nearest_idx
#                    c_min = Cost(V, E, q_nearest_idx) + CostLine(q_nearest, q_new)
#                    for q_near_idx in q_near_idx_list:
#                        q_near = V[q_near_idx]
#                        if CollisionFree(openrave, q_near, q_new, dof_indices):
#                            new_cost = Cost(V, E, q_near_idx) + CostLine(q_near, q_new)
#                            if new_cost < c_min:
#                                q_min_idx = q_near_idx
#                                c_min = new_cost

#                    if shortest_path_len != None and c_min > shortest_path_len:
#                        continue
                    q_new_idx += 1
                    V[q_new_idx] = q_new
                    E[q_new_idx] = q_min_idx
                    print "len(V) len(E)", len(V), len(E)

                    if debug:
                        edge_ids[(q_min_idx, q_new_idx)] = edge_id
                        self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_min_idx][0], V[q_min_idx][1], V[q_min_idx][2]), PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), edge_id, 0, 1, 0, frame='world', namespace='edges', scale=0.01)
                        edge_id += 1

                    time.append( rospy.Time.now() )

                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if Cost(V, E, q_new_idx) + CostLine(q_new, q_near) < Cost(V, E, q_near_idx) and CollisionFree(openrave, q_new, q_near, dof_indices):
                            # Parent()
                            q_parent_idx = E[q_near_idx]
                            print "rem: %s  %s"%(q_parent_idx, q_near_idx)

                            edge_id_del = None
                            if debug:
                                edge_id_del = edge_ids[(q_parent_idx, q_near_idx)]
                            E[q_near_idx] = q_new_idx
                            if debug:
                                edge_ids[(q_new_idx, q_near_idx)] = edge_id_del
                                self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), PyKDL.Vector(V[q_near_idx][0], V[q_near_idx][1], V[q_near_idx][2]), edge_id_del, 0, 1, 0, frame='world', namespace='edges', scale=0.01)

                    time.append( rospy.Time.now() )

                    vis = getVisibility(openrave, vis_bodies, q=q_new, dof_indices=dof_indices)
                    time.append( rospy.Time.now() )

                    if vis > best_vis:
                        goal_V_ids = [q_new_idx]
                        best_vis = vis
                        best_q = q_new
                        best_q_idx = q_new_idx
                        shortest_path_len = Cost(V, E, q_new_idx)

                        if shortest_path_len > rrt_switch:
                            self.phs = None
                        else:
                            self.phs = self.ProlateHyperspheroid(len(dof_indices), np.array(q_init), np.array(best_q), CostLine(q_init, best_q))
#                            if shortest_path_len > 2.0:
#                                self.phs.setTransverseDiameter(2.0)
#                            else:
                            self.phs.setTransverseDiameter(shortest_path_len)

                        self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                        DrawPath(V, E, q_new_idx)
                        print " %s  vis %s, shortest_path: %s"%(k, vis, shortest_path_len)
                    elif vis == best_vis and best_vis > 0:
                        goal_V_ids.append(q_new_idx)

                        if shortest_path_len == None or shortest_path_len > Cost(V, E, q_new_idx):
                            best_q = q_new
                            best_q_idx = q_new_idx
                            shortest_path_len_old = shortest_path_len
                            shortest_path_len = Cost(V, E, q_new_idx)

                            if shortest_path_len > rrt_switch:
                                self.phs = None
                            else:
                                self.phs = self.ProlateHyperspheroid(len(dof_indices), np.array(q_init), np.array(best_q), CostLine(q_init, best_q))
#                                if shortest_path_len > 2.0:
#                                    self.phs.setTransverseDiameter(2.0)
#                                else:
                                self.phs.setTransverseDiameter(shortest_path_len)

                            self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                            DrawPath(V, E, q_new_idx)
                            print " %s  vis %s, shortest_path: %s   delta: %s"%(k, vis, shortest_path_len, shortest_path_len_old - shortest_path_len)

#                    # process the tree nodes
#                    changes = 0
#                    costs_V = []
#                    for vi in V:
#                        cost = Cost(V, E, vi)
#                        costs_V.append( (cost, vi) )
#                    sorted_costs_V = sorted(costs_V, key=operator.itemgetter(0))
#                    for idx1 in range(1, len(sorted_costs_V)):
#                        cost, vi = sorted_costs_V[idx1]
#                        parent_idx = E[idx1]
#                        for idx2 in range(idx1):
#                            new_cost = Cost(V, E, idx2) + CostLine(V[idx2], V[idx1])
#                            if cost > new_cost and CollisionFree(openrave, V[idx2], V[idx1], dof_indices):
#                                if debug:
#                                    edge_id_del = edge_ids[(parent_idx, idx1)]
#                                E[idx1] = idx2
#                                if debug:
#                                    edge_ids[(idx2, idx1)] = edge_id_del
#                                    self.pub_marker.publishVectorMarker(PyKDL.Vector(V[idx2][0], V[idx2][1], V[idx2][2]), PyKDL.Vector(V[idx1][0], V[idx1][1], V[idx1][2]), edge_id_del, 0, 1, 0, frame='world', namespace='edges', scale=0.01)
#                                changes += 1
#                    print "changes", changes

                    for goal_idx in goal_V_ids:
                        goal_cost = Cost(V, E, goal_idx)
                        if shortest_path_len > goal_cost:
                            best_q_idx = goal_idx
                            best_q = V[best_q_idx]
                            shortest_path_len =  goal_cost
                            print "*********** found better goal:", shortest_path_len
                            self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                            DrawPath(V, E, best_q_idx)
                            if shortest_path_len > rrt_switch:
                                self.phs = None
                            else:
                                self.phs = self.ProlateHyperspheroid(len(dof_indices), np.array(q_init), np.array(best_q), CostLine(q_init, best_q))
#                                if shortest_path_len > 2.0:
#                                    self.phs.setTransverseDiameter(2.0)
#                                else:
                                self.phs.setTransverseDiameter(shortest_path_len)

                    if True and shortest_path_len_prev != shortest_path_len:
                        rem_nodes = []
                        for vi in V:
                            if CostLine(self.q_init, V[vi]) > shortest_path_len:
#                            if Cost(V, E, vi) > shortest_path_len:
                                rem_nodes.append(vi)
                        print "removing nodes:", len(rem_nodes)
                        for vi in rem_nodes:
                            del V[vi]
                            self.pub_marker.eraseMarkers(edge_ids[(E[vi], vi)], edge_ids[(E[vi], vi)]+1, frame_id='torso_base', namespace='edges')
                            del E[vi]
                            if vi in goal_V_ids:
                                goal_V_ids.remove(vi)
                        shortest_path_len_prev = shortest_path_len
                        while True:
                          orphan_nodes = []
                          for vi in E:
                              if (not vi in rem_nodes) and (E[vi] in rem_nodes):
                                  orphan_nodes.append(vi)
                          print "orphan_nodes", len(orphan_nodes)
                          rem_nodes = orphan_nodes
                          if len(rem_nodes) == 0:
                              break
#                          print "removing nodes:", len(rem_nodes)
                          for vi in rem_nodes:
                              del V[vi]
                              self.pub_marker.eraseMarkers(edge_ids[(E[vi], vi)], edge_ids[(E[vi], vi)]+1, frame_id='torso_base', namespace='edges')
                              del E[vi]
                              if vi in goal_V_ids:
                                  goal_V_ids.remove(vi)


#                    if shortest_path_len != None and shortest_path_len < 3.2:
#                        break
                    if self.stop_planning:
                        break
                    time.append( rospy.Time.now() )

#                    t_diff = []
#                    for ti in range(len(time)-1):
#                        t_diff.append( (time[ti+1] - time[ti]).to_sec() )
#                    print "noncol", t_diff
#                else:
#                    t_diff = []
#                    for ti in range(len(time)-1):
#                        t_diff.append( (time[ti+1] - time[ti]).to_sec() )
#                    print "col   ", t_diff

            path = GetPath(E, best_q_idx)
            print path

            traj = []
#            for q_idx in path:
#                traj.append(V[q_idx])
            for i in range(len(path)-1):
                q_idx1 = path[i]
                q_idx2 = path[i+1]
                for f in np.linspace(0.0, 1.0, 40):
                    traj.append( V[q_idx1] * (1.0 - f) + V[q_idx2] * f )

            while True:
                raw_input(".")
                openrave.showTrajectory(dof_names, 10.0, traj)
            
#            for q in V_vis:
#                openrave.robot_rave.SetDOFValues(q, dof_indices)
#                openrave.env.UpdatePublishedBodies()
#                raw_input(".")

        self.stop_planning = False
        thread.start_new_thread(self.inputThread, (None,))

        RRTstar(openrave, dof_limits, dof_indices)

    def spin(self):

        # TEST: phs
        if False:
            phs = self.ProlateHyperspheroid(3, np.array([0,0,0]), np.array([1,1,1]))
            phs.setTransverseDiameter(2.0)

            m_id = 0
            for i in range(1000):
                sphere = self.uniformInBall(1.0, 3)
                # Transform to the PHS
                vec = phs.transform(sphere)
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(sphere[0],sphere[1],sphere[2]-0.5), m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(vec[0],vec[1],vec[2]), m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                rospy.sleep(0.01)
#            print sphere
#            print vec

            exit(0)

        #
        # Initialise Openrave
        #

#        a1 = np.array([1,2,3])
#        a2 = np.array([3,2,1])
#        print a1*3
#        exit(0)

        rospack = rospkg.RosPack()
        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenraveURDF(env_file=rospack.get_path('velma_scripts') + '/data/key/vis_test.env.xml')
        openrave.readRobot(xacro_uri=rospack.get_path('velma_description') + '/robots/velma.urdf.xacro', srdf_uri=rospack.get_path('velma_description') + '/robots/velma.srdf')

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

        raw_input("Press ENTER to continue...")
        
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


