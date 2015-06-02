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
            ETA = 10.0/180.0*math.pi
            gamma = 0.1
            d = len(dof_indices)
            
            def Distance(q1, q2):
                q_diff = q1 - q2
                return math.sqrt( np.dot(q_diff, q_diff) )

            def SampleFree(openrave, dof_indices, dof_limits, best_q, start_q, shortest_path_len):
                if best_q != None and random.uniform(0,1) < 0.05:
                    return best_q
                while True:
                    q_rand_list = []
                    for i in range(len(dof_limits)):
                        q_rand_list.append( random.uniform(dof_limits[i][0]+0.01, dof_limits[i][1]-0.01) )
                    q_rand = np.array(q_rand_list)
                    if best_q != None:
                        dist = Distance(q_rand, start_q) + Distance(q_rand, best_q)
                        if dist > shortest_path_len:
                            continue
                    if isStateValid(openrave, q_rand, dof_indices):
                        return q_rand

            def Nearest(V, q):
                q_near = None
                for vi in range(len(V)):
                    dist = Distance(q_rand, V[vi])
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
                for vi in range(len(V)):
                    diff = V[vi]-q
                    dist = math.sqrt( np.dot(diff, diff) )
                    if dist < near_dist:
                        result.append(vi)
                return result

            def CostLine(q1, q2):
                return Distance(q1, q2)

            def Cost(V, E, q_idx):
                if not q_idx in E:
                    return 0.0
                parent_idx = E[q_idx]
                cost = CostLine(V[parent_idx], V[q_idx]) + Cost(V, E, parent_idx)
                return cost

            def CollisionFree(openrave, q1, q2, dof_indices):
                dist = Distance(q1,q2)
                steps = int(dist / (10.0/180.0*math.pi))
                if steps < 1:
                    steps = 1
                for i in range(steps):
                    t = (float(i)+1.0)/float(steps)
                    if not isStateValid(openrave, q1 * (1.0-t) + q2 * t, dof_indices):
                        return False
                return True

            def GetPath(V, E, q_idx):
                if not q_idx in E:
                    return [q_idx]
                parent_idx = E[q_idx]
                q_list = GetPath(V, E, parent_idx)
                q_list.append(q_idx)
                return q_list

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

            shortest_path_len = None
            best_vis = 0
            best_q = None
            best_q_idx = None
            V_vis = []
            V = []
            E = {}
            q_init = openrave.robot_rave.GetDOFValues(dof_indices)
            V.append(np.array(q_init))
            for k in range(4500):
                q_rand = SampleFree(openrave, dof_indices, dof_limits, best_q, q_init, shortest_path_len)
                q_nearest_idx = Nearest(V, q_rand)
                q_nearest = V[q_nearest_idx]
                q_new = Steer(q_nearest, q_rand)
                if CollisionFree(openrave, q_nearest, q_new, dof_indices):
#                    near_dist = gamma*math.log(len(V))/len(V)
                    near_dist = min(gamma*math.pow(math.log(len(V))/len(V), 1.0/d), ETA)
#                    near_dist = min(gamma*math.log(len(V))/len(V), ETA)
                    q_near_idx_list = Near(V, q_new, near_dist)
                    V.append(q_new)
                    q_new_idx = len(V) - 1
                    q_min_idx = q_nearest_idx
                    c_min = Cost(V, E, q_nearest_idx) + CostLine(q_nearest, q_new)
                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if CollisionFree(openrave, q_near, q_new, dof_indices) and Cost(V, E, q_near_idx) + CostLine(q_near, q_new) < c_min:
                            q_min_idx = q_near_idx
                            c_min = Cost(V, E, q_near_idx) + CostLine(q_near, q_new)

                    E[q_new_idx] = q_min_idx

                    if debug:
                        edge_ids[(q_min_idx, q_new_idx)] = edge_id
                        self.pub_marker.publishVectorMarker(PyKDL.Vector(V[q_min_idx][0], V[q_min_idx][1], V[q_min_idx][2]), PyKDL.Vector(V[q_new_idx][0], V[q_new_idx][1], V[q_new_idx][2]), edge_id, 0, 1, 0, frame='world', namespace='edges', scale=0.01)
                        edge_id += 1

                    for q_near_idx in q_near_idx_list:
                        q_near = V[q_near_idx]
                        if CollisionFree(openrave, q_new, q_near, dof_indices) and Cost(V, E, q_new_idx) + CostLine(q_new, q_near) < Cost(V, E, q_near_idx):
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

                    vis = getVisibility(openrave, vis_bodies, q=q_new, dof_indices=dof_indices)
                    if vis > best_vis:
                        best_vis = vis
                        best_q = q_new
                        best_q_idx = q_new_idx
                        shortest_path_len = Cost(V, E, q_new_idx)
                        self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                        DrawPath(V, E, q_new_idx)
                        print " %s  vis %s, shortest_path: %s"%(k, vis, shortest_path_len)
                    elif vis == best_vis and best_vis > 0:
                        if shortest_path_len == None or shortest_path_len > Cost(V, E, q_new_idx):
                            best_q = q_new
                            best_q_idx = q_new_idx
                            shortest_path_len = Cost(V, E, q_new_idx)
                            self.pub_marker.eraseMarkers(0, 200, frame_id='torso_base', namespace='shortest_path')
                            DrawPath(V, E, q_new_idx)
                        print " %s  vis %s, shortest_path: %s"%(k, vis, shortest_path_len)

            path = GetPath(V, E, best_q_idx)
            print path

            traj = []
            for q_idx in path:
                traj.append(V[q_idx])

            while True:
                raw_input(".")
                openrave.showTrajectory(dof_names, 10.0, traj)
            
#            for q in V_vis:
#                openrave.robot_rave.SetDOFValues(q, dof_indices)
#                openrave.env.UpdatePublishedBodies()
#                raw_input(".")

        RRTstar(openrave, dof_limits, dof_indices)

    def spin(self):

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

