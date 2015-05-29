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

#import ar_track_alvar_msgs.msg
#from ar_track_alvar_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
#from cartesian_trajectory_msgs.msg import *
from visualization_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import random
import PyKDL
import math
import numpy as np
import copy
from scipy import optimize
import scipy

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

import dijkstra
from subprocess import Popen, PIPE, STDOUT

import operator

import velma_fk_ik

class MarkerPublisher:
    def __init__(self):
        self.pub_marker = rospy.Publisher('/velma_markers', MarkerArray)

    def publishSinglePointMarker(self, pt, i, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=None):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = m_type
        marker.action = Marker.ADD
        if T != None:
            point = T*pt
            q = T.M.GetQuaternion()
            marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(q[0],q[1],q[2],q[3]) )
        else:
            marker.pose = Pose( Point(pt.x(),pt.y(),pt.z()), Quaternion(0,0,0,1) )
        marker.scale = scale
        marker.color = ColorRGBA(r,g,b,a)
        m.markers.append(marker)
        self.pub_marker.publish(m)
        return i+1

    def eraseMarkers(self, idx_from, idx_to, frame_id='torso_base', namespace='default'):
        m = MarkerArray()
        for idx in range(idx_from, idx_to):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = idx
            marker.action = Marker.DELETE
            m.markers.append(marker)
        if len(m.markers) > 0:
            self.pub_marker.publish(m)


    def publishMultiPointsMarker(self, pt, base_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=None):
        m = MarkerArray()
        ret_id = copy.copy(base_id)
        for i in range(0, len(pt)):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = ret_id
            ret_id += 1
            marker.type = m_type
            marker.action = Marker.ADD
            if T != None:
                point = T*pt[i]
                marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(0,0,0,1) )
            else:
                marker.pose = Pose( Point(pt[i].x(),pt[i].y(),pt[i].z()), Quaternion(0,0,0,1) )
            marker.scale = scale
            marker.color = ColorRGBA(r,g,b,0.5)
            m.markers.append(marker)
        self.pub_marker.publish(m)
        return ret_id

    def publishMultiPointsMarkerWithSize(self, pt, base_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.SPHERE, T=None):
        m = MarkerArray()
        ret_id = copy.copy(base_id)
        if T == None:
            T = PyKDL.Frame()
        for i in range(0, len(pt)):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = ret_id
            ret_id += 1
            marker.type = m_type
            marker.action = Marker.ADD
            point = T*pt[i][0]
            marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(0,0,0,1) )
            marker.scale = Vector3(pt[i][1], pt[i][1], pt[i][1])
            marker.color = ColorRGBA(r,g,b,0.5)
            m.markers.append(marker)
        self.pub_marker.publish(m)
        return ret_id

    def publishVectorMarker(self, v1, v2, i, r, g, b, frame='torso_base', namespace='default', scale=0.001):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points.append(Point(v1.x(), v1.y(), v1.z()))
        marker.points.append(Point(v2.x(), v2.y(), v2.z()))
        marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
        marker.scale = Vector3(scale, 2.0*scale, 0)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)
        return i+1

    def publishFrameMarker(self, T, base_id, scale=0.1, frame='torso_base', namespace='default'):
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(scale,0,0), base_id, 1, 0, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,scale,0), base_id+1, 0, 1, 0, frame, namespace)
        self.publishVectorMarker(T*PyKDL.Vector(), T*PyKDL.Vector(0,0,scale), base_id+2, 0, 0, 1, frame, namespace)
        return base_id+3

    def publishConstantMeshMarker(self, uri, base_id, r=1, g=0, b=0, scale=0.1, frame_id='torso_base', namespace='default', T=None):
        if T == None:
            T = PyKDL.Frame()
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = base_id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = uri #"package://pr2_description/meshes/base_v0/base.dae"
        marker.action = Marker.ADD
        point = T.p
        q = T.M.GetQuaternion()
        marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(q[0],q[1],q[2],q[3]) )
        marker.scale = Vector3(scale, scale, scale)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)
        return base_id+1

    def publishMeshMarker(self, mesh, base_id, r=1, g=0, b=0, scale=0.1, frame_id='torso_base', namespace='default', T=None):
        m = MarkerArray()
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = base_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        for f in mesh[1]:
            marker.points.append(Point(mesh[0][f[0]][0], mesh[0][f[0]][1], mesh[0][f[0]][2]))
            marker.points.append(Point(mesh[0][f[1]][0], mesh[0][f[1]][1], mesh[0][f[1]][2]))
            marker.points.append(Point(mesh[0][f[2]][0], mesh[0][f[2]][1], mesh[0][f[2]][2]))
        if T != None:
            point = T.p
            q = T.M.GetQuaternion()
            marker.pose = Pose( Point(point.x(),point.y(),point.z()), Quaternion(q[0],q[1],q[2],q[3]) )
        else:
            marker.pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
        marker.scale = Vector3(1.0, 1.0, 1.0)
        marker.color = ColorRGBA(r,g,b,0.5)
        m.markers.append(marker)
        self.pub_marker.publish(m)
        return base_id+1

def getAngle(v1, v2):
    return math.atan2((v1*v2).Norm(), PyKDL.dot(v1,v2))

def contactToWrenches(pos, normal, friction, Nconepoints):
            wrenches = []
            fdeltaang = 2.0*math.pi/float(Nconepoints)
            nz = normal
            if abs(nz.z()) < 0.7:
                nx = PyKDL.Vector(0,0,1)
            elif abs(nz.y()) < 0.7:
                nx = PyKDL.Vector(0,1,0)
            else:
                nx = PyKDL.Vector(1,0,0)
            ny = nz * nx
            nx = ny * nz
            nx.Normalize()
            ny.Normalize()
            nz.Normalize()
            R_n = PyKDL.Frame(PyKDL.Rotation(nx,ny,nz))
            fangle = 0.0
            for cp in range(Nconepoints):
                nn = R_n * PyKDL.Frame(PyKDL.Rotation.RotZ(fangle)) * PyKDL.Vector(friction,0,1)
                fangle += fdeltaang
                tr = pos * nn
                wr = PyKDL.Wrench(nn,tr)
                wrenches.append([wr[0], wr[1], wr[2], wr[3], wr[4], wr[5]])
            return wrenches

#% by Tolga Birdal
#% Q is an Mx4 matrix of quaternions. Qavg is the average quaternion
#% Based on 
#% Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
#% "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
#% no. 4 (2007): 1193-1197.
#function [Qavg]=avg_quaternion_markley(Q)

#% Form the symmetric accumulator matrix
#A = zeros(4,4);
#M = size(Q,1);

#for i=1:M
#    q = Q(i,:)';
#    A = q*q'+A; % rank 1 update
#end

#% scale
#A=(1.0/M)*A;

#% Get the eigenvector corresponding to largest eigen value
#[Qavg, Eval] = eigs(A,1);

#end

# from http://www.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging-quaternions
# by Tolga Birdal
# Q is an Mx4 matrix of quaternions. Qavg is the average quaternion
# Based on 
# Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
# "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
# no. 4 (2007): 1193-1197.

# Q.shape == (M, 4)
#function [Qavg]=avg_quaternion_markley(Q)
def avg_quaternion_markley(Q):

    #% Form the symmetric accumulator matrix
    #A = zeros(4,4);
    A = np.zeros((4,4))
    #M = size(Q,1);
    M = Q.shape[0]

    #for i=1:M
    for i in range(0,M):
    #    q = Q(i,:)';
        q = np.transpose(Q[i])
    #    A = q*q'+A; % rank 1 update
#        print "q*np.transpose(q)"
#        print q*np.transpose(q)
        A = q*np.transpose(q) + A; # rank 1 update
#        print "q=%s"%(q)
#        print "shape(q*qT)=%s"%((q*np.transpose(q)).shape)
#        print "q*qT=%s"%(q*np.transpose(q))
#        print "A=%s"%(A)
    #end

#    print "M=%s"%(M)
    #% scale
    #A=(1.0/M)*A;
    A=(1.0/M)*A

#    print "A"
    print A
    #% Get the eigenvector corresponding to largest eigen value
    #[Qavg, Eval] = eigs(A,1);
#    ei = numpy.linalg.eig(A)
    ei = scipy.sparse.linalg.eigsh(A,1)

    return ei


def generateNormalsSphere(angle, x_positive=None, y_positive=None, z_positive=None):
    if angle <= 0:
        return None
    v_approach = []
    i = 0
    steps_alpha = int(math.pi/angle)
    if steps_alpha < 2:
        steps_alpha = 2
    for alpha in np.linspace(-90.0/180.0*math.pi, 90.0/180.0*math.pi, steps_alpha):
        max_steps_beta = (360.0/180.0*math.pi)/angle
        steps = int(math.cos(alpha)*max_steps_beta)
        if steps < 1:
            steps = 1
        beta_d = 360.0/180.0*math.pi/steps
        for beta in np.arange(0.0, 360.0/180.0*math.pi, beta_d):
            pt = PyKDL.Vector(math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta), math.sin(alpha))
            if x_positive != None:
                if x_positive and pt.x() < 0:
                    continue
                if not x_positive and pt.x() > 0:
                    continue
            if y_positive != None:
                if y_positive and pt.y() < 0:
                    continue
                if not y_positive and pt.y() > 0:
                    continue
            if z_positive != None:
                if z_positive and pt.z() < 0:
                    continue
                if not z_positive and pt.z() > 0:
                    continue
            v_approach.append(pt)
    return v_approach

def generateFramesForNormals(angle, normals):
    steps = int((360.0/180.0*math.pi)/angle)
    if steps < 2:
        steps = 2
    angle_d = 360.0/180.0*math.pi/steps
    frames = []
    for z in normals:
        if abs(z.z()) < 0.7:
            y = PyKDL.Vector(0,0,1)
        else:
            y = PyKDL.Vector(0,1,0)
        x = y * z
        y = z * x
        x.Normalize()
        y.Normalize()
        for angle in np.arange(0.0, 359.9/180.0*math.pi, angle_d):
            frames.append(PyKDL.Frame(PyKDL.Rotation(x,y,z)) * PyKDL.Frame(PyKDL.Rotation.RotZ(angle)))
#            print angle/math.pi*180.0

    return frames

def pointInTriangle(A, B, C, P):
    # Compute vectors        
    v0 = [C[0] - A[0], C[1] - A[1]]
    v1 = [B[0] - A[0], B[1] - A[1]]
    v2 = [P[0] - A[0], P[1] - A[1]]

    # Compute dot products
    dot00 = v0[0]*v0[0] + v0[1]*v0[1]
    dot01 = v0[0]*v1[0] + v0[1]*v1[1]
    dot02 = v0[0]*v2[0] + v0[1]*v2[1]
    dot11 = v1[0]*v1[0] + v1[1]*v1[1]
    dot12 = v1[0]*v2[0] + v1[1]*v2[1]

    if dot00 * dot11 - dot01 * dot01 == 0.0:
        return False
    # Compute barycentric coordinates
    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    v = (dot00 * dot12 - dot01 * dot02) * invDenom

    # Check if point is in triangle
    return (u >= 0) and (v >= 0) and (u + v < 1)

def sampleMesh(vertices, indices, sample_dist, pt_list, radius, return_normals=False):
        points = []
        points_normals = []
        for s2 in pt_list:
            for face in indices:
                A = vertices[face[0]]
                B = vertices[face[1]]
                C = vertices[face[2]]
                pt_a = PyKDL.Vector(A[0],A[1],A[2])
                pt_b = PyKDL.Vector(B[0],B[1],B[2])
                pt_c = PyKDL.Vector(C[0],C[1],C[2])
                v0 = pt_b - pt_a
                v1 = pt_c - pt_a
                # calculate face normal
                normal = v0 * v1
                normal.Normalize()
                # calculate distance between the sphere center and the face
                s_dist = PyKDL.dot(normal, s2) - PyKDL.dot(normal, pt_a)
                # if the distance is greater than radius, ignore the face
                if abs(s_dist) > radius:
                    continue
                # calculate the projection of the sphere center to the face
                s_on = s2 - s_dist * normal
                # calculate the radius of circle being common part of sphere and face
                radius2_square = radius * radius - s_dist * s_dist
                if radius2_square < 0.0:   # in case of numerical error
                    radius2_square = 0.0
                radius2 = math.sqrt(radius2_square)
                # TODO: check only the face's area of interest
                v0p = v0 * v1 * v0
                v0p.Normalize()
                v1p = v1 * v0 * v1
                v1p.Normalize()
                d0 = PyKDL.dot(v0p, (s_on-pt_a))
                d1 = PyKDL.dot(v1p, (s_on-pt_a))
                n0 = v0.Norm()
                steps0 = int(n0/sample_dist)
                if steps0 < 1:
                    steps0 = 1
                step_len0 = n0/steps0
                n1 = v1.Norm()
                angle = getAngle(v0,v1)
                h = n1*math.sin(angle)
                steps1 = int(h/sample_dist)
                if steps1 < 1:
                    steps1 = 1
                step_len1 = h/steps1

                x0_min = (d1-radius)/math.sin(angle)
                x0_max = (d1+radius)/math.sin(angle)
                x1_min = d0-radius
                x1_max = d0+radius

                x0_min2 = max(step_len0/2.0, x0_min)
                x0_max2 = min(n0, x0_max)
                if x0_min2 >= x0_max2:
                    continue
                for x0 in np.arange(x0_min2, x0_max2, step_len0):
                    x1_min2 = max(step_len1/2.0, x1_min)
                    x1_max2 = min(h*(1.0-x0/n0), x1_max)
                    if x1_min2 >= x1_max2:
                        continue
                    for x1 in np.arange(x1_min2, x1_max2, step_len1):
                        point = pt_a + v0*(x0/n0) + v1*(x1/h)
                        in_range = False
                        for s2 in pt_list:
                            if (point-s2).Norm() < radius:
                                in_range = True
                                break
                        if in_range:
                            points.append(point)
                            points_normals.append(normal)
        if len(pt_list) == 1:
            if return_normals:
                return points, points_normals
            else:
                return points
        min_dists = []
        min_dists_p_index = []
        for s in pt_list:
            min_dists.append(1000000.0)
            min_dists_p_index.append(None)
        i = 0
        for s in pt_list:
            p_index = 0
            for p in points:
                d = (s-p).Norm()
                if d < min_dists[i]:
                    min_dists[i] = d
                    min_dists_p_index[i] = p_index
                p_index += 1
            i += 1
        first_contact_index = None
        for i in range(0, len(pt_list)):
            if min_dists[i] < sample_dist*2.0:
                first_contact_index = i
                break
        if first_contact_index == None:
            print "first_contact_index == None"
            return points
        init_pt = points[min_dists_p_index[first_contact_index]]
        points_ret = []
        list_to_check = []
        list_check_from = []
        for i in range(0, len(points)):
            if (init_pt-points[i]).Norm() > radius:
                continue
            if i == min_dists_p_index[first_contact_index]:
                list_check_from.append(points[i])
            else:
                list_to_check.append(points[i])
        points_ret = []
        added_point = True
        iteration = 0
        while added_point:
            added_point = False
            list_close = []
            list_far = []
            for p in list_to_check:
                added_p = False
                for check_from in list_check_from:
                    if (check_from-p).Norm() < sample_dist*2.0:
                        added_point = True
                        added_p = True
                        list_close.append(p)
                        break
                if not added_p:
                    list_far.append(p)
            points_ret += list_check_from
            list_to_check = copy.deepcopy(list_far)
            list_check_from = copy.deepcopy(list_close)
            iteration += 1
        return points_ret

def sampleMesh_old(vertices, indices, sample_dist, pt_list, radius):
        points = []
        for face in indices:
            A = vertices[face[0]]
            B = vertices[face[1]]
            C = vertices[face[2]]
            pt_a = PyKDL.Vector(A[0],A[1],A[2])
            pt_b = PyKDL.Vector(B[0],B[1],B[2])
            pt_c = PyKDL.Vector(C[0],C[1],C[2])
            v0 = pt_b - pt_a
            n0 = v0.Norm()
            steps0 = int(n0/sample_dist)
            if steps0 < 1:
                steps0 = 1
            step_len0 = n0/steps0
            v1 = pt_c - pt_a
            n1 = v1.Norm()
            angle = getAngle(v0,v1)
            h = n1*math.sin(angle)
            steps1 = int(h/sample_dist)
            if steps1 < 1:
                steps1 = 1
            step_len1 = h/steps1
            x0 = step_len0/2.0
            while x0 < n0:
                x1 = step_len1/2.0
                while x1 < h*(1.0-x0/n0):
                    point = pt_a + v0*(x0/n0) + v1*(x1/h)
                    in_range = False
                    for s2 in pt_list:
                        if (point-s2).Norm() < radius:
                            in_range = True
                            break
                    if in_range:
                        points.append(point)
                    x1 += step_len1
                x0 += step_len0
        if len(pt_list) == 1:
            return points
        min_dists = []
        min_dists_p_index = []
        for s in pt_list:
            min_dists.append(1000000.0)
            min_dists_p_index.append(None)
        i = 0
        for s in pt_list:
            p_index = 0
            for p in points:
                d = (s-p).Norm()
                if d < min_dists[i]:
                    min_dists[i] = d
                    min_dists_p_index[i] = p_index
                p_index += 1
            i += 1
        first_contact_index = None
        for i in range(0, len(pt_list)):
            if min_dists[i] < sample_dist*2.0:
                first_contact_index = i
                break
        if first_contact_index == None:
            print "first_contact_index == None"
            return points
        init_pt = points[min_dists_p_index[first_contact_index]]
        points_ret = []
        list_to_check = []
        list_check_from = []
        for i in range(0, len(points)):
            if (init_pt-points[i]).Norm() > radius:
                continue
            if i == min_dists_p_index[first_contact_index]:
                list_check_from.append(points[i])
            else:
                list_to_check.append(points[i])
        points_ret = []
        added_point = True
        iteration = 0
        while added_point:
            added_point = False
            list_close = []
            list_far = []
            for p in list_to_check:
                added_p = False
                for check_from in list_check_from:
                    if (check_from-p).Norm() < sample_dist*2.0:
                        added_point = True
                        added_p = True
                        list_close.append(p)
                        break
                if not added_p:
                    list_far.append(p)
            points_ret += list_check_from
            list_to_check = copy.deepcopy(list_far)
            list_check_from = copy.deepcopy(list_close)
            iteration += 1
        return points_ret

def estPlane(points_in):
    mean_pt = PyKDL.Vector()
    for p in points_in:
        mean_pt += p
    mean_pt *= (1.0/len(points_in))

    points = []
    for p in points_in:
        points.append(p-mean_pt)

    def calc_R(xa, ya):
        ret = []
        """ calculate the minimum distance of each contact point from jar surface pt """
        n = PyKDL.Frame(PyKDL.Rotation.RotX(xa)) * PyKDL.Frame(PyKDL.Rotation.RotY(ya)) * PyKDL.Vector(0,0,1)
        for p in points:
            ret.append(PyKDL.dot(n,p))
        return numpy.array(ret)
        
    def f_2(c):
        """ calculate the algebraic distance between each contact point and jar surface pt """
        Di = calc_R(*c)
        return Di

    angles_estimate = 0.0, 0.0
    angles_2, ier = optimize.leastsq(f_2, angles_estimate, maxfev = 1000)
    n = PyKDL.Frame(PyKDL.Rotation.RotX(angles_2[0])) * PyKDL.Frame(PyKDL.Rotation.RotY(angles_2[1])) * PyKDL.Vector(0,0,1)

    nz = n
    if math.fabs(n.x()) < 0.9:
        nx = PyKDL.Vector(1,0,0)
    else:
        nx = PyKDL.Vector(0,1,0)

    ny = nz*nx
    nx = ny*nz
    nx.Normalize()
    ny.Normalize()
    nz.Normalize()

    return PyKDL.Frame(PyKDL.Rotation(nx,ny,nz), mean_pt)

def reducePointsSet(points_in, min_dist):
    points_out = []
    for pt in points_in:
        similar = False
        for c in points_out:
            if (c[0]-pt).Norm() < min_dist:
                similar = True
                c.append(pt)
                break
        if not similar:
            points_out.append( [pt] )

    for idx in range(len(points_out)):
        mean_pt = PyKDL.Vector()
        for pt in points_out[idx]:
            mean_pt += pt
        mean_pt = mean_pt / len(points_out[idx])
        points_out[idx] = mean_pt
    return points_out

def sampleMeshUnitTest(vertices, indices, pub_marker):
    points = sampleMesh(vertices, indices, 0.002, [PyKDL.Vector(0.00,0,0.00)], 0.04)
    print len(points)
    m_id = 0
    m_id = pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
    raw_input("Press Enter to continue...")
    rospy.sleep(5.0)
    pt_list = []
    for i in range(0, 20):
        pt_list.append(PyKDL.Vector((1.0*i/20.0)*0.1-0.05, 0, 0))
    points = sampleMesh(vertices, indices, 0.002, pt_list, 0.01)
    print len(points)
    m_id = 0
    m_id = pub_marker.publishMultiPointsMarker(points, m_id, r=1, g=0, b=0, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.001, 0.001, 0.001))
    rospy.sleep(1.0)
    fr = estPlane(points)
    m_id = pub_marker.publishFrameMarker(fr, m_id)
    rospy.sleep(1.0)

def meanOrientation(T, weights=None):
    R = []
    for t in T:
        R.append( PyKDL.Frame(copy.deepcopy(t.M)) )

    if weights == None:
        wg = list( np.ones(len(T)) )
    else:
        wg = weights

    wg_sum = sum(wg)

    def calc_R(rx, ry, rz):
        R_mean = PyKDL.Frame(PyKDL.Rotation.EulerZYX(rx, ry, rz))
        diff = []
        for r in R:
            diff.append(PyKDL.diff( R_mean, r ))
        ret = []
        for idx in range(0, len(diff)):
            rot_err = diff[idx].rot.Norm()
            ret.append(rot_err * wg[idx] / wg_sum)
        #ret = [math.fabs(d.rot.x()) for d in diff] + [math.fabs(d.rot.y()) for d in diff] + [math.fabs(d.rot.z()) for d in diff]
        return ret
    def f_2(c):
        """ calculate the algebraic distance between each contact point and jar surface pt """
        Di = calc_R(*c)
        return Di
    def sumf_2(p):
        return math.fsum(np.array(f_2(p))**2)
    angle_estimate = R[0].M.GetEulerZYX()
#    angle_2, ier = optimize.leastsq(f_2, angle_estimate, maxfev = 10000)
    # least squares with constraints
    angle_2 = optimize.fmin_slsqp(sumf_2, angle_estimate, bounds=[(-math.pi, math.pi),(-math.pi, math.pi),(-math.pi, math.pi)], iprint=0)
    score = calc_R(angle_2[0],angle_2[1],angle_2[2])
    score_v = 0.0
    for s in score:
        score_v += s*s
    return [score_v, PyKDL.Frame(PyKDL.Rotation.EulerZYX(angle_2[0],angle_2[1],angle_2[2]))]

def meanPosition(T, weights=None):
    if weights == None:
        wg = list( np.ones(len(T)) )
    else:
        wg = weights
    wg_sum = sum(wg)
    mean_p = PyKDL.Vector()
    for idx in range(0, len(T)):
        mean_p += T[idx].p * wg[idx] / wg_sum
    return mean_p

# determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs.
def point_inside_polygon(x,y,poly):
    n = len(poly)
    inside =False
    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside

def getQHull(points):
    stdin_str = "3\n" + str(len(points)) + "\n"
    for p in points:
        stdin_str += str(p[0]) + " " + str(p[1]) + " " + str(p[2]) + "\n"

    p = Popen(['qconvex', 'Qt', 'i'], stdout=PIPE, stdin=PIPE, stderr=STDOUT)
    stdout_str = p.communicate(input=stdin_str)[0]

    faces = []
    values = stdout_str.split()
    if (len(values)-1)/3 != int(values[0]):
        print "getQHull error: %s %s"%((len(values)-1)/3, int(values[0]))
        return None
    for idx in range(1, len(values), 3):
        faces.append([int(values[idx+0]), int(values[idx+1]), int(values[idx+2])])

    # reduce the vertices set and remap indices
    v_map = {}
    v_idx = 0
    vertices = []
    faces_new = []
    for f in faces:
        face_new = []
        for i in range(0, 3):
            if f[i] in v_map:
                face_new.append(v_map[f[i]])
            else:
                v_map[f[i]] = v_idx
                vertices.append(points[f[i]])
                face_new.append(v_map[f[i]])
                v_idx += 1
        faces_new.append(face_new)
    return vertices, faces_new

def qhullDist(mesh1, mesh2):
    return None

def pointInMesh(vertices, faces, point):
    pos = 0
    for f in faces:
        A = vertices[f[0]]
        B = vertices[f[1]]
        C = vertices[f[2]]
        a = PyKDL.Vector(A[0], A[1], A[2])
        b = PyKDL.Vector(B[0], B[1], B[2])
        c = PyKDL.Vector(C[0], C[1], C[2])

        n = (b-a) * (c-a)
        n.Normalize()
        if n.z() == 0.0:
            continue

        if pointInTriangle( [a.x(), a.y()], [b.x(), b.y()], [c.x(), c.y()], [point.x(), point.y()] ):
            d = -PyKDL.dot(a, n)
            z = -(n.x() * point.x() + n.y() * point.y() + d)/n.z()
            if z > point.z():
                pos += 1

    if pos % 2 == 0:
        return False
    return True            

def getMeshBB(vertices, faces):
    minv = [None, None, None]
    maxv = [None, None, None]
    for v in vertices:
        for i in range(0, 3):
            if minv[i] == None or minv[i] > v[i]:
                minv[i] = v[i]
            if maxv[i] == None or maxv[i] < v[i]:
                maxv[i] = v[i]

    return minv, maxv

def generateComSamples(vertices, faces, count):
    bb = getMeshBB(vertices, faces)
    points = sampleMesh(vertices, faces, 0.01, [PyKDL.Vector()], 10.0)
    qhull = getQHull(points)
    qvertices, qfaces = qhull
    com_samples = []
    dist_add = 0.0
    for i in range(0,count):
        pt = PyKDL.Vector(random.uniform(bb[0][0]-dist_add, bb[1][0]+dist_add), random.uniform(bb[0][1]-dist_add, bb[1][1]+dist_add), random.uniform(bb[0][2]-dist_add, bb[1][2]+dist_add))
        if pointInMesh(qvertices, qfaces, pt):
            com_samples.append( pt )
    return com_samples

def comSamplesUnitTest(openrave, pub_marker, object_name):
    vertices, faces = openrave.getMesh(object_name)
    com = generateComSamples(vertices, faces, 2000)
    m_id = 0
    for pt in com:
        m_id = pub_marker.publishSinglePointMarker(pt, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=None)
        rospy.sleep(0.01)

def updateCom(T_B_O, T_B_O_2, contacts_O, com_pt, com_weights, m_id=0, pub_marker=None):
    diff_B = PyKDL.diff(T_B_O, T_B_O_2)
    up_v_B = PyKDL.Vector(0,0,1)
    n_v_B = diff_B.rot * up_v_B
    if pub_marker != None:
        m_id = pub_marker.publishVectorMarker(PyKDL.Vector(), diff_B.rot, m_id, r=0, g=1, b=0, frame='world', namespace='default', scale=0.01)
        m_id = pub_marker.publishVectorMarker(PyKDL.Vector(), n_v_B, m_id, r=1, g=1, b=1, frame='world', namespace='default', scale=0.01)

    n_O = PyKDL.Frame(T_B_O.Inverse().M) * n_v_B
    n_O_2 = PyKDL.Frame(T_B_O_2.Inverse().M) * n_v_B

    # get all com points that lies in the positive direction from the most negative contact point with respect to n_v in T_B_O and T_B_O_2
    c_dot_list = []
    c_dot_list_2 = []
    for c in contacts_O:
        dot = PyKDL.dot(c, n_O)
        c_dot_list.append(dot)
        dot_2 = PyKDL.dot(c, n_O_2)
        c_dot_list_2.append(dot_2)

    # get all com points that lies in the positive direction from given contact
    for contact_idx in range(0, len(c_dot_list)):
        for com_idx in range(0, len(com_pt)):
            c = com_pt[com_idx]
            dot = PyKDL.dot(c, n_O)
            dot_2 = PyKDL.dot(c, n_O_2)
            if dot > c_dot_list[contact_idx] and dot_2 > c_dot_list_2[contact_idx]:
                com_weights[com_idx] += 1

    return m_id

def updateCom2(T_B_O, T_B_O_2, contacts_O, com_pt, com_weights):
    diff = PyKDL.diff(T_B_O, T_B_O_2)

    up_v = PyKDL.Vector(0,0,1)
    n_v = diff.rot * up_v

    n_O = T_B_O.Inverse() * n_v
    n_O_2 = T_B_O_2.Inverse() * n_v

    # get all com points that lies in the positive direction from the most negative contact point with respect to n_v in T_B_O and T_B_O_2
    # get the minimum contact point for n_O
    min_c_dot = None
    for c in contacts_O:
        dot = PyKDL.dot(c, n_O)
        if min_c_dot == None or dot < min_c_dot:
            min_c_dot = dot
    # get all com points that lies in the positive direction from min_c_dot
    com_2_idx = []
    for c_idx in range(0, len(com_pt)):
        c = com_pt[c_idx]
        dot = PyKDL.dot(c, n_O)
        if dot > min_c_dot:
           com_2_idx.append(c_idx)

    # get the minimum contact point for n_O
    min_c_dot = None
    for c in contacts_O:
        dot = PyKDL.dot(c, n_O_2)
        if min_c_dot == None or dot < min_c_dot:
            min_c_dot = dot
    # get all com points that lies in the positive direction from min_c_dot
    com_3_idx = []
    for c_idx in com_2_idx:
        c = com_pt[c_idx]
        dot = PyKDL.dot(c, n_O_2)
        if dot > min_c_dot:
           com_3_idx.append(c_idx)

    for c_idx in range(0, len(com_pt)):
        com_weights[c_idx] -= 1

    for c_idx in com_3_idx:
        com_weights[c_idx] += 2

def updateComUnitTest(openrave, pub_marker, object_name):
            vertices, faces = openrave.getMesh(object_name)
            com = generateComSamples(vertices, faces, 2000)

            m_id = 0
            contacts_O = [
            PyKDL.Vector(-0.1, 0.03, 0),
            PyKDL.Vector(-0.15, 0.03, 0),
            PyKDL.Vector(-0.125, -0.03, 0)
            ]

            T_B_O = PyKDL.Frame(PyKDL.Vector(0.4,0,1.0)) * PyKDL.Frame(PyKDL.Rotation.RotZ(45.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(45.0/180.0*math.pi))
            m_id = pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O)
            m_id = pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O)

            T_B_O_2 = PyKDL.Frame(PyKDL.Vector(0,0,0.1)) * T_B_O * PyKDL.Frame(PyKDL.Rotation.RotY(30.0/180.0*math.pi))
            m_id = pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.354, 0.060, 0.060), T=T_B_O_2)
            m_id = pub_marker.publishMultiPointsMarker(contacts_O, m_id, r=1, g=0, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.002, 0.002, 0.002), T=T_B_O_2)

            com_weights = list(np.zeros(len(com)))
            m_id = updateCom(T_B_O, T_B_O_2, contacts_O, com, com_weights, m_id, pub_marker)

            for idx in range(0, len(com)):
                if int(com_weights[idx]) == 3:
                    m_id = pub_marker.publishSinglePointMarker(com[idx], m_id, r=1, g=0.9, b=0.9, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=T_B_O)
                elif int(com_weights[idx]) == 2:
                    m_id = pub_marker.publishSinglePointMarker(com[idx], m_id, r=1, g=0.5, b=0.5, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=T_B_O)
                elif int(com_weights[idx]) == 1:
                    m_id = pub_marker.publishSinglePointMarker(com[idx], m_id, r=1, g=0.2, b=0.2, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=T_B_O)
                elif int(com_weights[idx]) == 0:
                    m_id = pub_marker.publishSinglePointMarker(com[idx], m_id, r=0, g=1, b=0, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=T_B_O)
                else:
                    print com_weights[idx]
                    m_id = pub_marker.publishSinglePointMarker(com[idx], m_id, r=0, g=0, b=1, namespace='default', frame_id='world', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=T_B_O)
                rospy.sleep(0.01)

def projectPointToPlaneAlongVector(p, v, n, d, positive_only=True):
    pa = np.array(p)
    va = np.array(v)
    na = np.array(n)

    va_na = np.dot(va,-na)

    if abs(va_na) < 0.0000000001:
        return None

    if positive_only and va_na < 0:
        return None

    r = pa + ( va * ( (np.dot(na,pa)+d)/va_na ) )
    return r

class VelmaSolvers:
    def __init__(self):
        self.fk_solver = velma_fk_ik.VelmaFkIkSolver()

        self.robot = URDF.from_parameter_server()
        self.mimic_joints_map = {}
        self.joint_name_limit_map = {}

        joint_idx_ros = 0
        for i in range(len(self.robot.joints)):
            joint = self.robot.joints[i]
            
            if joint.joint_type == "fixed":
                continue
            if joint.mimic != None:
                self.mimic_joints_map[joint.name] = joint.mimic

            self.joint_name_limit_map[joint.name] = joint.limit

            joint_idx_ros += 1

    def updateMimicJoints(self, js_pos):
        for joint_name in self.mimic_joints_map:
            mimic_name = self.mimic_joints_map[joint_name].joint
            js_pos[joint_name] = js_pos[mimic_name] * self.mimic_joints_map[joint_name].multiplier
            if self.mimic_joints_map[joint_name].offset != None:
                js_pos[joint_name] += self.mimic_joints_map[joint_name].offset
        return

        position_map = {}
        for i in range(len(js.name)):
            position_map[js.name[i]] = js.position[i]

        for i in range(len(js.name)):
            joint_name = js.name[i]
            if joint_name in self.mimic_joints_map:
                js.position[i] = position_map[self.mimic_joints_map[joint_name].joint] * self.mimic_joints_map[joint_name].multiplier
                if self.mimic_joints_map[joint_name].offset != None:
                    js.position[i] += self.mimic_joints_map[joint_name].offset

    def updateJointLimits(self, js_pos):
        for joint_name in js_pos:
            if joint_name in self.mimic_joints_map:
                continue
            if js_pos[joint_name] < self.joint_name_limit_map[joint_name].lower:
                js_pos[joint_name] = self.joint_name_limit_map[joint_name].lower
            elif js_pos[joint_name] > self.joint_name_limit_map[joint_name].upper:
                js_pos[joint_name] = self.joint_name_limit_map[joint_name].upper
        return

        for i in range(len(js.name)):
            joint_name = js.name[i]
            if joint_name in self.mimic_joints_map:
                continue
            if js.position[i] < self.joint_name_limit_map[joint_name].lower:
                js.position[i] = self.joint_name_limit_map[joint_name].lower
            elif js.position[i] > self.joint_name_limit_map[joint_name].upper:
                js.position[i] = self.joint_name_limit_map[joint_name].upper

    def getCartImpWristTraj(self, js, goal_T_B_W):
        init_js = copy.deepcopy(js)
        init_T_B_W = self.fk_solver.calculateFk("right_arm_7_link", init_js)
        T_B_Wd = goal_T_B_W
        T_B_W_diff = PyKDL.diff(init_T_B_W, T_B_Wd, 1.0)

        self.updateJointLimits(init_js)
        self.updateMimicJoints(init_js)
        q_list = []
        for f in np.linspace(0.0, 1.0, 50):
            T_B_Wi = PyKDL.addDelta(init_T_B_W, T_B_W_diff, f)
            q_out = self.fk_solver.simulateTrajectory("right_arm_7_link", init_js, T_B_Wi)
            if q_out == None:
                print "error: getCartImpWristTraj: ", str(f)
                return None
            q_list.append(q_out)

            for i in range(7):
                joint_name = self.fk_solver.ik_joint_state_name["right_arm_7_link"][i]
                init_js[ joint_name ] = q_out[i]

        return q_list

