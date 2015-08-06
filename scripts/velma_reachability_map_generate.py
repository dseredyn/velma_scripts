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

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import velmautils
from multiprocessing import Process, Queue
import random

def drawDodecahedronFace(fr, a, i_base, name):
    # radius of the inscribed sphere
    r_i = a * 0.5 * math.sqrt(5.0/2.0 + 11.0/10.0*math.sqrt(5.0))

    R = 2.0 * a / math.sqrt(2.0*(5.0-math.sqrt(5.0)))

    # generate vertices
    v = []
    for i in range(0,5):
        v.append(PyKDL.Vector(R*math.cos(i*72.0/180.0*math.pi), R*math.sin(i*72.0/180.0*math.pi), r_i))

    i = 0
    # draw vertices
    for pt in v:
        publishSinglePointMarker(fr*pt, i_base+i, r=1, g=0, b=0, namespace=name, frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005))
        i += 1

    q = fr.M.GetQuaternion()
    center = PyKDL.Vector(0,0,r_i)
    publishSinglePointMarker(fr*center, i_base+i, r=0.5, g=0.5, b=0.5, namespace=name, frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(a, a, 0.001), orientation=Quaternion(q[0],q[1],q[2],q[3]))
    i += 1
    # draw axes
    publishVectorMarker(fr*center, fr*(center+PyKDL.Vector(a,0,0)), i_base+i, 1, 0, 0, frame='torso_base', namespace=name)
    i += 1
    publishVectorMarker(fr*center, fr*(center+PyKDL.Vector(0,a,0)), i_base+i, 0, 1, 0, frame='torso_base', namespace=name)
    i += 1
    publishVectorMarker(fr*center, fr*(center+PyKDL.Vector(0,0,a)), i_base+i, 0, 0, 1, frame='torso_base', namespace=name)
    i += 1

    return i_base+i

def generateRotationsForDodecahedron():
    angle = 72.0/180.0*math.pi
    dihedral_angle = math.acos(-1.0/math.sqrt(5.0))
    dihedral_transform_angle = math.pi - dihedral_angle

    i = 0
    fr1 = PyKDL.Frame()
#    i = drawDodecahedronFace(fr1, 0.1, i, "fr1")

    fr2 = fr1 * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr2, 0.1, i, "fr2")

    fr3 = fr2 * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr3, 0.1, i, "fr3")

    fr4 = fr3 * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr4, 0.1, i, "fr4")

    fr5 = fr4 * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr5, 0.1, i, "fr5")

    fr6 = fr1 * PyKDL.Frame( PyKDL.Rotation.RotZ(angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr6, 0.1, i, "fr6")

    fr7 = fr1 * PyKDL.Frame( PyKDL.Rotation.RotZ(-angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr7, 0.1, i, "fr7")

    fr8 = fr7 * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr8, 0.1, i, "fr8")

    fr9 = fr8 * PyKDL.Frame( PyKDL.Rotation.RotZ(-angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr9, 0.1, i, "fr9")

    fr10 = fr9 * PyKDL.Frame( PyKDL.Rotation.RotZ(-angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr10, 0.1, i, "fr10")

    fr11 = fr10 * PyKDL.Frame( PyKDL.Rotation.RotZ(-angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr11, 0.1, i, "fr11")

    fr12 = fr11 * PyKDL.Frame( PyKDL.Rotation.RotZ(-2.0*angle) ) * PyKDL.Frame( PyKDL.Rotation.RotY(-dihedral_transform_angle) ) * PyKDL.Frame( PyKDL.Rotation.RotZ(angle/2.0) )
#    i = drawDodecahedronFace(fr12, 0.1, i, "fr12")

    frames = []
    frames.append(fr1)
    frames.append(fr2)
    frames.append(fr3)
    frames.append(fr4)
    frames.append(fr5)
    frames.append(fr6)
    frames.append(fr7)
    frames.append(fr8)
    frames.append(fr9)
    frames.append(fr10)
    frames.append(fr11)
    frames.append(fr12)

    ret = []
    for f in frames:
        ret.append( copy.deepcopy(f.M) )
        ret.append( copy.deepcopy((f*PyKDL.Frame(PyKDL.Rotation.RotZ(angle))).M) )
        ret.append( copy.deepcopy((f*PyKDL.Frame(PyKDL.Rotation.RotZ(2.0*angle))).M) )
        ret.append( copy.deepcopy((f*PyKDL.Frame(PyKDL.Rotation.RotZ(3.0*angle))).M) )
        ret.append( copy.deepcopy((f*PyKDL.Frame(PyKDL.Rotation.RotZ(4.0*angle))).M) )

#    i = 0
#    for f in ret:
#        # draw axes
#        publishVectorMarker(PyKDL.Vector(), f*PyKDL.Vector(0.1,0,0), i, 1, 0, 0, frame='torso_base', namespace='default')
#        i += 1
#        publishVectorMarker(PyKDL.Vector(), f*PyKDL.Vector(0,0.1,0), i, 0, 1, 0, frame='torso_base', namespace='default')
#        i += 1
#        publishVectorMarker(PyKDL.Vector(), f*PyKDL.Vector(0,0,0.1), i, 0, 0, 1, frame='torso_base', namespace='default')
#        i += 1

    return ret

class VelmaIkSolver:

    def __init__(self, side, step, x_min, x_max, y_min, y_max, z_min, z_max, pt_c_in_T2, min_dist, max_dist, rot):
        self.min_dist2 = min_dist*min_dist
        self.max_dist2 = max_dist*max_dist
        self.x_set = list(np.arange(x_min, x_max, step))
        self.y_set = list(np.arange(y_min, y_max, step))
        if z_min > z_max:
            self.z_set = list(np.arange(z_min, z_max, -step))
        else:
            self.z_set = list(np.arange(z_min, z_max, step))
        self.pt_c_in_T2 = pt_c_in_T2
        self.rot = rot

        self.robot = URDF.from_parameter_server()
        self.tree = kdl_tree_from_urdf_model(self.robot)

        self.chain = self.tree.getChain("torso_link2", side + "_HandPalmLink")

        self.q_min = PyKDL.JntArray(7)
        self.q_max = PyKDL.JntArray(7)
        self.q_limit = 0.01
        self.q_min[0] = -2.96 + self.q_limit
        self.q_min[1] = -2.09 + self.q_limit
        self.q_min[2] = -2.96 + self.q_limit
#        self.q_min[3] = -2.09 + self.q_limit
        self.q_min[3] = 15.0/180.0*math.pi
        self.q_min[4] = -2.96 + self.q_limit
        self.q_min[5] = -2.09 + self.q_limit
        self.q_min[6] = -2.96 + self.q_limit
        self.q_max[0] = 2.96 - self.q_limit
#        self.q_max[1] = 2.09 - self.q_limit
        self.q_max[1] = -15.0/180.0*math.pi
        self.q_max[2] = 2.96 - self.q_limit
        self.q_max[3] = 2.09 - self.q_limit
        self.q_max[4] = 2.96 - self.q_limit
#        self.q_max[5] = 2.09 - self.q_limit
        self.q_max[5] = -15.0/180.0*math.pi
        self.q_max[6] = 2.96 - self.q_limit
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.vel_ik_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(self.chain, self.q_min, self.q_max, self.fk_solver, self.vel_ik_solver, 100)

        self.q_out = PyKDL.JntArray(7)

        self.singularity_angle = 15.0/180.0*math.pi

    def hasSingularity(self, q):
        if (math.fabs(q[1]) <= self.singularity_angle) or (math.fabs(q[3]) <= self.singularity_angle) or (math.fabs(q[5]) <= self.singularity_angle):
            return True
        return False

    def violatedOtherLimits(self, q):
        if q[0] >= 0.2:
            return True
        if q[3] <= 0.0:
            return True
        return False

    def calculateIk(self, x_index_start, x_index_end, queue):

        ret = {}
        x_index = x_index_start
        for x in self.x_set[x_index_start:x_index_end]:
            print >> sys.stderr, "x=%s"%(x)
            if rospy.is_shutdown():
                break
            y_index = 0
            for y in self.y_set:
                z_index = 0
                for z in self.z_set:
                    if rospy.is_shutdown():
                        break
                    # do not calculate the reachability map for torso area
                    if x > -0.5 and x < 0.2 and z > -0.14 and z < 0.14:
                        z_index += 1
                        continue
                    rot_index = 0
                    dist = (self.pt_c_in_T2.x()-x)*(self.pt_c_in_T2.x()-x) + (self.pt_c_in_T2.y()-y)*(self.pt_c_in_T2.y()-y) + (self.pt_c_in_T2.z()-z)*(self.pt_c_in_T2.z()-z)
                    if dist <= self.max_dist2 and dist >= self.min_dist2:
                        for r in self.rot:
                            fr = PyKDL.Frame(r, PyKDL.Vector(x,y,z))
                            success = False
                            for i in range(0,5):
                                q_init = PyKDL.JntArray(7)
                                for j in range(0,7):
                                    q_init[j] = random.uniform(self.q_min[j]+0.1, self.q_max[j]-0.1)
                                status = self.ik_solver.CartToJnt(q_init, fr, self.q_out)
                                if status == 0:# and not self.hasSingularity(self.q_out):
                                    success = True
#                                    print >> sys.stderr, "sing. %s"%(i)
                                    break
                            if success:
                                if not (x_index, y_index, z_index) in ret:
                                    ret[ (x_index, y_index, z_index) ] = set()
                                ret[ (x_index, y_index, z_index) ].add( rot_index )
                            rot_index += 1
                    z_index += 1
                y_index += 1
            x_index += 1
        print "queue put"
        queue.put(ret)

if __name__ == '__main__':

    # literature:
    # https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/PyKDL/kinfam.sip
    # http://wiki.ros.org/pykdl_utils
    # https://github.com/gt-ros-pkg/hrl-kdl/blob/hydro/pykdl_utils/src/pykdl_utils/kdl_parser.py
    # http://people.mech.kuleuven.be/~rsmits/kdl/api/html/annotated.html
    # http://people.mech.kuleuven.be/~rsmits/kdl/api/html/classKDL_1_1ChainIkSolverPos__NR__JL.html

    # to read:
    # https://github.com/benersuay/rmaps
    # http://www.dlr.de/rmc/rm/Portaldata/52/Resources/images/institute/robotersysteme/rollin_justin/mobile_manipulation/leidner2014object.pdf

    pub_marker = velmautils.MarkerPublisher()

    side = "left"

    rospy.init_node('velma_ik_solver')

    listener = tf.TransformListener()
    while True:
        if listener.canTransform('torso_link2', side + '_arm_2_link', rospy.Time(0)):
            pose = listener.lookupTransform('torso_link2', side + '_arm_2_link', rospy.Time(0))
            T_T2_L2 = pm.fromTf(pose)
            break
        rospy.sleep(0.1)

    pt_c_in_T2 = T_T2_L2 * PyKDL.Vector()
    min_dist = 0.330680893438 - 0.05
    max_dist = 0.903499165003 + 0.05

#    x_min = -1.0
    x_min = -0.2
    x_max = 1.0
    y_min = -0.4
#    y_max = 1.3
    y_max = 0.8

    if side == "right":
        z_min = -0.4
        z_max = 1.2
    else:
        z_min = 0.4
        z_max = -1.2
#        z_min = -1.2
#        z_max = 0.4


    rot = generateRotationsForDodecahedron()

    ik = VelmaIkSolver(side, 0.15, x_min, x_max, y_min, y_max, z_min, z_max, pt_c_in_T2, min_dist, max_dist, rot)

    ret = {}
    for i in range(0,len(ik.x_set),4):
            queues = []
            queues.append(Queue(1))
            p0 = Process(target=ik.calculateIk, args=(i, i+1, queues[-1]))
            p0.start()
            if i+1 < len(ik.x_set):
                queues.append(Queue(1))
                p1 = Process(target=ik.calculateIk, args=(i+1, i+2, queues[-1]))
                p1.start()
            if i+2 < len(ik.x_set):
                queues.append(Queue(1))
                p2 = Process(target=ik.calculateIk, args=(i+2, i+3, queues[-1]))
                p2.start()
            if i+3 < len(ik.x_set):
                queues.append(Queue(1))
                p3 = Process(target=ik.calculateIk, args=(i+3, i+4, queues[-1]))
                p3.start()
            p0.join()
            print >> sys.stderr, "p0 joined"
            if i+1 < len(ik.x_set):
                p1.join()
                print >> sys.stderr, "p1 joined"
            if i+2 < len(ik.x_set):
                p2.join()
                print >> sys.stderr, "p2 joined"
            if i+3 < len(ik.x_set):
                p3.join()
                print >> sys.stderr, "p3 joined"

            for q in queues:
                print "queue get"
                data = q.get()
                for coord in data:
                    if coord in ret:
                        print "ERROR: coord in ret"
                        exit(0)
                    else:
                        ret[coord] = data[coord]

    filename = "pose_lookup_table_"+side+".py"

    with open(filename, 'w') as f:

            f.write("import PyKDL\n\nrotations=[")
            for r in ik.rot:
                q = r.GetQuaternion()
                f.write("PyKDL.Rotation.Quaternion("+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+"),\n")
            f.write("]\n")

            f.write( "x_set="+str(ik.x_set)+"\n" )
            f.write( "y_set="+str(ik.y_set)+"\n" )
            f.write( "z_set="+str(ik.z_set)+"\n" )

            f.write("lookup_table={\n")
            for coord in ret:
                f.write( str(coord) + ":" + str(ret[coord]) + ",\n" )
            f.write("} # lookup_table\n")

            f.write( "min_dist="+str(min_dist)+"\n" )
            f.write( "max_dist="+str(max_dist)+"\n" )
            f.write( "pt_c_in_T2 = PyKDL.Vector(" + str(pt_c_in_T2.x()) + "," + str(pt_c_in_T2.y()) + "," + str(pt_c_in_T2.z()) + ")\n" )
            f.write("side=\""+side+"\"\n\n")

            f.write("def getIdx(v, v_min, v_max, step, count):\n")
            f.write("    f = (v - (v_min - step * 0.5)) / (v_max-v_min+step)\n")
            f.write("    idx = int(f * count)\n")
            f.write("    if idx >= count or idx < 0:\n")
            f.write("        idx = None\n")
            f.write("    return idx\n\n")

            f.write("def getIdxX(v):\n")
            f.write("    return getIdx(v, "+str(ik.x_set[0])+","+str(ik.x_set[-1])+","+str(ik.x_set[1]-ik.x_set[0])+","+str(len(ik.x_set))+")\n\n")

            f.write("def getIdxY(v):\n")
            f.write("    return getIdx(v, "+str(ik.y_set[0])+","+str(ik.y_set[-1])+","+str(ik.y_set[1]-ik.y_set[0])+","+str(len(ik.y_set))+")\n\n")

            f.write("def getIdxZ(v):\n")
            f.write("    return getIdx(v, "+str(ik.z_set[0])+","+str(ik.z_set[-1])+","+str(ik.z_set[1]-ik.z_set[0])+","+str(len(ik.z_set))+")\n\n")

            f.write("def getClosestRot(TR_T2_O):\n")
            f.write("    rot_idx = 0\n")
            f.write("    min_rot_idx = None\n")
            f.write("    min_diff = 1000000.0\n")
            f.write("    for rot in rotations:\n")
            f.write("        diff = PyKDL.diff(PyKDL.Frame(rot), TR_T2_O).rot.Norm()\n")
            f.write("        if diff < min_diff:\n")
            f.write("            min_diff = diff\n")
            f.write("            min_rot_idx = rot_idx\n")
            f.write("        rot_idx += 1\n")
            f.write("    return min_rot_idx\n\n")

    print "done."

