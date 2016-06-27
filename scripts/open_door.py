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

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from barrett_hand_controller_msgs.msg import *
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
import force_control_msgs.msg

import PyKDL
import math
import numpy as np
import copy
import matplotlib.pyplot as plt
import thread
from velma_common.velma_interface import VelmaInterface
import random
import velma_common.velmautils as velmautils
import itertools
import operator
import rospkg
from scipy import optimize

def makeWrench(f_x, f_y, f_z, t_x, t_y, t_z):
    return PyKDL.Wrench(PyKDL.Vector(f_x,f_y,f_z), PyKDL.Vector(t_x,t_y,t_z))

def calcOffsetStiffWr(stiff, wr):
    return PyKDL.Twist(
        PyKDL.Vector(wr.force.x() / stiff.force.x(), wr.force.y() / stiff.force.y(), wr.force.z() / stiff.force.z()),
        PyKDL.Vector(wr.torque.x() / stiff.torque.x(), wr.torque.y() / stiff.torque.y(), wr.torque.z() / stiff.torque.z()))

class OpenDoor:
    """
Class for the Control Subsystem behaviour: cabinet door opening.
"""

    def __init__(self, pub_marker=None):
        self.pub_marker = velmautils.MarkerPublisher()
        self.listener = tf.TransformListener();

    def estCircle(self, vec_list):
        px = []
        py = []
        for vec in vec_list:
            px.append(vec.x())
            py.append(vec.y())
        x_m = np.mean(px)
        y_m = np.mean(py)
    
        def calc_R(xc, yc):
            """ calculate the distance of each 2D points from the center (xc, yc) """
            return np.sqrt((px-xc)**2 + (py-yc)**2)

        def f_2(c):
            """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
            Ri = calc_R(*c)
            return Ri - Ri.mean()
        
        center_estimate = x_m, y_m
        center_2, ier = optimize.leastsq(f_2, center_estimate)

        xc, yc = center_2
        Ri_2   = calc_R(xc, yc)
        R      = Ri_2.mean()
        return [xc, yc, R]

    def publishDoorMarker(self, m_id, cx, cy, cz, r):
        return self.pub_marker.publishSinglePointMarker(
            PyKDL.Vector(cx, cy, cz), m_id, r=1, g=0, b=0, a=0.5, namespace='door', frame_id='torso_base', m_type=Marker.CYLINDER, scale=Vector3(r*2.0, r*2.0, 0.01), T=None)

    def spin(self):
#        px_B = [0.1, 0.1, 0.15, 0.2]
#        py_B = [0.1, 0.2, 0.25, 0.3]
#        rospy.sleep(1.0)
#        cc_x, cc_y, cc_r = self.estCircle(px_B, py_B)
#        self.publishDoorMarker(0, cc_x, cc_y, 0, cc_r)
#        print (cc_x, cc_y, cc_r)
#        door_angle = velmautils.getAngle(PyKDL.Vector(px_B[0] - cc_x, py_B[0] - cc_y, 0.0), PyKDL.Vector(px_B[-1] - cc_x, py_B[-1] - cc_y, 0.0))
#        print "door angle (deg): " + str( (door_angle/math.pi*180.0) )
#        raw_input(".")
#        return

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = VelmaInterface("/gazebo")
        velma.waitForInit()
        print "done."

        # door parameters from the vision system
        lh_pos_W = PyKDL.Vector(0.9, -0.1, 1.4)
        rh_pos_W = PyKDL.Vector(0.9, -0.15, 1.4)
        door_n_Wo = PyKDL.Vector(-1.0, 0.1, 0.0)
        door_n_Wo.Normalize()

        T_B_Wo = velma.getTf('B', 'Wo')
        door_n_B = PyKDL.Frame(T_B_Wo.M) * door_n_Wo
        lh_pos_B = T_B_Wo * lh_pos_W
        rh_pos_B = T_B_Wo * rh_pos_W

        velma.moveHandRight([1.0/180.0*math.pi, 1.0/180.0*math.pi, 1.0/180.0*math.pi, 179.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
        velma.waitForHandRight()
        rospy.sleep(0.5)
        velma.moveHandRight([100.0/180.0*math.pi, 100.0/180.0*math.pi, 100.0/180.0*math.pi, 179.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)

        if not velma.switchToJntImp():
            raise Exception()

        velma.moveToolRight(velma.getTf('Wr', 'Gr'), 0.1)
        velma.moveToolLeft(velma.getTf('Wl', 'Gl'), 0.1)
        if velma.waitForToolLeft() != 0 or velma.waitForToolRight() != 0:
            raise Exception()

        if not velma.switchToCartImp():
            raise Exception()

        gx_B = T_B_Wo * PyKDL.Vector(0,0,1)
        gz_B = door_n_B
        gy_B = gz_B * gx_B
        gy_B.Normalize()
        gx_B = gy_B * gz_B
        gx_B.Normalize()

        grip_pos_B = rh_pos_B + door_n_B * 0.05 - gy_B * 0.15

        T_B_Grd = PyKDL.Frame( PyKDL.Rotation(gx_B, gy_B, gz_B) * PyKDL.Rotation.RotY(180.0/180.0*math.pi), grip_pos_B )

        velma.waitForHandRight()
        rospy.sleep(0.5)

        velma.moveEffectorRight(T_B_Grd, 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        # wait a while to stabilize the robot and the F/T sensor output
        rospy.sleep(1)

        pub_fcl_r = rospy.Publisher('/right_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)
        pub_fcl_l = rospy.Publisher('/left_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)
        rospy.sleep(0.5)
        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.1, 0.1, 0.025), Vector3(0.01, 0.01, 0.01))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.3), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(1.1, 1.1, 1.1), Vector3(0.1, 0.1, 0.1))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        if not velma.switchToCartFcl():
            raise Exception()

        prev_T_B_Gr = None
        while not rospy.is_shutdown():
            T_B_Gr = velma.getTf('B', 'Gr')
            if prev_T_B_Gr != None:
                diff = PyKDL.diff(prev_T_B_Gr, T_B_Gr)
                wr = velma.getTransformedFTr()
                vel = diff.vel.Norm()
                rot = diff.rot.Norm()
                print vel, rot
                if vel < 0.003*0.1 and rot < (1.0/180.0*math.pi)*0.1 and wr.force.z() > 0.4:
                    break
            prev_T_B_Gr = T_B_Gr
            rospy.sleep(0.1)

#        raw_input("press ENTER to swith to CartImp...")
        print "moving backwards..."
        if not velma.switchToCartImp():
            raise Exception()

        T_B_Gr = velma.getTf('B', 'Gr')
        T_B_Gr_hit1 = T_B_Gr
        T_B_Grd = T_B_Gr * PyKDL.Frame(PyKDL.Vector(0,0.05,-0.05))

        velma.moveEffectorRight(T_B_Grd, 1.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        # wait a while to stabilize the robot and the F/T sensor output
        rospy.sleep(1)

        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.1, 0.1, 0.025), Vector3(0.01, 0.01, 0.01))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.3), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(1.1, 1.1, 1.1), Vector3(0.1, 0.1, 0.1))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        if not velma.switchToCartFcl():
            raise Exception()

        prev_T_B_Gr = None
        while not rospy.is_shutdown():
            T_B_Gr = velma.getTf('B', 'Gr')
            if prev_T_B_Gr != None:
                diff = PyKDL.diff(prev_T_B_Gr, T_B_Gr)
                wr = velma.getTransformedFTr()
                vel = diff.vel.Norm()
                rot = diff.rot.Norm()
                print vel, rot
                if vel < 0.003*0.1 and rot < (1.0/180.0*math.pi)*0.1 and wr.force.z() > 0.4:
                    break
            prev_T_B_Gr = T_B_Gr
            rospy.sleep(0.1)

        # update the door normal vector
        T_B_Gr = velma.getTf('B', 'Gr')
        T_B_Gr_hit2 = T_B_Gr

        gx_B = T_B_Wo * PyKDL.Vector(0,0,1)
        gy_B = T_B_Gr_hit2.p - T_B_Gr_hit1.p
        gz_B = gx_B * gy_B
        gx_B = gy_B * gz_B
        gx_B.Normalize()
        gy_B.Normalize()
        gz_B.Normalize()

        door_n_B = gz_B
        grip_pos_B = T_B_Gr.p + door_n_B * 0.01

        T_B_Grd = velma.getTf('B', 'Wo') * PyKDL.Frame( PyKDL.Rotation(gx_B, gy_B, gz_B) * PyKDL.Rotation.RotY(180.0/180.0*math.pi), grip_pos_B )

        if not velma.switchToCartImp():
            raise Exception()

        velma.moveEffectorRight(T_B_Grd, 1.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

#        velma.moveHandRight([35.0/180.0*math.pi, 35.0/180.0*math.pi, 35.0/180.0*math.pi, 1.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
#        velma.waitForHandRight()
#        rospy.sleep(1.0)

        # wait a while to stabilize the robot and the F/T sensor output
        rospy.sleep(1)

        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.001, 0.025, 0.025), Vector3(0.001, 0.001, 0.001))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.3, 0.0), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        if not velma.switchToCartFcl():
            raise Exception()

        prev_T_B_Gr = None
        while not rospy.is_shutdown():
            T_B_Gr = velma.getTf('B', 'Gr')
            if prev_T_B_Gr != None:
                diff = PyKDL.diff(prev_T_B_Gr, T_B_Gr)
                wr = velma.getTransformedFTr()
                vel = diff.vel.Norm()
                rot = diff.rot.Norm()
                print vel, rot
                if vel < 0.003*0.1 and rot < (1.0/180.0*math.pi)*0.1 and wr.force.y() > 0.4:
                    break
            prev_T_B_Gr = T_B_Gr
            rospy.sleep(0.1)

        print "found the handle"
        if not velma.switchToCartImp():
            raise Exception()

        stiff_move = makeWrench(200, 10, 10, 100, 100, 100)
        velma.moveImpedanceRight(stiff_move, 0.1)
        if velma.waitForImpedanceRight() != 0:
            raise Exception()

#        velma.moveHandRight([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 179.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
#        velma.waitForHandRight()
#        rospy.sleep(0.5)

        T_B_Gr = velma.getTf('B', 'Gr')
        init_R_B_Gr = copy.copy(T_B_Gr.M)

        T_B_Grd = T_B_Gr * PyKDL.Frame(PyKDL.Vector(0,0.1,0))
        velma.moveEffectorRight(T_B_Grd, 0.5, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        T_B_Grd = velma.getTf('B', 'Gr') * PyKDL.Frame(PyKDL.Vector(0,0.05,-0.20))
        velma.moveEffectorRight(T_B_Grd, 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        handle_pos_B = []
        m_id = 0
        time_start = rospy.Time.now()
        while True:
            duration = (rospy.Time.now() - time_start).to_sec()
            if duration > 6.0:
                break
#            result = velma.waitForEffectorRight(timeout_s=0.05)
            pos_B = velma.getTf('B', 'Fr22') * PyKDL.Vector(0.01, -0.01, 0)
            if len(handle_pos_B) == 0 or (pos_B-handle_pos_B[-1]).Norm() > 0.002:
                handle_pos_B.append(pos_B)
                m_id = self.pub_marker.publishSinglePointMarker(handle_pos_B[-1], m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='torso_base', m_type=Marker.CUBE, scale=Vector3(0.005, 0.005, 0.005), T=None)
            rospy.sleep(0.01)
#            if result != None:
#                if result != CartesianTrajectoryResult.SUCCESSFUL:
#                    print result
#                    raise Exception()
#                else:
#                    break

        cc_x, cc_y, cc_r = self.estCircle(handle_pos_B)
        self.publishDoorMarker(0, cc_x, cc_y, handle_pos_B[0].z(), cc_r)

        door_angle = velmautils.getAngle(handle_pos_B[0] - PyKDL.Vector(cc_x, cc_y, handle_pos_B[0].z()), handle_pos_B[-1] - PyKDL.Vector(cc_x, cc_y, handle_pos_B[-1].z()))
        print "door angle (deg): " + str( (door_angle/math.pi*180.0) )

        if cc_r > 0.4 or cc_r < 0.1:
            print "strange estimated circle radius: " + str(cc_r)
            raise Exception()

        # rotate around the handle
        T_B_Fr22 = velma.getTf('B', 'Fr22')
        T_Fr22_Contact = PyKDL.Frame(PyKDL.Vector(0.01, -0.01, 0))
        T_B_Contact = T_B_Fr22 * T_Fr22_Contact
        T_B_Contactd = PyKDL.Frame(PyKDL.Rotation.RotZ(door_angle) * T_B_Contact.M, T_B_Contact.p)
        T_B_Fr22d = T_B_Contactd * T_Fr22_Contact.Inverse()
        T_B_Grd = T_B_Fr22d * velma.getTf('Fr22', 'Gr')

        velma.moveEffectorRight(T_B_Grd, 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)


        return





















        T_B_Fr22 = velma.getTf('B', 'Fr22')

        handle_pos_B = T_B_Fr22 * PyKDL.Vector(0,-0.01,0)

        grip_pos_B = handle_pos_B + door_n_B * 0.1
        gx_B = velma.getTf('B', 'Wo') * PyKDL.Vector(0,0,1)
        gz_B = door_n_B
        gy_B = gz_B * gx_B
        gy_B.Normalize()
        gx_B = gy_B * gz_B
        gx_B.Normalize()

        T_B_Grd = PyKDL.Frame( PyKDL.Rotation(gx_B, gy_B, gz_B) * PyKDL.Rotation.RotY(180.0/180.0*math.pi), grip_pos_B )

        velma.moveEffectorRight(T_B_Grd, 2.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        result = velma.waitForEffectorRight()
        if result != CartesianTrajectoryResult.SUCCESSFUL:
            print result
            raise Exception()

        velma.moveHandRight([80.0/180.0*math.pi, 80.0/180.0*math.pi, 80.0/180.0*math.pi, 1.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
        velma.waitForHandRight()
        rospy.sleep(1.0)

        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.1, 0.1, 0.025), Vector3(0.01, 0.01, 0.01))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.5), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        if not velma.switchToCartFcl():
            raise Exception()

        prev_T_B_Gr = None
        while not rospy.is_shutdown():
            T_B_Gr = velma.getTf('B', 'Gr')
            if prev_T_B_Gr != None:
                diff = PyKDL.diff(prev_T_B_Gr, T_B_Gr)
                wr = velma.getTransformedFTr()
                vel = diff.vel.Norm()
                rot = diff.rot.Norm()
                print vel, rot
                if vel < 0.003*0.1 and rot < (1.0/180.0*math.pi)*0.1 and wr.force.z() > 0.4:
                    break
            prev_T_B_Gr = T_B_Gr
            rospy.sleep(0.1)

        velma.moveHandRight([90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, 1.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
        velma.waitForHandRight()
        rospy.sleep(1.0)


        return

        if False:
            T_Wr_Er = velma.getKDLtf('right_arm_7_link', 'right_HandPalmLink')
            T_Wl_El = velma.getKDLtf('left_arm_7_link', 'left_HandPalmLink')
            q = T_Wr_Er.M.GetQuaternion()
            p = T_Wr_Er.p
            print "right: p: " + str(p.x()) + " " + str(p.y()) + " "  + str(p.z()) + "  q: " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3])
            q = T_Wl_El.M.GetQuaternion()
            p = T_Wl_El.p
            print "left:  p: " + str(p.x()) + " " + str(p.y()) + " "  + str(p.z()) + "  q: " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3])

            p_E = PyKDL.Vector(0,0,0.06)
            p_Wr = T_Wr_Er * p_E
            p_Wl = T_Wl_El * p_E
            print "right gravity_arm_in_wrist: " + str(p_Wr.x()) + " " + str(p_Wr.y()) + " " + str(p_Wr.z())
            print "left gravity_arm_in_wrist:  " + str(p_Wl.x()) + " " + str(p_Wl.y()) + " " + str(p_Wl.z())
            return

        if False:
            while not rospy.is_shutdown():
                raw_input("press ENTER to swith to JntImp...")
                if not velma.switchToJntImp():
                    raise Exception()
                raw_input("press ENTER to swith to CartImp...")
                if not velma.switchToCartImp():
                    raise Exception()

            return

        velma.moveHandRight([45.0/180.0*math.pi, 45.0/180.0*math.pi, 45.0/180.0*math.pi, 60.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)
        velma.waitForHandRight()

        if not velma.switchToCartImpFT():
            raise Exception()

        if not velma.switchToCartImp():
            raise Exception()

        raw_input("Move end-effector to desired pose and press ENTER...")

        pub_fcl_r = rospy.Publisher('/right_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)
        pub_fcl_l = rospy.Publisher('/left_arm/fcl_param', force_control_msgs.msg.ForceControl, queue_size=0)
        rospy.sleep(0.5)
        goal = force_control_msgs.msg.ForceControl()
        goal.inertia = force_control_msgs.msg.Inertia(Vector3(20.0, 20.0, 20.0), Vector3(0.5, 0.5, 0.5))
        goal.reciprocaldamping = force_control_msgs.msg.ReciprocalDamping(Vector3(0.1, 0.1, 0.025), Vector3(1.0, 1.0, 1.0))
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.5), Vector3(0.0, 0.0, 0.0))
        goal.twist = geometry_msgs.msg.Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_r.publish(goal)
        goal.wrench = geometry_msgs.msg.Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub_fcl_l.publish(goal)

        print velma.getControllerBehaviour()
        if not velma.switchToCartFcl():
            raise Exception()
        print velma.getControllerBehaviour()

        raw_input("press ENTER to swith to CartImp...")
        if not velma.switchToCartImp():
            raise Exception()


        return

        if not velma.isJointImpedanceActive():
            if not velma.switchToJoint():
                raise Exception()

        velma.updateTransformations()

        velma.moveToolRight(velma.T_Wr_Gr, 0.1)
        velma.moveToolLeft(velma.T_Wl_Gl, 0.1)
        if velma.waitForToolLeft() != 0 or velma.waitForToolRight() != 0:
            raise Exception()

        stiff_move = makeWrench(500, 500, 500, 100, 100, 100)
        velma.moveImpedanceRight(stiff_move, 0.1)
        velma.moveImpedanceLeft(stiff_move, 0.1)
        if velma.waitForImpedanceLeft() != 0 or velma.waitForImpedanceRight() != 0:
            raise Exception()

        if not velma.switchToCart():
            raise Exception()

        velma.moveHandRight([0.1, 0.1, 0.1, 60.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)

        print "moving to initial pose"
        path_tol = calcOffsetStiffWr(stiff_move, makeWrench(10.0, 10.0, 10.0, 4.0, 4.0, 4.0))
        velma.moveEffectorRight(PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.5,-0.5, 1.0)), 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=path_tol)
        if velma.waitForEffectorRight() != CartesianTrajectoryResult.SUCCESSFUL:
            raise Exception()

        stiff_door = makeWrench(30, 30, 100, 10, 10, 10)
        velma.moveImpedanceRight(stiff_door, 0.1)
        if velma.waitForImpedanceRight() != 0:
            raise Exception()

        print "pushing forward"
        path_tol = calcOffsetStiffWr(stiff_door, makeWrench(8.0, 8.0, 8.0, 3.0, 3.0, 3.0))
        velma.moveEffectorRight(PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.9,-0.5, 1.0)), 10.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=path_tol)
        if velma.waitForEffectorRight() != CartesianTrajectoryResult.PATH_TOLERANCE_VIOLATED:
            raise Exception()

        print "moving to current pose"
        velma.updateTransformations()
        velma.moveEffectorRight(velma.T_B_Wr * velma.T_Wr_Tr, 0.5, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=None)
        if velma.waitForEffectorRight() != CartesianTrajectoryResult.SUCCESSFUL:
            raise Exception()

        print "moving to initial pose"
        velma.moveEffectorRight(PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.5,-0.5, 1.0)), 3.0, PyKDL.Wrench(PyKDL.Vector(10,10,10), PyKDL.Vector(4,4,4)), start_time=0.1, stamp=None, path_tol=path_tol)
        if velma.waitForEffectorRight() != CartesianTrajectoryResult.SUCCESSFUL:
            raise Exception()

        velma.moveHandRight([40.0/180.0*math.pi, 40.0/180.0*math.pi, 40.0/180.0*math.pi, 180.0/180.0*math.pi], [1.2, 1.2, 1.2, 1.2], [3000,3000,3000,3000], 4000, hold=True)

        velma.moveImpedanceRight(stiff_move, 0.1)
        if velma.waitForImpedanceRight() != 0:
            raise Exception()

        return

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

    rospy.init_node('open_door')

    task = OpenDoor()

    task.spin()


