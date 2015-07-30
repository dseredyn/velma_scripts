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

import state_server_msgs.srv

import PyKDL
import math
import numpy as np
import copy
import openravepy
from openravepy import *
import rospkg

import velmautils
from velma import Velma
import openraveinstance
import conversions as conv
import rrt_star_connect_planner
import tasks
import objectstate

import pose_lookup_table_left as plutl
import pose_lookup_table_right as plutr

import sys

class RobrexJarCabinet:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()

    def spin(self):

        sys.setrecursionlimit(200)

        # generate the set of grasps for a jar
        grasps_T_J_E = []
        for angle_jar_axis in np.arange(0.0, math.pi*2.0, 10.0/180.0*math.pi):
            for translation_jar_axis in np.linspace(-0.03, 0.03, 7):
                T_J_E = PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.17))
                grasps_T_J_E.append( (T_J_E, T_J_E.Inverse()) )
                T_J_E = PyKDL.Frame(PyKDL.Rotation.RotZ(-90.0/180.0*math.pi)) * PyKDL.Frame(PyKDL.Rotation.RotX(angle_jar_axis)) * PyKDL.Frame(PyKDL.Vector(translation_jar_axis, 0, -0.17))
                grasps_T_J_E.append( (T_J_E, T_J_E.Inverse()) )

        # TEST: OpenJarTaskRRT
        if False:
            task = tasks.OpenJarTaskRRT(None, ("right", grasps_T_J_E, ))

            m_id = 0
            for coord in task.valid:
                xi,yi,zi = coord
                x = plutl.x_set[xi]
                y = plutl.y_set[yi]
                z = plutl.z_set[zi]
                rot_set = task.valid[coord]
                size = float(len(rot_set))/40.0
                m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(x,y,z), m_id, r=1.0, g=1*size, b=1*size, a=1, namespace='default', frame_id="torso_link2", m_type=Marker.SPHERE, scale=Vector3(0.05*size, 0.05*size, 0.05*size))
                rospy.sleep(0.01)

            exit(0)

        rospack = rospkg.RosPack()
        env_file=rospack.get_path('velma_scripts') + '/data/common/velma_room.env.xml'
#        env_file=rospack.get_path('velma_scripts') + '/data/common/velma_room_obstacles.env.xml'
        srdf_path=rospack.get_path('velma_description') + '/robots/'

        obj_filenames = [
        rospack.get_path('velma_scripts') + '/data/jar/jar.kinbody.xml'
        ]

        rrt = rrt_star_connect_planner.PlannerRRT(3, env_file, obj_filenames, srdf_path)

        rospy.wait_for_service('/change_state')
        changeState = rospy.ServiceProxy('/change_state', state_server_msgs.srv.ChangeState)
        self.listener = tf.TransformListener()

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        self.pub_head_look_at = rospy.Publisher("/head_lookat_pose", geometry_msgs.msg.Pose)
        print "done."

        rospy.sleep(0.5)
        velma.updateTransformations()

        # switch to cartesian impedance mode...
#        velma.switchToCart()
#        exit(0)

        velma.switchToJoint()

        # TEST: moving in joint impedance mode
        if False:
            raw_input("Press ENTER to move the robot...")
            velma.switchToJoint()
            joint_names = [ "right_arm_4_joint" ]
            q_dest = [ velma.js_pos["right_arm_4_joint"] - 10.0/180.0*math.pi ]
            velma.moveJoint(q_dest, joint_names, 10.0, start_time=0.5)
            traj = [[q_dest], None, None, [10.0]]
            velma.moveJointTraj(traj, joint_names, start_time=0.5)
            result = velma.waitForJoint()
            print "moveJointTraj result", result.error_code
            exit(0)

        hv = [1.2, 1.2, 1.2, 1.2]
        ht = [3000, 3000, 3000, 3000]

        target_gripper = "right"

        if target_gripper == "left":
            velma.moveHandLeft([40.0/180.0*math.pi, 40.0/180.0*math.pi, 40.0/180.0*math.pi, 0], hv, ht, 5000, True)
            velma.moveHandRight([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)
        else:
            velma.moveHandLeft([120.0/180.0*math.pi, 120.0/180.0*math.pi, 120.0/180.0*math.pi, 0], hv, ht, 5000, True)
            velma.moveHandRight([40.0/180.0*math.pi, 40.0/180.0*math.pi, 40.0/180.0*math.pi, 0], hv, ht, 5000, True)
        result = velma.waitForHandLeft()
        print "waitForHandLeft result", result.error_code
        result = velma.waitForHandRight()
        print "waitForHandRight result", result.error_code

        #
        # Initialise Openrave
        #

        openrave = openraveinstance.OpenraveInstance()
        openrave.startOpenrave(collision='fcl')
        openrave.loadEnv(env_file)
        openrave.runOctomapClient(ros=True)
        openrave.readRobot(srdf_path=srdf_path)

        for filename in obj_filenames:
            body = openrave.env.ReadKinBodyXMLFile(filename)
            body.Enable(False)
            body.SetVisible(False)
            openrave.env.Add(body)

        mo_state = objectstate.MovableObjectsState(openrave.env, obj_filenames)

        openrave.setCamera(PyKDL.Vector(2.0, 0.0, 2.0), PyKDL.Vector(0.60, 0.0, 1.10))
        openrave.updateRobotConfigurationRos(velma.js_pos)

        while True:
            if self.listener.canTransform('torso_base', 'jar', rospy.Time(0)):
                pose = self.listener.lookupTransform('torso_base', 'jar', rospy.Time(0))
                T_B_J = pm.fromTf(pose)
                break

        print "waiting for planner..."
        rrt.waitForInit()
        print "planner initialized"

        T_B_E_list = []
        for T_J_E, T_E_J in grasps_T_J_E:
            T_B_Ed = T_B_J * T_J_E
            T_B_E_list.append(T_B_Ed)
        print "grasps for jar:", len(T_B_E_list)

        if True:
            # look around
            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.2))))
            rospy.sleep(3)
#            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,-1,1.2))))
#            rospy.sleep(2)
#            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,1,1.2))))
#            rospy.sleep(2)
#            self.pub_head_look_at.publish(pm.toMsg(PyKDL.Frame(PyKDL.Vector(1,0,1.2))))
#            rospy.sleep(2)

        raw_input("Press ENTER to start planning...")

        openrave.updateOctomap()
        tree_serialized = openrave.or_octomap_client.SendCommand("GetOcTree")

        for i in range(10):
            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            mo_state.update(self.listener, time_tf)
            mo_state.updateOpenrave(openrave.env)
            rospy.sleep(0.1)

        print "octomap updated"

        print "Planning trajectory to grasp the jar..."
        env_state = (openrave.robot_rave.GetDOFValues(), mo_state.obj_map, tree_serialized, [])
        path, dof_names = rrt.RRTstar(env_state, tasks.GraspTaskRRT, (target_gripper, T_B_E_list), 240.0)

        traj = []
        for i in range(len(path)-1):
            q1 = path[i]
            q2 = path[i+1]
            for f in np.linspace(0.0, 1.0, 40):
                traj.append( q1 * (1.0 - f) + q2 * f )
        while True:
            if raw_input("Type e to execute") == 'e':
                break
            openrave.showTrajectory(dof_names, 10.0, traj)

        traj = velma.prepareTrajectory(path, velma.getJointStatesByNames(dof_names), dof_names, speed_mult=1.0)

        velma.switchToJoint()
        velma.moveJointTraj(traj, dof_names, start_time=0.5)
        result = velma.waitForJoint()
        print "moveJointTraj result", result.error_code
        if result.error_code != 0:
            exit(0)

        raw_input("Press ENTER to continue...")

        openrave.updateRobotConfigurationRos(velma.js_pos)

        print "before grasp"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        raw_input("Press ENTER to close the gripper...")

        if target_gripper == "left":
            velma.moveHandLeft([90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, 0], hv, ht, 5000, True)
            result = velma.waitForHandLeft()
            print "moveHandLeft result", result.error_code
        else:
            velma.moveHandRight([90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, 0], hv, ht, 5000, True)
            result = velma.waitForHandRight()
            print "waitForHandRight result", result.error_code

        openrave.updateRobotConfigurationRos(velma.js_pos)

        print "after grasp"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        openrave.robot_rave.Grab(openrave.env.GetKinBody("jar"), openrave.robot_rave.GetLink(target_gripper+"_HandPalmLink"))
        changeState("grasp "+target_gripper+"_HandPalmLink "+ "jar")

        print "after grab"
        report = CollisionReport()
        if openrave.env.CheckCollision(openrave.robot_rave, report):
            print "collision"
        else:
            print "no collision"
        report = CollisionReport()
        if openrave.robot_rave.CheckSelfCollision(report):
            print "self-collision"
        else:
            print "no self-collision"

        print "Planning trajectory to pull the jar from the cabinet..."
        for i in range(10):
            time_tf = rospy.Time.now()-rospy.Duration(0.5)
            mo_state.update(self.listener, time_tf)
            mo_state.updateOpenrave(openrave.env)
            rospy.sleep(0.1)
        env_state = (openrave.robot_rave.GetDOFValues(), mo_state.obj_map, tree_serialized, [["jar", target_gripper+"_HandPalmLink"]])
#        path, dof_names = rrt.RRTstar(env_state, tasks.GraspTaskRRT, (target_gripper, T_B_E_list), 60.0)

        path, dof_names = rrt.RRTstar(env_state, tasks.MoveArmsCloseTaskRRT, (None,), 120.0)

#        path.reverse()

        traj = []
        for i in range(len(path)-1):
            q1 = path[i]
            q2 = path[i+1]
            for f in np.linspace(0.0, 1.0, 40):
                traj.append( q1 * (1.0 - f) + q2 * f )
        while True:
            if raw_input("Type e to execute") == 'e':
                break
            openrave.showTrajectory(dof_names, 10.0, traj)


        traj = velma.prepareTrajectory(path, velma.getJointStatesByNames(dof_names), dof_names, speed_mult=1.0)
        result = velma.moveJointTraj(traj, dof_names, start_time=0.5)
        result = velma.waitForJoint()
        print "moveJointTraj result", result.error_code
        if result.error_code != 0:
            exit(0)

        raw_input("Press ENTER to continue...")
        openrave.updateRobotConfigurationRos(velma.js_pos)

        changeState("drop "+target_gripper+"_HandPalmLink")

        raw_input("Press ENTER to exit...")

        rrt.cleanup()

if __name__ == '__main__':
    rospy.init_node('robrex_jar_cabinet')
    task = RobrexJarCabinet()
    task.spin()

