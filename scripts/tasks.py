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

import PyKDL
import numpy as np
import math
import random
import operator
from openravepy import *
import headkinematics
import velmautils
import tree
import conversions as conv

class LooAtTaskRRT:
    def __init__(self, openrave, args):
        self.openrave = openrave

        v_rot = 0.800
        v_lean = 0.375
        v_head = 0.392
        h_cam = 0.0
        v_cam = 0.225
        self.head_kin = headkinematics.HeadKinematics(v_rot, v_lean, v_head, h_cam, v_cam)

        self.kinect_fov = 30.0/180.0*math.pi

        # target: key pocket
        self.vis_targets = [
        ("vis_target_0", 0.1, PyKDL.Vector(0, -0.4, 1.0)),
        ("vis_target_1", 0.1, PyKDL.Vector(0.1, -0.4, 1.0)),
        ("vis_target_2", 0.1, PyKDL.Vector(0.1, -0.5, 1.0)),
        ("vis_target_3", 0.1, PyKDL.Vector(0, -0.5, 1.0)),
#        ("vis_target_4", 0.1, PyKDL.Vector(0.05, -0.45, 1.0)),
        ]
        self.head_target_B = PyKDL.Vector()
        for target in self.vis_targets:
            self.head_target_B += target[2]
        self.head_target_B = self.head_target_B / len(self.vis_targets)

        self.vis_bodies = []
        # target: test (vertical axis at the door plane)
#        self.vis_targets = [
#        ("vis_target_0", 0.1, PyKDL.Vector(1, 0.0, 1.2)),
#        ("vis_target_1", 0.1, PyKDL.Vector(1, 0.0, 1.3)),
#        ("vis_target_2", 0.1, PyKDL.Vector(1, 0.0, 1.4)),
#        ("vis_target_3", 0.1, PyKDL.Vector(1, 0.0, 1.5)),
#        ("vis_target_4", 0.1, PyKDL.Vector(1, 0.0, 1.6)),
#        ]

        for (name, diam, pos) in self.vis_targets:
            body = self.openrave.addSphere(name, diam)
            body.SetTransform(conv.KDLToOpenrave(PyKDL.Frame(pos)))
            self.vis_bodies.append( body )
            self.openrave.env.Remove( body )

        self.dof_names = [
        "head_pan_joint",
        "head_tilt_joint",
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "torso_0_joint",
        ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

    def GetDofLimits(self):
        return self.dof_limits

    def GetDofIndices(self):
        return self.dof_indices

    def GetDofNames(self):
        return self.dof_names

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
            self.openrave.switchCollisionModel("velmasimplified0")
            rays_hit = 0
            m_id = 0

            self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)

            for body in self.vis_bodies:
                self.openrave.env.Add( body )
            T_W_C = conv.OpenraveToKDL(self.openrave.robot_rave.GetLink("head_kinect_rgb_optical_frame").GetTransform())
            T_C_W = T_W_C.Inverse()
            cam_W = T_W_C * PyKDL.Vector()
            cam_dir_W = PyKDL.Frame(T_W_C.M) * PyKDL.Vector(0,0,0.5)

            # create rays connecting the optical frame and the target objects
            for (name, diam, pos_W) in self.vis_targets:
                pos_C = T_C_W * pos_W
                dir_W = pos_W - cam_W
                if pos_C.z() < 0.1:
                    continue
                if velmautils.getAngle(PyKDL.Vector(0,0,1), pos_C) > self.kinect_fov:
                    continue

                report = CollisionReport()
                ret = self.openrave.env.CheckCollision(Ray((cam_W[0], cam_W[1], cam_W[2]), (dir_W[0], dir_W[1], dir_W[2])), report)
                if ret and report.plink1 != None and report.plink1.GetParent().GetName().find("vis_target_") == 0:
                    rays_hit += 1
                else:
                    pass
            for body in self.vis_bodies:
                self.openrave.env.Remove( body )

            return rays_hit == 4

    def SampleGoal(self, start_q, shortest_path_len):
            ignore_dof = [self.dof_indices_map["torso_0_joint"], self.dof_indices_map["head_pan_joint"], self.dof_indices_map["head_tilt_joint"]]
            q_goal = np.empty(len(self.dof_names))
            for tries in range(200):

                torso_0_joint_idx = self.dof_indices_map["torso_0_joint"]
                q_goal[torso_0_joint_idx] = random.uniform(self.dof_limits[torso_0_joint_idx][0]+0.01, self.dof_limits[torso_0_joint_idx][1]-0.01)
                self.head_kin.UpdateTorsoPose(q_goal[self.dof_indices_map["torso_0_joint"]], self.openrave.robot_rave.GetJoint("torso_1_joint").GetValue(0))
                self.head_kin.UpdateTargetPosition(self.head_target_B.x(), self.head_target_B.y(), self.head_target_B.z())
                self.head_kin.TransformTargetToHeadFrame()
                joint_pan, joint_tilt = self.head_kin.CalculateHeadPose()
                if joint_pan == None:
                    continue
                joint_pan = max(joint_pan, self.dof_limits[self.dof_indices_map["head_pan_joint"]][0])
                joint_pan = min(joint_pan, self.dof_limits[self.dof_indices_map["head_pan_joint"]][1])
                joint_tilt = max(joint_tilt, self.dof_limits[self.dof_indices_map["head_tilt_joint"]][0])
                joint_tilt = min(joint_tilt, self.dof_limits[self.dof_indices_map["head_tilt_joint"]][1])
                q_goal[self.dof_indices_map["head_pan_joint"]] = joint_pan
                q_goal[self.dof_indices_map["head_tilt_joint"]] = joint_tilt

                if shortest_path_len == None:
                    for i in range(len(self.dof_names)):
                        if i in ignore_dof:
                            continue
                        q_goal[i] = random.uniform(self.dof_limits[i][0]+0.01, self.dof_limits[i][1]-0.01)
                else:
                    diff = 0.0
                    for dof_idx in ignore_dof:
                        diff += (start_q[dof_idx] - q_goal[dof_idx]) * (start_q[dof_idx] - q_goal[dof_idx])
                    shortest_path_len2 = shortest_path_len*shortest_path_len - diff
                    if shortest_path_len2 < 0.0:
                        continue
                    shortest_path_len2 = math.sqrt(shortest_path_len2)
                    q_goal2 = tree.uniformInBall(shortest_path_len2, self.dof_limits, start_q, ignore_dof=ignore_dof)
                    for dof_idx in ignore_dof:
                        q_goal2[dof_idx] = q_goal[dof_idx]
                    q_goal = q_goal2

                # sanity check
                if shortest_path_len != None and np.linalg.norm(q_goal-start_q) > shortest_path_len:
                    print "ERROR: np.linalg.norm(q_goal-start_q) > shortest_path_len", np.linalg.norm(q_goal-start_q), ">", shortest_path_len
                    exit(0)

                if self.checkGoal(q_goal):
                    return [q_goal]
            return None

class KeyRotTaskRRT:
    def __init__(self, openrave, args):
        self.openrave = openrave

        self.T_E_O = PyKDL.Frame()
        self.T_O_E = self.T_E_O.Inverse()
        self.key_axis_O = PyKDL.Vector(0,0,1)
        self.key_up_O = PyKDL.Vector(1,0,0)
        self.key_side_O = self.key_axis_O * self.key_up_O

        self.key_endpoint_O = PyKDL.Vector(0.000256401261281, -0.000625166847342, 0.232297442735)
        self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.71891504857, -0.0529880479354, 0.691118088949, 0.0520500417212), PyKDL.Vector(0.883081316461, -0.100813768303, 0.95381559114))

        # get the transformation from wrist to palm
        link_E = self.openrave.robot_rave.GetLink("right_HandPalmLink")
        link_W = self.openrave.robot_rave.GetLink("right_arm_7_link")
        T_World_E = conv.OpenraveToKDL(link_E.GetTransform())
        T_World_W = conv.OpenraveToKDL(link_W.GetTransform())
        self.T_W_E = T_World_W.Inverse() * T_World_E
        self.T_E_W = self.T_W_E.Inverse()

        self.key_traj1_T_B_W = []
        for angle in np.linspace(0.0/180.0*math.pi, -180.0/180.0*math.pi, 10):
            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
            self.key_traj1_T_B_W.append( (T_B_W, angle) )
        self.key_traj2_T_B_W = []
        for angle in np.linspace(0.0/180.0*math.pi, 180.0/180.0*math.pi, 10):
            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
            self.key_traj2_T_B_W.append( (T_B_W, angle) )
        self.velma_solvers = velmautils.VelmaSolvers()

        self.dof_names = [
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "right_arm_4_joint",
        "right_arm_5_joint",
        "right_arm_6_joint",
        "torso_0_joint",
        ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_names_ik = [
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "right_arm_4_joint",
        "right_arm_5_joint",
        "right_arm_6_joint",
        ]

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

        self.free_dof_idx = self.dof_indices_map[self.openrave.free_joint["right_arm"]]
        self.torso_0_joint_idx = self.dof_indices_map["torso_0_joint"]
        self.ignore_dof = [
        self.dof_indices_map["torso_0_joint"],
        self.dof_indices_map["right_arm_0_joint"],
        self.dof_indices_map["right_arm_1_joint"],
        self.dof_indices_map["right_arm_2_joint"],
        self.dof_indices_map["right_arm_3_joint"],
        self.dof_indices_map["right_arm_4_joint"],
        self.dof_indices_map["right_arm_5_joint"],
        self.dof_indices_map["right_arm_6_joint"]]

    def GetDofLimits(self):
        return self.dof_limits

    def GetDofIndices(self):
        return self.dof_indices

    def GetDofNames(self):
        return self.dof_names

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
        # interpolate trajectory (in the cartesian space)
        self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
        link_E = self.openrave.robot_rave.GetLink("right_HandPalmLink")
        T_World_E = conv.OpenraveToKDL(link_E.GetTransform())
        T_B_O = self.openrave.T_World_Br.Inverse() * T_World_E * self.T_E_O
        diff = PyKDL.diff(self.T_B_O_nearHole, T_B_O)
        if diff.vel.Norm() > 0.02 or diff.rot.Norm() > 10.0/180.0*math.pi:
            return False

        angle1 = 0.0
        for T_B_W, angle in self.key_traj1_T_B_W:
            init_js = self.openrave.getRobotConfigurationRos()
            traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_W)
            if traj == None:
                break
            angle1 = angle
            qar = {}
            for qi in range(len(traj[-1])):
                qar["right_arm_"+str(qi)+"_joint"] = traj[-1][qi]
            self.openrave.updateRobotConfigurationRos(qar)

        self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
        angle2 = 0.0
        for T_B_W, angle in self.key_traj2_T_B_W:
            init_js = self.openrave.getRobotConfigurationRos()
            traj = self.velma_solvers.getCartImpWristTraj(init_js, T_B_W)
            if traj == None:
                break
            angle2 = angle
            qar = {}
            for qi in range(len(traj[-1])):
                qar["right_arm_"+str(qi)+"_joint"] = traj[-1][qi]
            self.openrave.updateRobotConfigurationRos(qar)

        if abs(angle1-angle2) > 190.0/180.0*math.pi:
            return True
        else:
            return False

    def SampleGoal(self, start_q, shortest_path_len):
        self.openrave.switchCollisionModel("velmasimplified0")

        start_arm_q = np.empty(len(self.dof_names_ik))
        for dof_ik_idx in range(len(self.dof_names_ik)):
            start_arm_q[dof_ik_idx] = start_q[self.dof_indices_map[self.dof_names_ik[dof_ik_idx]]]

        T_B_E = self.T_B_O_nearHole * self.T_O_E

        q_goal = np.zeros(len(self.dof_names))
        for tries in range(50):
            # random free joint value for the arm
            q_goal[self.free_dof_idx] = random.uniform(self.dof_limits[self.free_dof_idx][0]+0.01, self.dof_limits[self.free_dof_idx][1]-0.01)
            freevalues = [ (q_goal[self.free_dof_idx]-self.dof_limits[self.free_dof_idx][0])/(self.dof_limits[self.free_dof_idx][1]-self.dof_limits[self.free_dof_idx][0]) ]

            # random torso joint value
            q_goal[self.torso_0_joint_idx] = random.uniform(self.dof_limits[self.torso_0_joint_idx][0]+0.01, self.dof_limits[self.torso_0_joint_idx][1]-0.01)

            self.openrave.robot_rave.SetDOFValues(q_goal, self.dof_indices)
            solutions = self.openrave.findIkSolutions(T_B_E, man_name="right_arm", freevalues=freevalues)

#            solutions_dist = []
#            # sort the solutions
#            for sol in solutions:
#                dist = np.linalg.norm(start_arm_q-np.array(sol))
#                solutions_dist.append( (dist, sol) )

#            sorted_solutions = sorted(solutions_dist, key=operator.itemgetter(0))

            goal_list = []

#            for dist, sol in sorted_solutions:
            for sol in solutions:
                for arm_dof_idx in range(len(self.dof_names_ik)):
                    dof_name = self.dof_names_ik[arm_dof_idx]
                    q_goal[self.dof_indices_map[dof_name]] = sol[arm_dof_idx]

                if shortest_path_len == None:
                    for i in range(len(self.dof_names)):
                        if i in self.ignore_dof:
                            continue
                        q_goal[i] = random.uniform(self.dof_limits[i][0]+0.01, self.dof_limits[i][1]-0.01)
                else:
                    diff = 0.0
                    for dof_idx in self.ignore_dof:
                        diff += (start_q[dof_idx] - q_goal[dof_idx]) * (start_q[dof_idx] - q_goal[dof_idx])
                    shortest_path_len2 = shortest_path_len*shortest_path_len - diff
                    if shortest_path_len2 < 0.0:
                        continue
                    shortest_path_len2 = math.sqrt(shortest_path_len2)
                    q_goal2 = tree.uniformInBall(shortest_path_len2, self.dof_limits, start_q, ignore_dof=self.ignore_dof)
                    for dof_idx in self.ignore_dof:
                        q_goal2[dof_idx] = q_goal[dof_idx]
                    q_goal = q_goal2

                # sanity check
                if shortest_path_len != None and np.linalg.norm(q_goal-start_q) > shortest_path_len:
                    print "ERROR: np.linalg.norm(q_goal-start_q) > shortest_path_len", np.linalg.norm(q_goal-start_q), ">", shortest_path_len
                    exit(0)

                if self.checkGoal(q_goal):
                    goal_list.append(q_goal)
#                    return q_goal
            if len(goal_list) > 0:
                return goal_list
        return None

class MoveArmsCloseTaskRRT:
    def __init__(self, openrave, args):
        self.openrave = openrave

        self.dof_names = [
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        ]

        self.dof_tol = [
        (1.8, 2.3),
        (1.9, 2.06),
        (-1.2, -1.0),
        (-2.06, -1.9),
        (0.7, 1.2),
        (-2.06, -1.9),
        (1.0, 1.2),
        (1.9, 2.06),
        ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

    def GetDofLimits(self):
        return self.dof_limits

    def GetDofIndices(self):
        return self.dof_indices

    def GetDofNames(self):
        return self.dof_names

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
        for q_idx in range(len(q)):
            if q[q_idx] < self.dof_tol[q_idx][0] or q[q_idx] > self.dof_tol[q_idx][1]:
                return False
        return True

    def SampleGoal(self, start_q, shortest_path_len):
            q_goal = np.empty(len(self.dof_names))
            for i in range(len(self.dof_names)):
                q_goal[i] = random.uniform(self.dof_tol[i][0], self.dof_tol[i][1])
            return [q_goal]

class GraspTaskRRT:
    def __init__(self, openrave, args):
        if len(args) != 2:
            raise ValueError('GraspTaskRRT: wrong number of arguments: ' + str(len(args)))

        if args[0] != "left" and args[0] != "right":
            raise ValueError('GraspTaskRRT: wrong argument[0] value: ' + args[0])

        if len(args[1]) == 0:
            raise ValueError('GraspTaskRRT: wrong argument[1] value')

        self.openrave = openrave

#        def getGripperPose():
#            return PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.6, 0, 1.6))

        self.gripper = args[0]
        self.T_B_E_list = args[1]

#        self.T_E_O = PyKDL.Frame()
#        self.T_O_E = self.T_E_O.Inverse()
#        self.key_axis_O = PyKDL.Vector(0,0,1)
#        self.key_up_O = PyKDL.Vector(1,0,0)
#        self.key_side_O = self.key_axis_O * self.key_up_O

#        self.key_endpoint_O = PyKDL.Vector(0.000256401261281, -0.000625166847342, 0.232297442735)
#        self.T_B_O_nearHole = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.71891504857, -0.0529880479354, 0.691118088949, 0.0520500417212), PyKDL.Vector(0.883081316461, -0.100813768303, 0.95381559114))

        # get the transformation from wrist to palm
        link_E = self.openrave.robot_rave.GetLink(self.gripper + "_HandPalmLink")
        link_W = self.openrave.robot_rave.GetLink(self.gripper + "_arm_7_link")
        T_World_E = conv.OpenraveToKDL(link_E.GetTransform())
        T_World_W = conv.OpenraveToKDL(link_W.GetTransform())
        self.T_W_E = T_World_W.Inverse() * T_World_E
        self.T_E_W = self.T_W_E.Inverse()

#        self.key_traj1_T_B_W = []
#        for angle in np.linspace(0.0/180.0*math.pi, -180.0/180.0*math.pi, 10):
#            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
#            self.key_traj1_T_B_W.append( (T_B_W, angle) )
#        self.key_traj2_T_B_W = []
#        for angle in np.linspace(0.0/180.0*math.pi, 180.0/180.0*math.pi, 10):
#            T_B_W = self.T_B_O_nearHole * PyKDL.Frame(PyKDL.Rotation.Rot(self.key_axis_O, angle)) * self.T_O_E * self.T_E_W
#            self.key_traj2_T_B_W.append( (T_B_W, angle) )
#        self.velma_solvers = velmautils.VelmaSolvers()

        self.dof_names = [
        "left_arm_0_joint",
        "left_arm_1_joint",
        "left_arm_2_joint",
        "left_arm_3_joint",
        "right_arm_0_joint",
        "right_arm_1_joint",
        "right_arm_2_joint",
        "right_arm_3_joint",
        "torso_0_joint",
        ]

        if self.gripper == "right":
            self.dof_names += [
            "right_arm_4_joint",
            "right_arm_5_joint",
            "right_arm_6_joint",
            ]
        else:
            self.dof_names += [
            "left_arm_4_joint",
            "left_arm_5_joint",
            "left_arm_6_joint",
            ]

        self.dof_indices = []
        self.dof_limits = []
        for joint_name in self.dof_names:
            joint = openrave.robot_rave.GetJoint(joint_name)
            self.dof_indices.append( joint.GetDOFIndex() )
            lim_lo, lim_up = joint.GetLimits()
            self.dof_limits.append( (lim_lo[0], lim_up[0]) )

        self.dof_names_ik = []
        for i in range(7):
            self.dof_names_ik.append( self.gripper + "_arm_" + str(i) + "_joint" )

        self.dof_indices_map = {}
        for i in range(len(self.dof_names)):
            self.dof_indices_map[self.dof_names[i]] = i

        self.free_dof_idx = self.dof_indices_map[self.openrave.free_joint[self.gripper + "_arm"]]
        self.torso_0_joint_idx = self.dof_indices_map["torso_0_joint"]
        self.ignore_dof = [ self.dof_indices_map["torso_0_joint"] ]
        for dof_name in self.dof_names_ik:
            self.ignore_dof.append( self.dof_indices_map[dof_name] )

        self.goals_T_B_E = []

    def GetDofLimits(self):
        return self.dof_limits

    def GetDofIndices(self):
        return self.dof_indices

    def GetDofNames(self):
        return self.dof_names

    def getActiveDOF(self, q):
        q_ret = np.empty(len(self.dof_indices))
        q_ret_idx = 0
        for dof_idx in self.dof_indices:
            q_ret[q_ret_idx] = q[dof_idx]
            q_ret_idx += 1
        return q_ret

    def checkGoal(self, q):
        # interpolate trajectory (in the cartesian space)
        self.openrave.robot_rave.SetDOFValues(q, self.dof_indices)
        link_E = self.openrave.robot_rave.GetLink("right_HandPalmLink")
        T_World_E = conv.OpenraveToKDL(link_E.GetTransform())
        T_B_E = self.openrave.T_World_Br.Inverse() * T_World_E

        for T_B_Eg in self.goals_T_B_E:
            diff = PyKDL.diff(T_B_Eg, T_B_E)
            if diff.vel.Norm() <= 0.02 and diff.rot.Norm() <= 10.0/180.0*math.pi:
                return True

        return False

    def SampleGoal(self, start_q, shortest_path_len):
        self.openrave.switchCollisionModel("velmasimplified0")

        start_arm_q = np.empty(len(self.dof_names_ik))
        for dof_ik_idx in range(len(self.dof_names_ik)):
            start_arm_q[dof_ik_idx] = start_q[self.dof_indices_map[self.dof_names_ik[dof_ik_idx]]]

        T_B_Ed = self.T_B_E_list[random.randint(0, len(self.T_B_E_list)-1)]

        q_goal = np.zeros(len(self.dof_names))
        for tries in range(50):
            # random free joint value for the arm
            q_goal[self.free_dof_idx] = random.uniform(self.dof_limits[self.free_dof_idx][0]+0.01, self.dof_limits[self.free_dof_idx][1]-0.01)
            freevalues = [ (q_goal[self.free_dof_idx]-self.dof_limits[self.free_dof_idx][0])/(self.dof_limits[self.free_dof_idx][1]-self.dof_limits[self.free_dof_idx][0]) ]

            # random torso joint value
            q_goal[self.torso_0_joint_idx] = random.uniform(self.dof_limits[self.torso_0_joint_idx][0]+0.01, self.dof_limits[self.torso_0_joint_idx][1]-0.01)

            self.openrave.robot_rave.SetDOFValues(q_goal, self.dof_indices)
            solutions = self.openrave.findIkSolutions(T_B_Ed, man_name=self.gripper+"_arm", freevalues=freevalues)
            if len(solutions) == 0:
                continue

#            solutions_dist = []
#            # sort the solutions
#            for sol in solutions:
#                dist = np.linalg.norm(start_arm_q-np.array(sol))
#                solutions_dist.append( (dist, sol) )
#            sorted_solutions = sorted(solutions_dist, key=operator.itemgetter(0))

            goal_list = []
#            for dist, sol in sorted_solutions:
            for sol in solutions:
                for arm_dof_idx in range(len(self.dof_names_ik)):
                    dof_name = self.dof_names_ik[arm_dof_idx]
                    q_goal[self.dof_indices_map[dof_name]] = sol[arm_dof_idx]

                if shortest_path_len == None:
                    # somtimes search for a goal much closer
#                    if random.uniform(0,1) < 0.5:
#                        q_goal = tree.uniformInBall(5.0, self.dof_limits, start_q, ignore_dof=self.ignore_dof)
#                    else:
                    for i in range(len(self.dof_names)):
                            if i in self.ignore_dof:
                                continue
                            q_goal[i] = random.uniform(self.dof_limits[i][0]+0.01, self.dof_limits[i][1]-0.01)
                else:
                    diff = 0.0
                    for dof_idx in self.ignore_dof:
                        diff += (start_q[dof_idx] - q_goal[dof_idx]) * (start_q[dof_idx] - q_goal[dof_idx])
                    shortest_path_len2 = shortest_path_len*shortest_path_len - diff
                    if shortest_path_len2 < 0.0:
                        continue
                    shortest_path_len2 = math.sqrt(shortest_path_len2)

                    # somtimes search for a goal much closer
#                    if random.uniform(0,1) < 0.5:
#                        if shortest_path_len2 > 10.0:
#                            shortest_path_len2 = 10.0
#                        shortest_path_len2 *= 0.5
                    q_goal2 = tree.uniformInBall(shortest_path_len2, self.dof_limits, start_q, ignore_dof=self.ignore_dof)
                    for dof_idx in self.ignore_dof:
                        q_goal2[dof_idx] = q_goal[dof_idx]
                    q_goal = q_goal2

                # sanity check
                if shortest_path_len != None and np.linalg.norm(q_goal-start_q) > shortest_path_len:
                    print "ERROR: np.linalg.norm(q_goal-start_q) > shortest_path_len", np.linalg.norm(q_goal-start_q), ">", shortest_path_len
                    exit(0)

                self.goals_T_B_E.append(T_B_Ed)
                goal_list.append(q_goal)
#                return q_goal
            if len(goal_list) > 0:
                return goal_list
        return None

