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

import PyKDL
import math
import numpy as np

class HeadKinematics:

    def __init__(self, h_rot_, h_lean_, h_head_, h_cam_, v_cam_):
        self.h_rot = h_rot_
        self.h_lean = h_lean_
        self.h_head = h_head_
        self.h_cam = h_cam_
        self.v_cam = v_cam_
        self.torso_joint = [0.0, 0.0]

    def UpdateTorsoPose(self, joint1, joint2):
        self.torso_joint[0] = joint1
        self.torso_joint[1] = joint2 + math.pi / 2

    def UpdateTargetPosition(self, xx, yy, zz):
        self.xb = xx
        self.yb = yy
        self.zb = zz

    def UpdateTargetPositionHeadFrame(self, xx, yy, zz):
        self.xh = xx
        self.yh = yy
        self.zh = zz

    def TransformTargetToHeadFrame(self):
        #double x1, y1, z1;  // Target coordinates with respect to torso rotating link
        #// double xh, yh, zh; // Target coordinates with respect to head frame (after torso lean link)
      
        x1 = self.xb * math.cos(self.torso_joint[0]) + self.yb * math.sin(self.torso_joint[0])
        y1 = -self.xb * math.sin(self.torso_joint[0]) + self.yb * math.cos(self.torso_joint[0])
        z1 = self.zb - self.h_rot;

        self.xh = x1 - self.h_lean * math.sin(self.torso_joint[1])
        self.yh = y1;
        self.zh = z1 - self.h_lean * math.cos(self.torso_joint[1]) - self.h_head

    def CalculateHeadPose(self):
        #double theta1, theta2;
        #double alpha;
        #double beta;
        #double x1, y1, z1;

        # Theta1
        if self.xh == 0 and self.yh == 0:
            return None, None
        if self.xh*self.xh + self.yh*self.yh < self.v_cam*self.v_cam:
            return None, None
        theta1 = math.asin(self.h_cam / math.sqrt(self.xh*self.xh+self.yh*self.yh)) + math.atan2(self.yh, self.xh)

        x1 = self.xh / math.cos(theta1)
        z1 = self.zh

        # Theta2
        if x1 == 0 and z1 == 0:
            return None, None
        alpha = math.atan2(x1, -z1);

        beta = math.acos(self.v_cam / math.sqrt(self.xh * self.xh + self.yh * self.yh + self.zh * self.zh))

        theta2 = math.pi - (alpha + beta)

        joint_pan = theta1
        joint_tilt = theta2
        return joint_pan, joint_tilt


