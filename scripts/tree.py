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

import numpy as np
import random

def uniformInBall(r, limits, q_start, ignore_dof=[]):
                tries = 0
                while True:
                    tries += 1
                    n = len(limits)
                    vec = np.empty(n)
                    for i in range(n):
                        if i in ignore_dof:
                            vec[i] = q_start[i]
                        else:
                            lo_limit = max(limits[i][0], q_start[i]-r)
                            up_limit = min(limits[i][1], q_start[i]+r)
                            vec[i] = random.uniform(lo_limit, up_limit)
                    diff = q_start - vec
                    diff_len = np.linalg.norm(q_start - vec)
                    if diff_len > r:
                        diff = diff/diff_len * r * random.uniform(0,1)
                        vec = q_start - diff
                    return vec

def Distance(q1, q2):
                return np.linalg.norm(q1-q2)

def GetPath(E, q_idx):
                if not q_idx in E:
                    return [q_idx]
                parent_idx = E[q_idx]
                q_list = GetPath(E, parent_idx)
                q_list.append(q_idx)
                return q_list

def CostLine(q1, q2):
                cost = Distance(q1, q2)# * (np.linalg.norm(q1-self.q_init) + np.linalg.norm(q2-self.q_init)) * 0.5
                return cost

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

