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
import conversions as conv
import tf_conversions.posemath as pm

class ObjectState:
    def __init__(self):
        self.name = None
        self.name_col = None
        self.last_update = None
        self.T_W_O = None
        self.enabled = False

class MovableObjectsState:
    def __init__(self, env, filename_list):
        self.obj_map = {}
        col_map = {}
        for filename in filename_list:
            obj = ObjectState()
            obj.name = env.ReadKinBodyXMLFile(filename).GetName()
            if obj.name.endswith("_collision"):
                col_map[obj.name[:-len("_collision")]] = obj.name
                continue
            self.obj_map[obj.name] = obj

        for name in col_map:
            self.obj_map[name].name_col = col_map[name]

    def update(self, tf_listener, time_tf):
        added = []
        removed = []
        for obj_name in self.obj_map:
            obj = self.obj_map[obj_name]
            if tf_listener.canTransform('world', obj_name, time_tf):
                pose = tf_listener.lookupTransform('world', obj_name, time_tf)
                obj.T_W_O = pm.fromTf(pose)
                if not obj.enabled:
                    added.append(obj_name)
                    if obj.name_col != None:
                        added.append(obj.name_col)
                    obj.enabled = True
                obj.last_update = time_tf
            else:
                if obj.enabled and (time_tf - obj.last_update).to_sec() > 2.0:
                    removed.append(obj_name)
                    if obj.name_col != None:
                        removed.append(obj.name_col)
                    obj.enabled = False
        return added, removed

    def updateOpenrave(self, env):
        for obj_name in self.obj_map:
            obj = self.obj_map[obj_name]
            body = env.GetKinBody(obj_name)
            if obj.name_col != None:
                body_col = env.GetKinBody(obj.name_col)

            if obj.enabled:
                if not body.IsEnabled():
                    body.Enable(True)
                    body.SetVisible(True)
                    if obj.name_col != None:
                        body_col.Enable(True)
                        body_col.SetVisible(True)
                body.SetTransform(conv.KDLToOpenrave(obj.T_W_O))
                if obj.name_col != None:
                    body_col.SetTransform(conv.KDLToOpenrave(obj.T_W_O))
            else:
                if body.IsEnabled():
                    body.Enable(False)
                    body.SetVisible(False)
                    if obj.name_col != None:
                        body_col.Enable(False)
                        body_col.SetVisible(False)

