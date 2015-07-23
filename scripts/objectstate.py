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
        self.last_update = None
        self.T_W_O = None
        self.enabled = False

class MovableObjectsState:
    def __init__(self, env, filename_list):
        self.current_vis_set = set()
        self.obj_map = {}
        for filename in filename_list:
            obj = ObjectState()
            print filename
            obj.name = env.ReadKinBodyXMLFile(filename).GetName()
            print obj.name
            self.obj_map[obj.name] = obj

    def update(self, tf_listener, time_tf):
        added = []
        removed = []
        for obj_name in self.obj_map:
            obj = self.obj_map[obj_name]
            if tf_listener.canTransform('world', obj_name, time_tf):
                pose = tf_listener.lookupTransform('world', obj_name, time_tf)
                obj.T_W_O = pm.fromTf(pose)
                if not obj_name in self.current_vis_set:
                    added.append(obj_name)
                    self.current_vis_set.add(obj_name)
                    obj.enabled = True
                obj.last_update = time_tf
            else:
                if obj_name in self.current_vis_set and (time_tf - obj.last_update).to_sec() > 2.0:
                    removed.append(obj_name)
                    self.current_vis_set.remove(obj_name)
                    obj.enabled = False
        return added, removed

    def updateOpenrave(self, env):
        for obj_name in self.obj_map:
            obj = self.obj_map[obj_name]
            body = env.GetKinBody(obj_name)
            if obj_name in self.current_vis_set:
                if not body.IsEnabled():
                    body.Enable(True)
                    body.SetVisible(True)
                body.SetTransform(conv.KDLToOpenrave(obj.T_W_O))
            else:
                if body.IsEnabled():
                    body.Enable(False)
                    body.SetVisible(False)


#class MovableObjectsState:
#    def __init__(self, tf_listener, env, filename_list):
#        self.env = env
#        self.listener = tf_listener
#
#        self.obj_list = []
#        for filename in filename_list:
#            obj = ObjectState()
#            obj.kinbody = self.env.ReadKinBodyXMLFile(filename)
#            self.obj_list.append(obj)
#
#    def update(self, time_tf):
#        added = []
#        removed = []
#        for obj in self.obj_list:
#            obj_name = obj.kinbody.GetName()
#            if self.listener.canTransform('world', obj_name, time_tf):
#                pose = self.listener.lookupTransform('world', obj_name, time_tf)
#                obj.T_W_O = pm.fromTf(pose)
#                if obj.kinbody.GetEnvironmentId() == 0:
#                    obj.kinbody.SetTransform(conv.KDLToOpenrave(obj.T_W_O))
#                    self.env.Add(obj.kinbody)
#                    added.append(obj)
#                obj.last_update = time_tf
#            else:
#                if obj.kinbody.GetEnvironmentId() != 0 and (time_tf - obj.last_update).to_sec() > 2.0:
#                    self.env.Remove(obj.kinbody)
#                    removed.append(obj)
#        return added, removed

