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
import random
import itertools
import operator
import rospkg
from scipy import optimize
import re

def opened(ws, ob):
    if ws.typeMatch(ob[0], 'Door'):
        assert "state" in ob[1]
        return ob[1]["state"] == "opened"
    elif ws.typeMatch(ob[0], 'Jar'):
        assert "cap" in ob[1]
        return ob[1]["cap"] == None
    elif ws.typeMatch(ob[0], 'Container'):
        if "door1" in ob[1]:
            door1_name = ob[1]["door1"]
        else:
            door1_name = None
        if "door2" in ob[1]:
            door2_name = ob[1]["door2"]
        else:
            door2_name = None
        if (door1_name == None or opened(ws, ws.getObject(door1_name))) and (door2_name == None or opened(ws, ws.getObject(door2_name))):
            return True
        else:
            return False

    raise TypeError("wrong type in predicate opened: " + ob[0])

def reachable(ws, o):
    if o[0] == 'Object':
        assert "pose" in o[1]
        str_list = o[1]["pose"].split()
        if str_list[0] == "inside":
            cont_name = str_list[1]
            cont = ws.getObject(cont_name)
            if "door1" in cont[1]:
                door1_name = cont[1]["door1"]
            else:
                door1_name = None
            if "door2" in cont[1]:
                door2_name = cont[1]["door2"]
            else:
                door2_name = None
            if (door1_name == None or opened(ws, ws.getObject(door1_name))) and (door2_name == None or opened(ws, ws.getObject(door2_name))):
                return True
            else:
                return False
        else:
            return True
    raise TypeError("wrong types in predicate reachable: " + o[0] + " " + c[0])

def inside(ws, o1, o2):
    if ws.typeMatch(o1[0],'Object') and ws.typeMatch(o2[0],'Container'):
        assert "pose" in o1[1]
        str_list = o1[1]["pose"].split()
        if str_list[0] == "inside":
            cont_name = str_list[1]
            if cont_name == o2[1]["name"]:
                return True
            else:
                return False
        else:
            return False
    raise TypeError("wrong types in predicate inside: " + o1[0] + " " + o2[0])

def part_of(ws, d, o):
    if d[0] == 'Door' and o[0] == 'Container':
        if o[1]["door1"] == d[1]["name"] or o[1]["door2"] == d[1]["name"]:
            return True
        else:
            return False
    raise TypeError("wrong types in predicate part_of: " + d[0] + " " + o[0])

def free(ws, ob):
    if ob[0] == 'Manipulator':
        assert "grasped_name" in ob[1]
        return ob[1]["grasped_name"] == None
    raise TypeError("wrong type in predicate free: " + ob[0])

def grasped(ws, ob1, ob2):
    if ob1[0] == 'Manipulator' and ob2[0] == 'Door':
        assert "grasped_name" in ob1[1]
        assert "name" in ob2[1]
        return ob1[1]["grasped_name"] == ob2[1]["name"]
    elif ob1[0] == 'Manipulator' and ob2[0] == 'Object':
        assert "grasped_name" in ob1[1]
        assert "name" in ob2[1]
        return ob1[1]["grasped_name"] == ob2[1]["name"]
    raise TypeError("wrong types in predicate grasped: " + ob1[0] + " " + ob2[0])

def conf_feasible(ws, ob):
    if ob[0] == 'Manipulator':
        assert "conf_feasible" in ob[1]
        return ob[1]["conf_feasible"]
    raise TypeError("wrong type in predicate conf_feasible: " + ob[0])

def ajar(ws, ob):
    if ob[0] == 'Door':
        assert "state" in ob[1]
        return ob[1]["state"] == "ajar"
    raise TypeError("wrong type in predicate ajar: " + ob[0])

def closed(ws, ob):
    if ob[0] == 'Door':
        assert "state" in ob[1]
        return ob[1]["state"] == "closed"
    elif ws.typeMatch(ob[0], 'Jar'):
        assert "cap" in ob[1]
        return ob[1]["cap"] != None
    elif ws.typeMatch(ob[0],'Container'):
        if "door1" in ob[1]:
            door1_name = ob[1]["door1"]
        else:
            door1_name = None
        if "door2" in ob[1]:
            door2_name = ob[1]["door2"]
        else:
            door2_name = None
        if (door1_name == None or closed(ws, ws.getObject(door1_name))) and (door2_name == None or closed(ws, ws.getObject(door2_name))):
            return True
        else:
            return False

    raise TypeError("wrong type in predicate closed: " + ob[0])

def pose_certain(ws, ob):
    if ws.typeMatch(ob[0], 'VerticalPlane'):
        assert "pose_certain" in ob[1]
        return ob[1]["pose_certain"]
    raise TypeError("wrong type in predicate pose_certain: " + ob[0])

def at_pose_above(ws, ob1, ob2):
    if ws.typeMatch(ob1[0], 'Object') and ws.typeMatch(ob2[0], 'Object'):
        str_list = ob1[1]["pose"].split()
        if str_list[0] == "above":
            above_name = str_list[1]
            if above_name == ob2[1]["name"]:
                return True
            else:
                return False
        else:
            return False
    raise TypeError("wrong types in predicate at_pose_above: " + ob1[0] + " " + ob2[0])

def at_pose_inside(ws, ob1, ob2):
    if ws.typeMatch(ob1[0], 'Object') and ws.typeMatch(ob2[0], 'Container'):
        str_list = ob1[1]["pose"].split()
        if str_list[0] == "inside":
            inside_name = str_list[1]
            if inside_name == ob2[1]["name"]:
                return True
            else:
                return False
        else:
            return False
    raise TypeError("wrong types in predicate at_pose_inside: " + ob1[0] + " " + ob2[0])

def clear_on(ws, ob):
    return True

# arguments: expr_str, parameters
# returned value:
# dictionary: pred_str:(pred_name, [arg1_name, arg2_name,...], [arg1_type, arg2_type,...]):
def extractPredicatesAbst(expr_str, parameters):
        assert parameters != None
        result = {}
        e = 0
        while True:
            s = expr_str.find("[", e)
            if s < 0:
                break
            e = expr_str.find("]", s)+1
            if expr_str[s+1:e-1] in result:
                continue
            exp = expr_str[s+1:e-1].split()
            pred_name = exp[0]
            arg_name = []
            arg_type = []
            for idx in range(1, len(exp)):
                assert exp[idx][0] == "?"
                arg_name.append(exp[idx])
                arg_type.append( parameters[exp[idx]] )
            result[expr_str[s+1:e-1]] = (pred_name, arg_name, arg_type)
        return result

class Predicate:

    def __init__(self, pred_str):
        self.pred_str = pred_str
        pred_words = self.pred_str.split()
        assert len(pred_words) > 1
        self.pred_name = pred_words[0]
        self.pred_args = self.pred_str[len(self.pred_name):]
        self.pred_objs = pred_words[1:]

    def getName(self):
        return self.pred_name

    def getStr(self):
        return self.pred_str

    def getArgs(self):
        return self.pred_args

    def getObjs(self):
        return self.pred_objs

    def getTypes(self, world_state):
        return self.pred_objs

# arguments: expr_str, parameters
# returned value:
# dictionary: pred_str:(pred_name, [obj1_name, obj2_name,...], [arg1_name, arg2_name,...], [arg1_type, arg2_type,...]):
def extractPredicates(expr_str):
        result = []
        e = 0
        while True:
            s = expr_str.find("[", e)
            if s < 0:
                break
            e = expr_str.find("]", s)+1
            if expr_str[s+1:e-1] in result:
                continue
            result.append(Predicate(expr_str[s+1:e-1]))
        return result

def getAllPossibilities(goal_exp, pred_list):
        # get all required conditions that archieve the goal expression
        goal_product = itertools.product([True, False], repeat=len(pred_list))
        goal_cases = []
        for subset in goal_product:
            goal_str = goal_exp
            i = 0
            case = []
            for pred in pred_list:
                goal_str = goal_str.replace("["+pred.getStr()+"]", str(subset[i]))
                i += 1
            if eval(goal_str):
                goal_cases.append(subset)
        return goal_cases

def substitute(exp_str, subst_map):
    result = exp_str
    for s in subst_map:
        result = result.replace(s, subst_map[s])
    return result

def generateSubstitutionCases(obj_type_map, parameters, substitutions):

                # get other substitutions
                subst2 = {}
                subst2_inv = {}
                type_pool = {}
                for var in parameters:
                    if not var in substitutions:
                        var_type = parameters[var]
                        subst2[var] = var_type
                        if not var_type in subst2_inv:
                            subst2_inv[var_type] = [var]
                        else:
                            subst2_inv[var_type].append(var)
                        if not var_type in type_pool:
                            objs = obj_type_map[var_type]
                            type_pool[var_type] = []
                            for obj in objs:
                                obj_used = False
                                for vv in substitutions:
                                    if obj == substitutions[vv]:
                                        obj_used = True
                                if not obj_used:
                                    type_pool[var_type].append(obj)
                                else:
                                    pass
                                    #assert False    # ok

                ll = []
                ll_types = []
                # generate cases for substitution
                subst_cases = []
                for var_type in subst2_inv:
                    list_a = []
                    x = itertools.combinations( type_pool[var_type], len(subst2_inv[var_type]) )
                    for elem in x:
                        p = itertools.permutations(elem)
                        for e in p:
                            list_a.append(e)
                    ll.append( list_a )
                    ll_types.append(var_type)
                prod = itertools.product(*ll)
                for e in prod:
                    s_case = {}
                    for ti in range(len(ll_types)):
                        type_name = ll_types[ti]
                        for vi in range(len(e[ti])):
                            subst_var = subst2_inv[type_name][vi]
                            subst_dest = e[ti][vi]
                            s_case[subst_var] = subst_dest
                    subst_cases.append(s_case)
                return subst_cases



class PredicatesContainer:

    def __init__(self):
        predicates = [free, grasped, conf_feasible, opened, ajar, closed, pose_certain, inside, reachable, part_of, at_pose_above, clear_on, at_pose_inside]
        self.predicates_map = {}
        for pred in predicates:
            self.predicates_map[pred.__name__] = pred

    def getPred(self, world_state, name, args):
        arg_names = args.split()
        arg_list = []
        for arg in arg_names:
            arg_list.append( world_state.getObject(arg) )
        return self.predicates_map[name](world_state, *arg_list)

class WorldState:

    def __init__(self):
        self.objects_t_n_map = {}
        self.objects_n_t_map = {}
        self.objects_n_map = {}

        self.types_t_b_map = {}
        self.types_t_attr_map = {}

    def addType(self, type_name, base_types, attributes):
        assert not type_name in self.types_t_b_map
        assert not type_name in self.types_t_attr_map
        self.types_t_b_map[type_name] = base_types
        self.types_t_attr_map[type_name] = attributes

    def addGeometricType(self, type_name):
        pass

    def simulateAction(self, a, args):
        arg_names = args.split()
        arg_list = []
        for arg in arg_names:
            arg_list.append( self.objects_n_map[arg] )
        return a.actionSim(*arg_list)

    def addObject(self, obj_type, obj_name, obj_parameters):
        assert obj_type in self.types_t_b_map
        assert not obj_name in self.objects_n_map
        assert not "name" in obj_parameters
        if not obj_type in self.objects_t_n_map:
            self.objects_t_n_map[obj_type] = []
        self.objects_t_n_map[obj_type].append( obj_name )

        for attr in self.types_t_attr_map[obj_type]:
            if not attr in obj_parameters:
                raise TypeError("missing attribute " + attr + " in object " + obj_name + " of class " + obj_type)

        for tb in self.types_t_b_map[obj_type]:
            for attr in self.types_t_attr_map[tb]:
                if not attr in obj_parameters:
                    raise TypeError("missing attribute " + attr + " in object " + obj_name + " of class " + tb)

        for tb in self.types_t_b_map[obj_type]:
            if not tb in self.objects_t_n_map:
                self.objects_t_n_map[tb] = []
            self.objects_t_n_map[tb].append( obj_name )
            
        obj_parameters["name"] = obj_name
        self.objects_n_map[obj_name] = (obj_type, obj_parameters)
        self.objects_n_t_map[obj_name] = obj_type

    def getObject(self, obj_name):
        assert obj_name in self.objects_n_map
        return self.objects_n_map[obj_name]

    def getObjectType(self, obj_name):
        assert obj_name in self.objects_n_t_map
        return self.objects_n_t_map[obj_name]

    def typeMatch(self, type_name, base_type):
        assert type_name in self.types_t_b_map
        if base_type == type_name:
            return True
        else:
            for t in self.types_t_b_map[type_name]:
                if self.typeMatch(t, base_type):
                    return True
        return False

    def getEffect(self, action, pred_name, pred_objs, pred_value):
        for pred_str in action.effect_map:
            substitutions = {}
            if action.effect_map[pred_str][0] == pred_name:
                match = True
                for arg_i in range(len(pred_objs)):
                    if not self.typeMatch(self.getObjectType(pred_objs[arg_i]), action.effect_map[pred_str][2][arg_i]):
                        match = False
                        break
                    substitutions[ action.effect_map[pred_str][1][arg_i] ] = pred_objs[arg_i]
                if match and pred_value:
                    return substitutions
        return None

class Action:

    def __init__(self, name, parameters, relations, precondition, effect, rt_failure, action_sim):
        self.name = name

        par = parameters.split(",")
        self.parameters = {}
        self.param_list = []
        for p in par:
            decl = p.split()
            self.parameters[decl[0]] = decl[1]
            self.param_list.append( decl[0] )

        self.relations = relations

        self.precondition = precondition
        self.rt_failure = rt_failure

        self.effect_map = extractPredicatesAbst(effect, self.parameters)

        self.action_sim = action_sim

    def actionSim(self, *args):
        if self.action_sim != None:
            return self.action_sim(*args)
        return None

class Scenario:

    def __init__(self, init_state, goal, actions):
        self.init_state = copy.deepcopy(init_state)
        self.goal = copy.copy(goal)
        self.actions = actions
        self.action_map = {}
        for a in self.actions:
            self.action_map[a.name] = a

        self.steps = [
            (None, self.init_state, None, None),
            (self.goal, None, None, None) ]

    def processGoal(self, pred_list, pred_value_list, pc, world_state):
                assert len(pred_list) == len(pred_value_list)
                result_steps = []
                indent_str = " "
#                step_added = False
                for pred_idx in range(len(pred_list)):
                    predicate = pred_list[pred_idx]
                    pred_value = pred_value_list[pred_idx]
                    pred_name = predicate.getName()
                    pred_args = predicate.getArgs()
                    pred_objs = predicate.getObjs()

                    curr_value = pc.getPred(world_state, pred_name, pred_args)
                    print indent_str + " ", pred_value, "==", pred_name, pred_args, " (current value: ", curr_value, ")"

                    # the predicate is not yet satisfied
                    if curr_value != pred_value:
                        print indent_str + "  ", "not satisfied"
                        solution_found = False
                        for a in self.actions:
                            # get certain substitutions for a given action
                            substitutions = world_state.getEffect(a, pred_name, pred_objs, pred_value)
                            if substitutions != None:

                                action_found = True
                                print indent_str + "  ", a.name, substitutions
#                                print world_state.objects_t_n_map
                                subst_cases = generateSubstitutionCases(world_state.objects_t_n_map, a.parameters, substitutions)
                                case_found = False

                                for sc in subst_cases:
                                    # fork here: substitutions
                                    #sc = subst_cases[0]

                                    sc_all = sc.copy()
                                    sc_all.update(substitutions)
                                    print "sc_all", sc_all

                                    # check if the substitution satisfies the relations constraints
                                    relations = substitute(a.relations, sc_all)
#                                    print "relations", relations
                                    rel_pred = extractPredicates(relations)
#                                    print rel_pred
                                    for r_pred in rel_pred:
                                        r_pred_name = r_pred.getName()
                                        r_pred_args = r_pred.getArgs()
                                        print "getPred", r_pred_name, r_pred_args
                                        r_curr_value = pc.getPred(world_state, r_pred_name, r_pred_args)

                                        relations = relations.replace("["+r_pred.getStr()+"]", str(r_curr_value))
                                    if not eval(relations):
                                        continue

                                    new_state = copy.deepcopy(world_state)

                                    action_args = ""
                                    for par_name in a.param_list:
                                        obj_name = sc_all[par_name]
                                        action_args += obj_name + " "
                                    new_state.simulateAction(a, action_args)

                                    precond = substitute(a.precondition, sc_all)
                                    print "step added", a.name, precond
                                    result_steps.append( (precond, new_state, a.name, sc_all) )
                                    case_found = True
    #                                step_added = True
                                    break
                                print "case_found",case_found
                                if not case_found:
                                    continue

#                    if step_added:
#                        break
                return result_steps #step_added

    def process(self, pc):
        indent_str = " "

        all_c = 1
        while True:
            print "*"
            print "*"
            print "*"
            for s_idx in range(0, len(self.steps)-1):
                prev_step = self.steps[s_idx]
                next_step = self.steps[s_idx+1]
                goal = next_step[0]
                world_state = prev_step[1]
                print "iterating steps:"

                print "getAllPossibilities", goal
                pred_list = extractPredicates(goal)
                posi = getAllPossibilities(goal, pred_list)

                # fork here: goal variants
                p = posi[0]
                all_c *= len(posi)
#                step_added = False

                new_steps = self.processGoal(pred_list, posi[0], pc, world_state)
                if len(new_steps) > 0:
                    print "************* possible substitutions:"
                    for st in new_steps:
                        print st[2], st[3]
                    print "*************"
                    all_c *= len(new_steps)
                    step_added = True
                    self.steps.insert( s_idx+1, new_steps[0] )
                else:
                    step_added = False

                if step_added:
                    break
            if not step_added:
                break
            # post process all steps - propagate world changes
            for s_idx in range(0, len(self.steps)-1):
                prev_step = self.steps[s_idx]
                next_step = self.steps[s_idx+1]
                world_state = prev_step[1]
                action_name = next_step[2]
                sc_all = next_step[3]

                if action_name != None:
                    a = self.action_map[action_name]
                    new_state = copy.deepcopy(world_state)
                    action_args = ""
                    for par_name in a.param_list:
                        obj_name = sc_all[par_name]
                        action_args += obj_name + " "

                    new_state.simulateAction(a, action_args)
                    print "world state: " + a.name, sc_all
                    self.steps[s_idx+1] = (self.steps[s_idx+1][0], new_state, self.steps[s_idx+1][2], self.steps[s_idx+1][3])

            if rospy.is_shutdown():
                break


        print "steps:"
        for s in self.steps:
            print s[2], s[3]

        print "all_c", all_c
        return

class SymbolicPlanner:
    """
class for SymbolicPlanner
"""

    def __init__(self, pub_marker=None):
        pass

    def spin(self):

        pc = PredicatesContainer()

        ws = WorldState()
        ws.addType("Object", [], ["pose"])
        ws.addType("Container", ["Object"], ["door1", "door2"])
        ws.addType("Jar", ["Container"], ["cap"])
        ws.addType("VerticalPlane", [], ["pose_certain"])
        ws.addType("Door", ["VerticalPlane"], ["state", "parent"])
        ws.addType("Manipulator", [], ["grasped_name", "conf_feasible"])

#        ws.addGeometricType("Pose", [], [])
        ws.addObject("Container", "cabinet01", {"door1":"door_cab_l", "door2":"door_cab_r", "pose":None})
        ws.addObject("Door", "door_cab_l", {"state":"closed", "parent":"cabinet01", "pose_certain":False})
        ws.addObject("Door", "door_cab_r", {"state":"closed", "parent":"cabinet01", "pose_certain":False})
        ws.addObject("Container", "bowl01", {"door1":None, "door2":None, "pose":None})
        ws.addObject("Object", "jar_cap01", {"pose":"on jar01"})
        ws.addObject("Jar", "jar01", {"door1":None, "door2":None, "pose":"inside cabinet01", "cap":"jar_cap01"})
        ws.addObject("Manipulator", "man_l", {"grasped_name":None, "conf_feasible":True})
        ws.addObject("Manipulator", "man_r", {"grasped_name":None, "conf_feasible":True})
        ws.addObject("Object", "powder01", {"pose":"inside jar01"})

#        self.goal = "([opened ?door_cab_r] or (not [opened ?door_cab_r])) and [closed ?door_cab_l]"
#        self.goal = "[opened door_cab_r] and [free man_l]"
#        self.goal = "[opened door_cab_r] and [free man_l] and [grasped man_r jar]"
#        self.goal = "[grasped man_r jar]"
#        self.goal = "[closed door_cab_r] and [closed door_cab_l] and [free man_l] and [grasped man_r jar]"
        self.goal = "[inside powder01 bowl01] and [at_pose_inside jar01 cabinet01] and [closed cabinet01]"

        def a_pour_sim(c1, c2, s):
            assert c1 != None
            assert c2 != None
            assert s != None
            assert "pose" in s[1]
            assert "name" in c1[1]
            assert "name" in c2[1]
            assert s[1]["pose"] == ("inside " + c2[1]["name"])
            s[1]["pose"] = "inside " + c1[1]["name"]

        a_pour = Action("pour",
                "?c1 Container,?c2 Container,?s Object",
                "[inside ?s ?c2]",
                "[at_pose_above ?c2 ?c1] and [opened ?c1] and [opened ?c2]", #and [clear_on ?c1]
                "[inside ?s ?c1]",
                "",
                a_pour_sim)

        def a_transport_sim(o1, o2):
            assert o1 != None
            assert o2 != None
            assert "pose" in o1[1]
            assert "name" in o2[1]
            assert o1[1]["pose"] != ("above " + o2[1]["name"])
            o1[1]["pose"] = "above " + o2[1]["name"]

        a_transport = Action("transport",
                "?o1 Object,?o2 Object",
                "not [at_pose_above ?o1 ?o2]",
                "True",
                "[at_pose_above ?o1 ?o2]",
                "",
                a_transport_sim)

        def a_transport2_sim(o1, o2):
            assert o1 != None
            assert o2 != None
            assert "pose" in o1[1]
            assert "name" in o2[1]
            assert o1[1]["pose"] != ("inside " + o2[1]["name"])
            o1[1]["pose"] = "inside " + o2[1]["name"]

        a_transport2 = Action("transport2",
                "?o1 Object,?o2 Object",
                "not [at_pose_inside ?o1 ?o2]",
                "True",
                "[at_pose_inside ?o1 ?o2]",
                "",
                a_transport2_sim)

        def a_open_jar_sim(j):
            assert j != None
            assert "cap" in j[1]
            j[1]["cap"] = None

        a_open_jar = Action("open_jar",
                "?j Jar",
                "[closed ?j]",
                "True",
                "[opened ?j]",
                "",
                a_open_jar_sim)


        # unit tests
        assert pc.getPred(ws, "free", "man_l") == True
        assert pc.getPred(ws, "grasped", "man_l door_cab_l") == False
        assert pc.getPred(ws, "conf_feasible", "man_l") == True

        def a_explore_sim(d, m):
            assert d != None
            assert "pose_certain" in d[1]
            assert d[1]["pose_certain"] == False
            d[1]["pose_certain"] = True

        a_explore = Action("explore",
                "?vp VerticalPlane,?m Manipulator",
                None,
                "[free ?m] and (not [pose_certain ?vp])",
                "[pose_certain ?vp]",
                "",
                a_explore_sim)

        def a_grasp_door_sim(d, m):
            assert d != None
            assert m != None
            assert "grasped_name" in m[1]
#            assert m[1]["grasped_name"] == None
            assert "name" in d[1]
            m[1]["grasped_name"] = d[1]["name"]

        a_grasp_door = Action("grasp_door",
                "?d Door,?m Manipulator",
                None,
                "[free ?m] and [pose_certain ?d]",
                "[grasped ?m ?d]",
                "",
                a_grasp_door_sim)

        def a_ungrasp_door_sim(d, m):
            assert m != None
            m.is_free = True
            m.grasped_name = None

        a_ungrasp_door = Action("ungrasp_door",
                "?d Door,?m Manipulator",
                None,
                "[grasped ?m ?d]",
                "[free ?m]",
                "",
                a_ungrasp_door_sim)

        def a_ungrasp_sim(m):
            assert m != None
            assert "grasped_name" in m[1]
            assert m[1]["grasped_name"] != None
            m[1]["grasped_name"] = None

        a_ungrasp = Action("ungrasp",
                "?m Manipulator",
                None,
                "not [free ?m]",
                "[free ?m]",
                "",
                a_ungrasp_sim)

        def a_open_door_sim(d, m):
            assert d != None
            assert "state" in d[1]
            d[1]["state"] = "opened"

        a_open_door = Action("open_door",
                "?d Door,?m Manipulator",
                None,
                "[grasped ?m ?d] and [conf_feasible ?m] and ([closed ?d] or [ajar ?d])",
                "[opened ?d]",
                "[grasped ?m ?d] and (not [conf_feasible ?m]) and ([closed ?d] or [ajar ?d])",
                a_open_door_sim)

        def a_close_door_sim(d, m):
            assert d != None
            assert "state" in d[1]
            d[1]["state"] = "closed"

        a_close_door = Action("close_door",
                "?d Door,?m Manipulator",
                None,
                "[grasped ?m ?d] and [conf_feasible ?m] and ([opened ?d] or [ajar ?d])",
                "[closed ?d]",
                "",
                a_close_door_sim)

        def a_grasp_object_sim(o, m):
            assert o != None
            assert m != None
            assert "grasped_name" in m[1]
            assert m[1]["grasped_name"] == None
            assert "name" in o[1]
            m[1]["grasped_name"] = o[1]["name"]

        a_grasp_object = Action("grasp_object",
                "?o Object,?m Manipulator",
                None,
                "[free ?m] and [reachable ?o]",
                "[grasped ?m ?o]",
                "",
                a_grasp_object_sim)

        def a_uncover_sim(o1, o2, m):
            assert o1 != None
            assert o2 != None
            assert m != None
            m.is_free = False
            m.grasped_name = o.name

        a_uncover = Action("uncover",
                "?o Object,?c Container,?d1 Door,?d2 Door",
                None,
                "(not [reachable ?o]) and [inside ?o ?c] and [part_of ?d1 ?c] and [part_of ?d2 ?c] and [opened ?d1] and [opened ?d2]",
                "[reachable ?o]",
                "",
                None)

        self.actions = [a_explore, a_grasp_door, a_ungrasp, a_open_door, a_close_door, a_grasp_object, a_uncover, a_pour, a_transport, a_open_jar, a_transport2]

        s = Scenario(ws, self.goal, self.actions)
        s.process(pc)
        return

if __name__ == '__main__':

    rospy.init_node('symbolic_planner')

    task = SymbolicPlanner()

    task.spin()


