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

#def reachable(o):
#    if o.type == 'Object':
#        return o.is_reachable

def reachable(o, c):
    if o.type == 'Object' and c.type == 'Container':
        if o.container_in == c.name:
            return True
        else:
            return False
    raise TypeError("wrong types in predicate reachable: " + o.type + " " + c.type)

def inside(o1, o2):
    if o1.type == 'Object' and o2.type == 'Container':
        return True
    raise TypeError("wrong types in predicate inside: " + o1.type + " " + o2.type)

def part_of(d, o):
    if d.type == 'Door' and o.type == 'Container':
        if d.container == o.name:
            return True
        else:
            return False
    raise TypeError("wrong types in predicate part_of: " + d.type + " " + o.type)

def free(ob):
    if ob.type == 'Manipulator':
        return ob.is_free
    raise TypeError("wrong type in predicate free: " + ob.type)

def grasped(ob1, ob2):
    if ob1.type == 'Manipulator' and ob2.type == 'Door':
        return ob1.grasped_name == ob2.name
    if ob1.type == 'Manipulator' and ob2.type == 'Object':
        return ob1.grasped_name == ob2.name
    raise TypeError("wrong types in predicate grasped: " + ob1.type + " " + ob2.type)

def conf_feasible(ob):
    if ob.type == 'Manipulator':
        return ob.is_conf_feasible
    raise TypeError("wrong type in predicate conf_feasible: " + ob.type)

def opened(ob):
    if ob.type == 'Door':
        return ob.is_opened
    raise TypeError("wrong type in predicate opened: " + ob.type)

def ajar(ob):
    if ob.type == 'Door':
        return ob.is_ajar
    raise TypeError("wrong type in predicate ajar: " + ob.type)

def closed(ob):
    if ob.type == 'Door':
        return ob.is_closed
    raise TypeError("wrong type in predicate closed: " + ob.type)

def pose_certain(ob):
    if ob.type == 'Door':
        return ob.is_pose_certain
    raise TypeError("wrong type in predicate pose_certain: " + ob.type)

# arguments: expr_str
# returned value:
# dictionary: pred_str:(pred_name, [obj1_name, obj2_name,...]):
#def extractPredicatesInst(expr_str):
#        result = {}
#        e = 0
#        while True:
#            s = expr_str.find("[", e)
#            if s < 0:
#                break
#            e = expr_str.find("]", s)+1
#            if expr_str[s+1:e-1] in result:
#                continue
#            exp = expr_str[s+1:e-1].split()
#            pred_name = exp[0]
#            args = []
#            for idx in range(1, len(exp)):
#                args.append( exp[idx] )
#            result[expr_str[s+1:e-1]] = (pred_name, args)
#        return result

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

# arguments: expr_str, parameters
# returned value:
# dictionary: pred_str:(pred_name, [obj1_name, obj2_name,...], [arg1_name, arg2_name,...], [arg1_type, arg2_type,...]):
def extractPredicates(expr_str, parameters, obj_types_map):
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
            obj_names = []
            arg_names = []
            arg_types = []
            for idx in range(1, len(exp)):
                if parameters != None and exp[idx] in parameters:
                    assert exp[idx][0] == "?"
                    obj_names.append( None )
                    arg_names.append( exp[idx] )
                    arg_types.append( parameters[exp[idx]] )
                else:
                    assert exp[idx][0] != "?"
                    assert exp[idx] in obj_types_map
                    obj_names.append( exp[idx] )
                    arg_names.append( None )
                    arg_types.append( obj_types_map[exp[idx]] )
            result[expr_str[s+1:e-1]] = (pred_name, obj_names, arg_names, arg_types)
        return result

def getAllPossibilities(goal_exp, parameters, obj_types_map):
        # get all required conditions that archieve the goal expression
        goal_pred = extractPredicates(goal_exp, parameters, obj_types_map)
        goal_product = itertools.product([True, False], repeat = len(goal_pred))
        goal_cases = []
        for subset in goal_product:
            goal_str = goal_exp
            i = 0
            case = {}
            for pred in goal_pred:
                goal_str = goal_str.replace("["+pred+"]", str(subset[i]))
                case[pred] = (goal_pred[pred], subset[i])
                i += 1
            if eval(goal_str):
                goal_cases.append(case)
                #print goal_str
        return goal_cases

def getObjTypes(obj_list):
    result = []
    for obj in obj_list:
        result.append( obj.type )
    return result

def substitute(exp_str, subst_map):
    result = exp_str
    for s in subst_map:
        result = result.replace(s, subst_map[s])
    return result

def getObject(world_state, name):
    for ob in world_state:
        if ob.name == name:
            return ob
    return None

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
                                objs = obj_type_map[var_type] #self.getObjectsOfType(var_type)
                                type_pool[var_type] = []
                                for obj in objs:
                                    obj_used = False
                                    for vv in substitutions:
                                        if obj == substitutions[vv]:
                                            obj_used = True
                                    if not obj_used:
                                        type_pool[var_type].append(obj)
                                    else:
                                        assert False    # ok

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

class Object:
    def __init__(self, name):
        self.type = 'Object'
        self.name = name
        self.is_reachable = True
        self.container_in = None

    def printObj(self):
        print self.type, self.name

class Manipulator:

    def __init__(self, name):
        self.type = 'Manipulator'
        self.name = name
        self.is_free = True
        self.grasped_name = None
        self.is_conf_feasible = True

    def printObj(self):
        print self.type, self.name, "is_free:", self.is_free, "grasped_name:", self.grasped_name, "is_conf_feasible:", self.is_conf_feasible

    def grasp(self, ob):
        self.grasped_name = ob.name

    def ungrasp(self, ob):
        self.grasped_name = None

class Door:

    def __init__(self, name, container):
        self.type = 'Door'
        self.name = name
        self.is_opened = False
        self.is_ajar = False
        self.is_closed = True
        self.is_pose_certain = False
        self.container = container

    def printObj(self):
        print self.type, self.name, "is_opened:", self.is_opened, "is_ajar:", self.is_ajar, "is_closed:", self.is_closed, "is_pose_certain:", self.is_pose_certain, "container:", self.container

class Container:

    def __init__(self, name):
        self.type = 'Container'
        self.name = name

    def printObj(self):
        print self.type, self.name

class Action:

    def __init__(self, name, parameters, precondition, effect, rt_failure, action_sim):
        self.name = name

        par = parameters.split(",")
        self.parameters = {}
        self.param_list = []
        for p in par:
            decl = p.split()
            self.parameters[decl[0]] = decl[1]
            self.param_list.append( decl[0] )

        self.precondition = precondition
        self.rt_failure = rt_failure

        self.effect_map = extractPredicatesAbst(effect, self.parameters)

        self.action_sim = action_sim

    def hasEffect(self, pred_name, pred_types, pred_value):
        for pred_str in self.effect_map:
            if self.effect_map[pred_str][0] == pred_name:
                match = True
                for arg_i in range(len(pred_types)):
                    if self.effect_map[pred_str][2][arg_i] != pred_types[arg_i]:
                        match = False
                        break
                if match and pred_value:
                    return True

    def getEffect(self, pred_name, pred_objs, pred_types, pred_value):
        for pred_str in self.effect_map:
            substitutions = {}
            if self.effect_map[pred_str][0] == pred_name:
                match = True
                for arg_i in range(len(pred_types)):
                    if self.effect_map[pred_str][2][arg_i] != pred_types[arg_i]:
                        match = False
                        break
                    substitutions[ self.effect_map[pred_str][1][arg_i] ] = pred_objs[arg_i]
                if match and pred_value:
                    return substitutions
        return None

    def actionSim(self, *args):
        if self.action_sim != None:
            return self.action_sim(*args)
        return None

class Scenario:

    def __init__(self, init_state, goal, predicates_map, actions):
        self.init_state = copy.deepcopy(init_state)
        self.goal = copy.copy(goal)
        self.predicates_map = predicates_map
        self.obj_types_map = {}
        self.obj_types_map_inv = {}
        self.actions = actions
        self.action_map = {}
        for a in self.actions:
            self.action_map[a.name] = a

        for obj in self.init_state:
            self.obj_types_map[obj.name] = obj.type
            if not obj.type in self.obj_types_map_inv:
                self.obj_types_map_inv[obj.type] = []
            self.obj_types_map_inv[obj.type].append( obj.name )

    def getPred(self, name, *arg):
        return self.predicates_map[name](*arg)

    def process(self):
        indent_str = " "

        steps = [
            (None, self.init_state, None, None),
            (self.goal, None, None, None) ]

        while True:
            print "*"
            print "*"
            print "*"
            for s_idx in range(0, len(steps)-1):
                prev_step = steps[s_idx]
                next_step = steps[s_idx+1]
                goal = next_step[0]
                world_state = prev_step[1]
                print "iterating steps:"
                print "state:"
                for obj in world_state:
                    obj.printObj()
                print "goal:", goal

                print "getAllPossibilities", goal
                posi = getAllPossibilities(goal, None, self.obj_types_map)

                # fork here
                p = posi[0]
                step_added = False
                for pred in p:
                    pred_name = p[pred][0][0]
                    pred_objs = p[pred][0][1]
                    pred_types = p[pred][0][3]
                    pred_value = p[pred][1]
                    objs = []
                    all_inst = True
                    for obj_name in pred_objs:
                        if obj_name == None:
                            all_inst = False
                            break
                        objs.append(getObject(world_state, obj_name))

                    assert all_inst
                    curr_value = self.getPred(pred_name, *objs)
                    print indent_str + " ", pred_value, "==", pred_name, pred_objs, pred_types, " (current value: ", curr_value, ")"

                    # the predicate is not yet satisfied
                    if curr_value != pred_value:
                        print indent_str + "  ", "not satisfied"
                        solution_found = False
                        for a in self.actions:
                            # get certain substitutions for a given action
                            substitutions = a.getEffect(pred_name, pred_objs, pred_types, pred_value)
                            if substitutions != None:

                                action_found = True
                                print indent_str + "  ", a.name, substitutions

                                subst_cases = generateSubstitutionCases(self.obj_types_map_inv, a.parameters, substitutions)

                                # fork here
                                sc = subst_cases[0]

                                sc_all = sc.copy()
                                sc_all.update(substitutions)
                                #precond = substitute(a.precondition, sc_all)

                                new_state = copy.deepcopy(world_state)

                                action_args = []
                                for par_name in a.param_list:
                                    obj_name = sc_all[par_name]
                                    action_args.append( getObject(new_state, obj_name) )

                                a.actionSim( *action_args )
                                precond = substitute(a.precondition, sc_all)
                                print "step added", a.name, precond
                                steps.insert( s_idx+1, (precond, new_state, a.name, sc_all) )
                                step_added = True
                                break
                    if step_added:
                        break
                if step_added:
                    break
            if not step_added:
                break
            # post process all steps - propagate world changes
            for s_idx in range(0, len(steps)-1):
                prev_step = steps[s_idx]
                next_step = steps[s_idx+1]
                world_state = prev_step[1]
                action_name = next_step[2]
                sc_all = next_step[3]

                if action_name != None:
                    a = self.action_map[action_name]
                    new_state = copy.deepcopy(world_state)
                    action_args = []
                    for par_name in a.param_list:
                        obj_name = sc_all[par_name]
                        action_args.append( getObject(new_state, obj_name) )
                    a.actionSim( *action_args )
                    print "world state: " + a.name, sc_all
                    for obj in new_state:
                        obj.printObj()
                    steps[s_idx+1] = (steps[s_idx+1][0], new_state, steps[s_idx+1][2], steps[s_idx+1][3])

#            raw_input(".")

            if rospy.is_shutdown():
                break


        print "steps:"
        for s in steps:
            print s[2], s[3]
        return

    def extractGoalPossibilities(self):
        pass        

class SymbolicPlanner:
    """
class for SymbolicPlanner
"""

    def __init__(self, pub_marker=None):
        predicates = [free, grasped, conf_feasible, opened, ajar, closed, pose_certain, inside, part_of, reachable]
        self.predicates_map = {}
        for pred in predicates:
            self.predicates_map[pred.__name__] = pred

    def getPred(self, name, *arg):
        return self.predicates_map[name](*arg)

    def getObject(self, name):
        for ob in self.world_state:
            if ob.name == name:
                return ob
        return None

    def getObjectsOfType(self, type_name):
        result = []
        for ob in self.world_state:
            if ob.type == type_name:
                result.append( ob.name )
        return result

    def spin(self):

        if False:
            list_a = []
            x = itertools.combinations( (1,2), 2 )
            for elem in x:
                p = itertools.permutations(elem)
                for e in p:
                    list_a.append(e)

            list_b = []
            x = itertools.combinations( ('a','b','c'), 2 )
            for elem in x:
                p = itertools.permutations(elem)
                for e in p:
                    list_b.append(e)

            print "list_a"
            print list_a
            print "list_b"
            print list_b

            list_c = ['x','y']
            ll = [list_a, list_b, list_c]
            prod = itertools.product(*ll)
            print "prod"
            for e in prod:
                print e
            return

        cab = Container("cab")
        dl = Door("door_cab_l", "cab")
        dr = Door("door_cab_r", "cab")
        ml = Manipulator("man_l")
        mr = Manipulator("man_r")
        jar = Object("jar")
        jar.is_reachable = False
        jar.container_in = "cab"

        self.world_state = [dl, dr, ml, mr, jar, cab]
        obj_types_map = {}
        for obj in self.world_state:
            obj_types_map[obj.name] = obj.type

#        self.goal = "([opened ?door_cab_r] or (not [opened ?door_cab_r])) and [closed ?door_cab_l]"
        self.goal = "[opened door_cab_r] and [free man_l] and [grasped man_r jar]"

        # unit tests
        assert self.getPred("free", ml) == True
        assert self.getPred("grasped", ml, dl) == False
        assert self.getPred("conf_feasible", ml) == True
        ml.grasp(dl)
        assert self.getPred("grasped", ml, dl) == True
        ml.ungrasp(dl)
        assert self.getPred("grasped", ml, dl) == False

#(:action explore
#  :parameters (?s - vertical_surface ?m - _manipulator)
#  :precondition (and (free ?m) (not (pose_certain ?s)))
#  :effect (pose_certain ?s)
#)

        def a_explore_sim(d, m):
            assert d != None
            d.is_pose_certain = True

        a_explore = Action("explore",
                "?d Door,?m Manipulator",
                "[free ?m] and (not [pose_certain ?d])",
                "[pose_certain ?d]",
                "",
                a_explore_sim)

#(:action grasp_door_o
#  :parameters (?d - door ?m - _manipulator)
#  :precondition (and (free ?m) (pose_certain ?d))
#  :effect (grasped_o ?d ?m)
#)

        def a_grasp_door_sim(d, m):
            assert d != None
            assert m != None
            m.is_free = False
            m.grasped_name = d.name

        a_grasp_door = Action("grasp_door",
                "?d Door,?m Manipulator",
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
                "[grasped ?m ?d]",
                "[free ?m]",
                "",
                a_ungrasp_door_sim)

        def a_ungrasp_sim(m):
            m.is_free = True
            m.grasped_name = None

        a_ungrasp = Action("ungrasp",
                "?m Manipulator",
                "not [free ?m]",
                "[free ?m]",
                "",
                a_ungrasp_sim)

#(:action open_door
#  :parameters (?d - _door ?m - _manipulator)
#  :precondition (and(grasped_o ?d ?m) (feasible ?m) (or (closed ?d) (ajar ?d)))
#  :effect (and (grasped_o ?d ?m) (opened ?d))
#  :rt_failure (and (grasped_o ?d ?m) (not(feasible ?m)) (or (closed ?d) (ajar ?d)))
#)

        def a_open_door_sim(d, m):
            assert d != None
            d.is_opened = True
            d.is_ajar = False
            d.is_closed = False
            d.container
            # TODO

        a_open_door = Action("open_door",
                "?d Door,?m Manipulator",
                "[grasped ?m ?d] and [conf_feasible ?m] and ([closed ?d] or [ajar ?d])",
                "[opened ?d]",
                "[grasped ?m ?d] and (not [conf_feasible ?m]) and ([closed ?d] or [ajar ?d])",
                a_open_door_sim)

        def a_close_door_sim(d, m):
            assert d != None
            d.is_opened = False
            d.is_ajar = False
            d.is_closed = True

        a_close_door = Action("close_door",
                "?d Door,?m Manipulator",
                "[grasped ?m ?d] and [conf_feasible ?m] and ([opened ?d] or [ajar ?d])",
                "[closed ?d]",
                "",
                a_close_door_sim)

        def a_grasp_object_sim(o, m):
            assert o != None
            assert m != None
            m.is_free = False
            m.grasped_name = o.name

        a_grasp_object = Action("grasp_object",
                "?o Object,?m Manipulator",
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
                "?o1 Object,?o2 Container,?d Door",
                "[inside ?o1 ?o2] and [part_of ?d ?o2] and [opened ?d]",
                "[reachable ?o1]",
                "",
                None)

#inside jar cabinet
#closed door_cab_l
#closed door_cab_r


#        a_take_out = Action("take_out",
#                "?o1 Object,?o2 Object,?m Manipulator",
#                "[inside ?o1 ?o2] and [opened ?o2] and [grasped ?o1 ?m]",
#                "not [inside ?o1 ?o2]",
#                "",
#                a_grasp_object_sim)

        self.actions = [a_explore, a_grasp_door, a_ungrasp, a_open_door, a_close_door, a_grasp_object, a_uncover]

        s = Scenario(self.world_state, self.goal, self.predicates_map, self.actions)
        s.process()
        return

if __name__ == '__main__':

    rospy.init_node('symbolic_planner')

    task = SymbolicPlanner()

    task.spin()


