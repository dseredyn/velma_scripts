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

def free(ob):
    if ob.type == 'Manipulator':
        return ob.is_free
    raise TypeError("wrong type in predicate free: " + ob.type)

def grasped(ob1, ob2):
    if ob1.type == 'Manipulator' and ob2.type == 'Door':
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
def extractPredicatesInst(expr_str):
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
            args = []
            for idx in range(1, len(exp)):
                args.append( exp[idx] )
            result[expr_str[s+1:e-1]] = (pred_name, args)
        return result

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

class Manipulator:

#    def free(self):
#        return self.is_free

#    def grasped(self, ob):
#        return self.grasped_name == ob.name

#    def conf_feasible(self):
#        return self.is_conf_feasible

    def __init__(self, name):
        self.type = 'Manipulator'
        self.name = name
        self.is_free = True
        self.grasped_name = None
        self.is_conf_feasible = True
#        self.pred_map = {
#            "free": self.free,
#            "grasped": self.grasped,
#            "conf_feasible": self.conf_feasible,
#        }

#    def getPred(self, name, *arg):
#        if name in self.pred_map:
#            return self.pred_map[name](*arg)
#        raise NameError("name could not be found: predicate " + name + " in class " + self.type + " instance " + self.name)

    def grasp(self, ob):
        self.grasped_name = ob.name

    def ungrasp(self, ob):
        self.grasped_name = None

class Door(object):

    def opened(self):
        return self.is_opened

    def ajar(self):
        return self.is_ajar

    def closed(self):
        return self.is_closed

    def pose_certain(self):
        return self.is_pose_certain

    def __init__(self, name):
        self.type = 'Door'
        self.name = name
        self.is_opened = False
        self.is_ajar = False
        self.is_closed = True
        self.is_pose_certain = False
        self.pred_map = {
            "opened": self.opened,
            "is_ajar": self.ajar,
            "is_closed": self.closed,
            "pose_certain": self.pose_certain,
        }

    def getPred(self, name):
        if name in self.pred_map:
            return self.pred_map[name]()
        raise NameError("name could not be found: predicate " + name + " in class " + self.type + " instance " + self.name)

class Action:

    def __init__(self, name, parameters, precondition, effect, rt_failure):
        self.name = name

        par = parameters.split(",")
        self.parameters = {}
        for p in par:
            decl = p.split()
            self.parameters[decl[0]] = decl[1]

        self.precondition = precondition
        self.rt_failure = rt_failure

        self.effect_map = extractPredicatesAbst(effect, self.parameters)

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

class SymbolicPlanner:
    """
class for SymbolicPlanner
"""

    def __init__(self, pub_marker=None):
        predicates = [free, grasped, conf_feasible, opened, ajar, closed, pose_certain]
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

    def searchPossibilities(self, goal_str, param_str, obj_types_map, depth=0):
        indent_str = " " * (depth)

        posi = getAllPossibilities(goal_str, param_str, obj_types_map)

        found_possibility = False
        for p in posi:
            print indent_str + "possibility"
            pred_map = {}
            all_satisfied = True
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
                    objs.append(self.getObject(obj_name))
                curr_value = None
                assert all_inst
                curr_value = self.getPred(pred_name, *objs)
                print indent_str + " ", pred_value, "==", pred_name, pred_objs, pred_types, " (current value: ", curr_value, ")"

                # the predicate is not yet satisfied
                if curr_value != pred_value:
                    solution_found = False
                    for a in self.actions:
                        # get certain substitutions
                        substitutions = a.getEffect(pred_name, pred_objs, pred_types, pred_value)
                        if substitutions != None:
                            action_found = True
                            print indent_str + "  ", a.name, substitutions
                            precondition = a.precondition
                            for s in substitutions:
                                precondition = precondition.replace( s, substitutions[s] )
#                            print indent_str + "  ", precondition
                            # get other substitutions
                            subst2 = {}
                            subst2_inv = {}
                            type_pool = {}
                            for var in a.parameters:
                                if not var in substitutions:
                                    var_type = a.parameters[var]
                                    subst2[var] = var_type
                                    if not var_type in subst2_inv:
                                        subst2_inv[var_type] = [var]
                                    else:
                                        subst2_inv[var_type].append(var)
                                    if not var_type in type_pool:
                                        type_pool[var_type] = self.getObjectsOfType(var_type)
#                            print indent_str, "  ", subst2
#                            print indent_str, "  ", subst2_inv 
#                            print indent_str, "  ", type_pool

                            ll = []
                            ll_types = []
                            # generate cases for substitution
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
#                            print ll_types
                            for e in prod:
                                precondition2 = precondition
                                for ti in range(len(ll_types)):
                                    type_name = ll_types[ti]
                                    for vi in range(len(e[ti])):
                                        subst_var = subst2_inv[type_name][vi]
                                        subst_dest = e[ti][vi]
                                        precondition2 = precondition2.replace(subst_var, subst_dest)
#                                print indent_str + "   ", precondition2
                                if self.searchPossibilities(precondition2, "", obj_types_map, depth+3):
                                    solution_found = True
                    if not solution_found:
                        all_satisfied = False
            if not all_satisfied:
                print indent_str + "discard"
            else:
                found_possibility = True
        return found_possibility

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

        dl = Door("door_cab_l")
        dr = Door("door_cab_r")
        ml = Manipulator("man_l")
        mr = Manipulator("man_r")

        self.world_state = [dl, dr, ml, mr]
        obj_types_map = {}
        for obj in self.world_state:
            obj_types_map[obj.name] = obj.type

#        self.goal = "([opened ?door_cab_r] or (not [opened ?door_cab_r])) and [closed ?door_cab_l]"
        self.goal = "[opened door_cab_r]"

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
        a_explore = Action("explore",
                "?d Door,?m Manipulator",
                "[free ?m] and (not [pose_certain ?d])",
                "[pose_certain ?d]",
                "")

#(:action grasp_door_o
#  :parameters (?d - door ?m - _manipulator)
#  :precondition (and (free ?m) (pose_certain ?d))
#  :effect (grasped_o ?d ?m)
#)

        a_grasp_door = Action("grasp_door",
                "?d Door,?m Manipulator",
                "[free ?m] and [pose_certain ?d]",
                "[grasped ?m ?d]",
                "")

#(:action open_door
#  :parameters (?d - _door ?m - _manipulator)
#  :precondition (and(grasped_o ?d ?m) (feasible ?m) (or (closed ?d) (ajar ?d)))
#  :effect (and (grasped_o ?d ?m) (opened ?d))
#  :rt_failure (and (grasped_o ?d ?m) (not(feasible ?m)) (or (closed ?d) (ajar ?d)))
#)

        a_open_door = Action("open_door",
                "?d Door,?m Manipulator",
#                "[grasped ?m ?d] and [conf_feasible ?m] and (not [opened ?d])",#[closed ?d] or [ajar ?d])",
                "[grasped ?m ?d] and [conf_feasible ?m] and ([closed ?d] or [ajar ?d])",
                "[opened ?d]",
                "[grasped ?m ?d] and (not [conf_feasible ?m]) and ([closed ?d] or [ajar ?d])")

        a_close_door = Action("close_door",
                "?d Door,?m Manipulator",
                "[grasped ?m ?d] and [conf_feasible ?m] and ([opened ?d] or [ajar ?d])",
                "[closed ?d]",
                "")

        self.actions = [a_explore, a_grasp_door, a_open_door, a_close_door]

#        print self.extractPredicatesInst(self.goal)
#        print extractPredicatesAbst(a_open_door.precondition, a_open_door.parameters)
        #print extractPredicates(a_open_door.precondition, a_open_door.parameters)
        #print extractPredicates(self.goal, "")

        self.searchPossibilities(self.goal, "", obj_types_map)

        return

        posi = getAllPossibilities(self.goal, "", obj_types_map)
#        posi = getAllPossibilities(a_open_door.precondition, a_open_door.parameters)
        print posi

        for p in posi:
            print "possibility"
            for pred in p:
                pred_name = p[pred][0][0]
                pred_objs = p[pred][0][1]
                pred_types = p[pred][0][3]
                pred_value = p[pred][1]
                print "   ", pred_value, " == ", pred_name, " ", pred_objs, " ", pred_types
                objs = []
                all_inst = True
                for obj_name in pred_objs:
                    if obj_name == None:
                        all_inst = False
                        break
                    objs.append(self.getObject(obj_name))
                if all_inst:
                    curr_value = self.getPred(pred_name, *objs)
                    print "      current value: ", curr_value
                    if curr_value == False:
                        pass
                else:
                    print "      current value: unknown"

                for a in self.actions:
                    var_names = a.getEffect(pred_name, pred_types, pred_value)
                    if var_names != None:#a.hasEffect(pred_name, pred_types, pred_value):
                        print "      action ", a.name
        return


        # get all actions that lead to the goal
        for case in goal_cases:
            print "case"
            for pred in case:
                pred_value = case[pred][1]
                pred_name = case[pred][0][0]
                pred_args = case[pred][0][1]
                pred_types = getObjTypes(pred_args)
                print "  ", pred_value, " == ", pred_name
                # get actions
                for a in self.actions:
                    var_names = a.getEffect(pred_name, pred_types, pred_value)
                    if var_names != None:#a.hasEffect(pred_name, pred_types, pred_value):
                        print "    action ", a.name

        pred_map = {}
#        for pred in goal_pred:
#            pred_map[pred[0]] = 
#            print self.getPred(pred[1], *pred[2])

        # find action with result that satisfies the goal

        return
        print self.goal
        e = 0
        new_goal = ""
        while True:
            s = self.goal.find("[", e)
            if s < 0:
                new_goal = new_goal + self.goal[e:]
                break
            new_goal = new_goal + self.goal[e:s]
            e = self.goal.find("]", s)+1
            exp = self.goal[s+1:e-1].split()
            pred_name = exp[0]
            args = []
            for idx in range(1, len(exp)):
                args.append( self.getObject(exp[idx][1:]) )
            print exp
            result = self.getPred(pred_name, *args)
            new_goal = new_goal + str(result)
            print result

        print new_goal
        print eval(new_goal)

        # find action with result that satisfies the goal
#        for a in actions:
#            a.effect

#        print eval("True and True")
#        print eval("(True and True) and False")

        print "done."
        

if __name__ == '__main__':

    rospy.init_node('symbolic_planner')

    task = SymbolicPlanner()

    task.spin()


