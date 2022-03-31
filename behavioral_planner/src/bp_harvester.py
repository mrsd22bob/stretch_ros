#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from behavioral_planner.srv import *
import time

target_count = 3

navigate_key = 9
navigate_over_key = 10
explore_key = 11
explore_over_key = 12
man_no_op_key = 13
man_no_op_over_key = 14
man_op_key = 15
man_op_over_key = 16

process_over_key = 56

navigate_flag = 0
explore_flag = 0
man_no_op_flag = 0
man_op_flag = 0

def bp_to_nav_client(key):
    rospy.wait_for_service('bp_to_nav')
    try:
        req = rospy.ServiceProxy('bp_to_nav', bpService)
        resp = req(key)
        return resp.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def bp_to_man_client(key):
    rospy.wait_for_service('bp_to_man')
    try:
        req = rospy.ServiceProxy('bp_to_man', bpService)
        resp = req(key)
        return resp.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def nodes_to_bp_handle(req):
    if(req.req==navigate_over_key):
        print("BP: Navigation over")
        global navigate_flag
        navigate_flag =1
        return bpServiceResponse(req.req)
    if(req.req==explore_over_key):
        print("BP: Exploration over")
        global explore_flag
        explore_flag =1
        return bpServiceResponse(req.req)
    if(req.req==man_no_op_over_key):
        print("BP: Manipulation without operation over")
        global man_no_op_flag
        man_no_op_flag =1
        return bpServiceResponse(req.req)
    if(req.req==man_op_over_key):
        print("BP: Manipulation with operation over")
        global man_op_flag
        man_op_flag =1
        return bpServiceResponse(req.req)
    else:
        return bpServiceResponse(0)

def nodes_to_bp_server():
    rospy.init_node('nodes_to_bp_server')
    s = rospy.Service('nodes_to_bp', bpService, nodes_to_bp_handle)
    print("BP: Ready")

def navigate():
    global navigate_flag
    while(int(bp_to_nav_client(navigate_key))!=navigate_key):
        continue
    print("BP: Navigation begins")
    while(navigate_flag==0):
        continue
    navigate_flag = 0

def man_no_op():
    global man_no_op_flag
    while(int(bp_to_man_client(man_no_op_key))!=man_no_op_key):
        continue
    print("BP: Manipulation without operation begins")
    while(man_no_op_flag==0):
        continue
    man_no_op_flag = 0

def man_op():
    global man_op_flag
    while(int(bp_to_man_client(man_op_key))!=man_op_key):
        continue
    print("BP: Manipulation with operation begins")
    while(man_op_flag==0):
        continue
    man_op_flag = 0

def explore():
    global explore_flag
    while(int(bp_to_nav_client(explore_key))!=explore_key):
        continue
    print("BP: Exploration begins")
    while(explore_flag==0):
        continue
    explore_flag =0

def exit():
    bp_to_nav_client(process_over_key)
    bp_to_man_client(process_over_key)
    print("BP: Process over, exiting")

def auto():
    count = 0
    while(count<target_count):
        navigate()
        man_no_op()
        man_op()
        count+=1
        print("BP: Process done %d times" %count)
    explore()

if __name__ == "__main__":
    nodes_to_bp_server()

    while(1):
        time.sleep(1)
        print("BP:")
        print("1. Exploration")
        print("2. Navigation")
        print("3. Manipulation without operation")
        print("4. Manipulation with operation")
        print("5. Auto")
        print("6. Exit")
        x = int(input("Enter value: "))

        if(x==1):
            explore()
        if(x==2):
            navigate()
        if(x==3):
            man_no_op()
        if(x==4):
            man_op()
        if(x==5):
            auto()
        if(x==6):
            exit()
            break
