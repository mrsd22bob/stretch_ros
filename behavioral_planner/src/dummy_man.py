#!/usr/bin/env python

from __future__ import print_function

from behavioral_planner.srv import *
import rospy

man_no_op_key = 13
man_no_op_over_key = 14
man_op_key = 15
man_op_over_key = 16

process_over_key = 56

man_no_op_flag = 0
man_op_flag = 0
process_over = 0

def bp_to_man_handle(req):
    if(req.req==man_no_op_key):
        print("MAN: Manipulation without operation begins")
        global man_no_op_flag
        man_no_op_flag = 1
        return bpServiceResponse(req.req)
    if(req.req==man_op_key):
        print("MAN: Manipulation with operation begins")
        global man_op_flag
        man_op_flag = 1
        return bpServiceResponse(req.req)
    if(req.req==process_over_key):
        print("MAN: Process over, exiting")
        global process_over
        process_over = 1
        return bpServiceResponse(req.req)
    else:
        return bpServiceResponse(0)

def bp_to_man_server():
    rospy.init_node('bp_to_man_server')
    s = rospy.Service('bp_to_man', bpService, bp_to_man_handle)
    print("MAN: Ready")

def nodes_to_bp_client(key):
    rospy.wait_for_service('nodes_to_bp')
    try:
        req = rospy.ServiceProxy('nodes_to_bp', bpService)
        resp = req(key)
        return resp.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    bp_to_man_server()
    while(1):
        if(man_no_op_flag==1):
            #man_no_op

            #reset man_no_op
            man_no_op_flag=0
            #send man_no_op done to BP
            if(nodes_to_bp_client(man_no_op_over_key)==man_no_op_over_key):
                print("MAN: Manipulation without operation over")

        if(man_op_flag==1):
            #man_op

            #reset man_op
            man_op_flag=0
            #send done to BP
            if(nodes_to_bp_client(man_op_over_key)==man_op_over_key):
                print("MAN: Manipulation with operation over")

        if(process_over == 1):
            break


