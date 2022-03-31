#!/usr/bin/env python

from __future__ import print_function

from behavioral_planner.srv import *
import rospy

navigate_key = 9
navigate_over_key = 10
explore_key = 11
explore_over_key = 12
process_over_key = 56

navigate_flag = 0
explore_flag = 0
process_over = 0

def bp_to_nav_handle(req):
    if(req.req==explore_key):
        print("NAV: Exploration begins")
        global explore_flag
        explore_flag = 1
        return bpServiceResponse(req.req)
    if(req.req==navigate_key):
        print("NAV: Navigation begins")
        global navigate_flag
        navigate_flag = 1
        return bpServiceResponse(req.req)
    if(req.req==process_over_key):
        print("NAV: Process over, exiting")
        global process_over
        process_over = 1
        return bpServiceResponse(req.req)
    else:
        return bpServiceResponse(0)

def bp_to_nav_server():
    rospy.init_node('bp_to_nav_server')
    s = rospy.Service('bp_to_nav', bpService, bp_to_nav_handle)
    print("NAV: Ready")

def nodes_to_bp_client(key):
    rospy.wait_for_service('nodes_to_bp')
    try:
        req = rospy.ServiceProxy('nodes_to_bp', bpService)
        resp = req(key)
        return resp.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    bp_to_nav_server()
    while(1):
        if(explore_flag==1):
            #explore

            #reset explore
            explore_flag=0
            #send explore done to BP
            if(nodes_to_bp_client(explore_over_key)==explore_over_key):
                print("NAV: Exploration over")

        if(navigate_flag==1):
            #navigate
            

            #reset navigate
            navigate_flag=0
            #send done to BP
            if(nodes_to_bp_client(navigate_over_key)==navigate_over_key):
                print("NAV: Navigation over")

        if(process_over == 1):
            break


