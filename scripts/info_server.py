#! /usr/bin/env python3

import rospy
from assignment_2_2022.srv import Ngoal

info = Ngoal()

def ret_info(req):
    return [rospy.get_param('n_goal'), rospy.get_param('n_deleted')]
    
def info_server():
    rospy.init_node('info_server')
    rospy.Service('/info', Ngoal, ret_info)
    rospy.spin()

if __name__ == "__main__":
    info_server()