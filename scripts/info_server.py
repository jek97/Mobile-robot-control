#! /usr/bin/env python3

import rospy
from assignment_2_2022.srv import Ngoal

"""
..module:: info_server
  :platform: Unix
  :synopsis: Python module for return upon request the number of goal achieìved and deleted
  :version: 0.1

..moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

This node will provide the system with a service, used by the user_interface for counting the number of goal reached and the number of goal deleted upon request.

Parameter:
   n_goal (input)
   n_deleted (input)

Service:
   /info (server)

"""

info = Ngoal()

def ret_info():
    """
    Args: None

    In this method the message is fulfilled with the number of goal achived and deleted, obtained from the ros parameter server, and sent back to the client.

    Return: [int(n_goal), int(n_deleted)]: number of goal achived and number of goal deleted
    """
    return [rospy.get_param('n_goal'), rospy.get_param('n_deleted')]
    
def info_server():
    """
    Args: None

    This method inialize the node, create the srvice */info* and then spin.
    
    Return: None
    """
    rospy.init_node('info_server')
    rospy.Service('/info', Ngoal, ret_info)
    rospy.spin()

if __name__ == "__main__":
    info_server()