#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import assignment_2_2022.msg
import math

"""
..module:: node_c
  :platform: Unix
  :synopsis: Python module for printing on the terminal the robot informations
  :version: 0.1

..moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

This node will use the information received to print on the terminal the actual distance to the goal and the average speed of tyhe robot.

Parameter:
   des_pos_x (input)
   des_pos_y (input)
   
Subscriber:
   /configuration

"""
goal = Point()
d = 0 # init the distance
v = 0 # init the actual velocity
old_v = 0 # init the previous actual velocity
av = 0 # init the average velocity
n = 0 # init the counter

def confy_cb(msg):
    """
    Args: assignment_2_2022.msg.Conf(msg): actual position and velocity of the robot

    In this method first the goal is retrieved form the ros parameter server, the distance between the robot position and the goal is evaluated together with the average speed, all controlled by the counter 'n' used for the average operation, if it's a multiple of 40 then the two information are displaied.
    
    Return: None
    """
    global n, d, v, av
    # increase the counter:
    n += 1
    # obtain the goal position:
    goal.x = rospy.get_param('des_pos_x') # get the actual x goal
    goal.y = rospy.get_param('des_pos_y') # get the actual x goal
    # compute the distance:
    d = math.sqrt(pow(goal.y - msg.y, 2) + pow(goal.x - msg.x, 2))
    # compute the average speed:
    v = math.sqrt(pow(msg.Vx, 2) + pow(msg.Vy, 2)) # compute the actual speed module
    av = (old_v + v) / n # compute the average speed in time
    if ((n % 40) == 0):
        # print on the terminal:
        log = "distance from the goal: %f; average speed: %f" % (d, av)
        rospy.loginfo(log)

def main():
    """
    Args: None

    This method initialize the node, subscribe to the topic */configuration* to aquire the information about the robot position and velocity and wait for the master to be launched, after that the node will spin.
    
    Return: None
    """
    rospy.init_node('node_c', anonymous=True) # init the node
    sub_confy = rospy.Subscriber('/configuration', assignment_2_2022.msg.Conf, confy_cb) # sub to odom to obtain position and velocity

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()
