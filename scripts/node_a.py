#! /usr/bin/env python3

"""
.. module:: node_a
   :platform: Unix
   :synopsis: Python module for the translation and publication of the robot position and velocity

.. moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

This node works as an adapter converting the odometry information received in the custom message Conf.msg type.

Subscriber:
   /odom

Publisher:
   /configuration

"""

import rospy
from nav_msgs.msg import Odometry
import assignment_2_2022.msg

confy = assignment_2_2022.msg.Conf()
pub_conf = rospy.Publisher('/configuration', assignment_2_2022.msg.Conf, queue_size=1) # publish to configuration to pass the actual one

def odom_cb(msg):
    """
    Args: Odometry(msg): position and orientation of the robot

    This method aquire the odometry message and fulfill the custom message confy and pusblish it under the topic */configuration*.
    
    Return: None
    """
    # received the message retrieve the position and velocity and fulfill the custom message
    confy.x = msg.pose.pose.position.x
    confy.y = msg.pose.pose.position.y
    confy.Vx = msg.twist.twist.linear.x
    confy.Vy = msg.twist.twist.linear.x

    # publish the message on the given topic
    pub_conf.publish(confy) 

def main():
    """
    Args: None

    This method initialize the node, create a publisher to the topic */configuration* (outside of the main in such a way it can be used by the callback), subscribe to the topic */odom* and wait for the master to be launched.
    
    Return: None
    """
    rospy.init_node('node_a', anonymous=True) # init the node
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_cb) # sub to odom to obtain position and velocity

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()
