#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import assignment_2_2022.msg
import math


goal = Point()
d = 0 # init the distance
v = 0 # init the actual velocity
old_v = 0 # init the previous actual velocity
av = 0 # init the average velocity
n = 0 # init the counter

def confy_cb(msg):
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
    rospy.init_node('node_c', anonymous=True) # init the node
    sub_confy = rospy.Subscriber('/configuration', assignment_2_2022.msg.Conf, confy_cb) # sub to odom to obtain position and velocity

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()
