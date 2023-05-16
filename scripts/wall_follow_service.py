#! /usr/bin/env python3

"""
.. module:: wall_follower_service
   :platform: Unix
   :synopsis: Python module for the robot bug0 control, wall following part

.. moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

This node implement the behavior of, one an obstacle is encountered along the path, following the wall perimeter, maintaining the wall on the robot right side at a given distance.

Subscriber:
   /scan

Publisher:
   /cmd_vel

Service:
   /wall_follower_switch (server)

"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import *

active_ = False

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


def wall_follower_switch(req):
    """
    Args: SetBool(req): service request message 

    This method will return the success responce to the service client.

    Return: SetBoolResponse(res): service responce message
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    """
    Args: LaserScan(msg): Lidar data

    This method will check for the closer wall in the 5 circular sector in front of the robot and call the method 'take_action()'
    
    Return: None
    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    """
    Args: int(state): desired state of the state machine
    
    this method change the state of the robot between the *go_to_point* behavior and the *wall_follower* one based on the state set by the rest of the algorithm
    
    Return: None
    """
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    """
    Args: None

    This method based on the position of the closest wall will change the state of the node.if there is no wall closer then the threshold in front of the robot, or there is one but on the left or on both left and right the node will move in the state 0 that, with the next iteration of the main will call the method *find_wall()*,
    if instead there is a wall infront of the robot, or in front and on the right, or in front and on the left or all around the robot, the node will switch to the state 1, that at the next iteration will call the method *turn_left()*, if the robot has a wall on it's right it will switch on the state 2 and on the next iteration will call the method *follow_the_wall()*. 
    
    Return: None
    """
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    """
    Args: None

    This method will set the message to publish on */cmd_vel* to move the robot along a circular trajectory till it find a wall.

    Return: Twist(msg): desired linear and angular speed
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left():
    """
    Args: None

    This method will set the message to publish on */cmd_vel* to turn the robot on the left.
    
    Return: Twist(msg): desired linear and angular speed
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall():
    """
    Args: None

    This method will set the message to publish on */cmd_vel* to move straight.

    Return: Twist(msg): desired linear and angular speed.
    """
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main():
    """
    Args: None

    This method after initialization will connect as a publisher to the topic */cmd_vel*, to controll the robot motion, subscribe to the topic */scan*, to get the information of the lidar and will connect to the service *wall_follower_switch* to activate or deactivate this node.
    
    Return: None
    """
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
