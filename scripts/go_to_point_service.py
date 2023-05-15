#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

"""
..module:: go_to_point_service
  :platform: Unix
  :synopsis: Python module for the robot bug0 control, straight motion part
  :version: 0.1

..moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

This node implement the simple behavior of the robot of, given a goal position, reach it moving in straight line, it will be integrated with the other nodes to perform the bug0 algorithm.

Parameters:
   des_pos_x (input)
   des_pos_y (input)

Subscriber:
   /odom 

Publisher:
   /cmd_vel

Service:
   /go_to_point_switch (server)

"""

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.3

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# service callbacks


def go_to_point_switch(req):
    """
    Args: SetBool req: service request message 

    This method will return the succes responce to the service client.

    Return: SetBoolResponse res: service responce message
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks


def clbk_odom(msg):
    """
    Args: Odometry msg: odometry message

    This method will fulfill the variables 'position_', 'yaw_' with the received message informations
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    This method will notify the change of state
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    This method will normalize the angle argument obtaining an angle always between -180° and +180°
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """ 
    This method will evaluate the heading error between the robot and the goal and if over a threshold will pulish a rotational speed to allign with it; otherwise it will change the current state of the node by the function 'change_state' that will notify the changes.
    """
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    This method will use a PID controller to move the robot in a straight line to the goal, passed as argument, then if this motion increase the misalignment of the robot over a threshold the state will be changed back to 0 to correct this fact; otherwise if the distance between the robot and the goal fall under a certain threshold the node will switch to the new state 2.
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d*(err_pos)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    """
    This method that will simply set the velocity to zero and publish it.
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
                

def main():
    """
    the node will, after initialization, create a publisher to the topic '/cmd_vel' of type Twist, it's the topic used to communicate with the robot giving it the velocity, both linear and angular at each step; after that the node will subscribe to the topic '/odom' of type Odomentry, that will give a feedback to the algorithm giving the actual position and velocity of the robot;
    moreover the node will check the service *go_to_point* switch that will be used by the 'bug_action_server' node to switch between the behavior of this node and the one of 'wall_follower'.
    Thanks to the above mentioned subscription to the topic '/odom' all the times the robot receive a new message it will update its position and orientation by the callback 'clbk_odom()'.
    finished this initial phase the node will check the actual goal position in the ros parameter server under the names 'des_pos_x' and 'des_pos_y'.
    """
    global pub, active_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            desired_position_.x = rospy.get_param('des_pos_x')
            desired_position_.y = rospy.get_param('des_pos_y')
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
