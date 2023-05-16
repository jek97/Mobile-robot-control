#! /usr/bin/env python3

"""
.. module:: bug_as
   :platform: Unix
   :synopsis: Python module for the robot bug0 control

.. moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

this node , based on a state machine architecture, together with the two node go_to_point and wall_follower will menage the implementation of the bug0 algorithm, indeed based on the lidar information will decide when to switch between the go_to_point behavior and the wall_follower one.

Parameter:
   des_pos_x (output)
   des_pos_y (output)

Subscriber:
   /scan
   /odom 

Publisher:
   /cmd_vel

Service:
   /go_to_point_switch (client)
   /wall_follower_switch (client)

Action:
   /reaching_goal (Server)

"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from tf import transformations
from std_srvs.srv import *
import time


srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
pose_ = Pose()
desired_position_ = Point()
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'done']
state_ = 0
# 0 - go to point
# 1 - wall following
# 2 - done
# 3 - canceled
# callbacks


def clbk_odom(msg):
    """
    Args: Odometry(msg): position and orientation of the robot

    This method aquire the position information and put them in the variables *position*, *pose_* and *yaw_* in this way they are ready to use for the rest of the algorithm.
    
    Return: None
    """
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    """
    Args: LaserScan(msg): Lidar data

    This method checks which is the minimum distance between the robot and the environment objects in the five circular sectors in front of the robot and set that distance value under one name for each sector.
    
    Return: None
    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    """
    Args: int(state): desired state of the state machine

    this method change the state of the robot between the *go_to_point* behavior and the *wall_follower* one based on the state set by the rest of the algorithm
    
    Return: None
    """
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)


def normalize_angle(angle):
    """
    Args: float(angle): angle in radiants not normalized

    This method will normalize the angle argument obtaining an angle always between -pi and +pi°

    Return: float(angle): angle in radiants normalized
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def done():
    """
    Args: None

    This method that will simply set the velocity to zero and publish it.
    
    Return: None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    
    
def planning(goal):
    """
    Args: assignment_2_2022.msg.PlanningAction(goal): desired goal

    This method initially set the goal received from the action client in the ros parameter server, then getting inside the while loop it compute the distance respect the goal.
    after that initially the algorithm check if the action client required to delete the goal, and in this case the right feedback message is set and pusblished, the goal is preempted, the state is changed to 2 (the change state method will be analized right after), and the robot speed is set to zero.
    otherwise if the distance to the goal is less then 0.5 we assume it to be reached, the state is chaged to 2, as before the right feedback message is set and published and tho robot is blocked in its position.
    in case no wall is detected (so the array *regions_* is void) the algorithm just contineu.
    then if the state is set to 0, so the motion is controlled by the *go_to_point* behavior, the node send a feedback and if there is an obstacle nearer then 0.2 in front of the robot it changes teh state to 1.
    instead if the state in which we are is the 1, so the motion is controlled by the *wall_follower* behavior, the node send a feedback and controll the misalignment to the goal, if there is no obstacle in front of the robot closer then 1 and the misalignment is under the threshold 0.05 the state is changed back to 0.
    finally if the state is the 2 this method just break.
    if none of these case are fulfilled then the robot is in an unknown state, an error log message is displayed and the method wait.
    if in all of the cases the robot reach the goal, and so the variable success is set to true, the node display a log message and set the goal as succeeded.
    
    Return: None
    """
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    change_state(0)
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    
    feedback = assignment_2_2022.msg.PlanningFeedback()
    result = assignment_2_2022.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                        pow(desired_position_.x - position_.x, 2))
        if act_s.is_preempt_requested():
            rospy.loginfo("Goal was preeempted")
            feedback.stat = "Target cancelled!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            success = False
            change_state(2)
            done()
            break
        elif err_pos < 0.5:
            change_state(2)
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break       
        elif regions_ == None:
            continue
        
        elif state_ == 0:
            feedback.stat = "State 0: go to point"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            if regions_['front'] < 0.2:
                change_state(1)
        elif state_ == 1:
            feedback.stat = "State 1: avoid obstacle"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)
        elif state_== 2:
            break
            
            
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
    
    

def main():
    """
    Args: None

    This method initially wait 2 second to allow the other nodes to start, then the node is initialized, the goal is initially set as the initial position, the node subscribe to the topic */scan* for the scan data and the topic */odom* to obtain the position informations of the robot.
    the node also create a publisher to the topic */cmd_vel* to control the robot motion and connect to both the services */go_to_point_switch* and */wall_follower_switch* to activate them when needed.
    finally the node create an action server under the topic */reaching_goal* to control the motion of the robot till the goal position.
    
    Return: None
    """
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    
    desired_position_.x = 0.0
    desired_position_.y = 1.0
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2022.msg.PlanningAction, planning, auto_start=False)
    act_s.start()
   
    # initialize going to the point
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()
