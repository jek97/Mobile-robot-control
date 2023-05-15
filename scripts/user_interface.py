#! /usr/bin/env python3

import rospy
import actionlib
import actionlib.msg
from assignment_2_2022.srv import Ngoal
import assignment_2_2022.msg
import time
import sys
import select
import os   

"""
..module:: user_interface
  :platform: Unix
  :synopsis: Python module for the robot user interface
  :version: 0.1

..moduleauthor:: Giacomo Lugano jek.lugano@yahoo.com

this node menage the interaction with the simulation, through it it's possible to set a new goal, delete the current one, call the service provided by info_server to see how many goal have been reached and how many have been deleted

Parameters:
   n_goal (input/output)
   n_deleted (input/output)

Service:
   /info (client)

Action:
   /reaching_goal (client)

"""

def main():
    gf = 0 # goal flag to increase the number of goal only the first time the state pass to succeeded
    rospy.init_node('user_interface', anonymous=True) # init the node
    act_clnt = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    act_clnt.wait_for_server()
    srv_info = rospy.ServiceProxy("/info", Ngoal)
    rospy.wait_for_service("/info")

    goal = assignment_2_2022.msg.PlanningGoal() 
    goal.target_pose.pose.position.z = 0.0
    info = Ngoal()
    n_goal = 0 # number of goal reached
    n_deleted = 0 # number of goal deleted
    count = 0
    rospy.set_param('n_goal', n_goal) # setting the parameters in the paramserver
    rospy.set_param('n_deleted', n_deleted) # setting the parameters in the paramserver
    time.sleep(3)
    while True:
        if (act_clnt.get_state() == 3 and gf == 0): # if i've reached the goal
            n_goal = rospy.get_param('n_goal') # get the actual number of goal
            n_goal = n_goal + 1 # increase it
            rospy.set_param('n_goal', n_goal) # load it in the paramserver
            gf = 1
        elif (act_clnt.get_state() != 3):
            gf = 0

        if (count > 0): # to display the information if the counter is bigger then 0 decrease it
            count = count - 1

        elif (count == 0): # if it's 0 then clear the screan and write the menu
            os.system('cls' if os.name == 'nt' else 'clear')
            log = "press N for setting a new goal, D to delete the previous one, I for getting information about the number of goal reached and deleted:"
            rospy.loginfo(log)

        v = select.select([sys.stdin], [], [], 0.5)[0] # get the user input           
        if v:                                                    
            f = sys.stdin.readline().rstrip()
        else:
            f = 0
            continue

        if (f == "n"): # then set the new goal
            log = "plese enter the x coordinate of the goal:"
            rospy.loginfo(log)
            goal.target_pose.pose.position.x = float(input()) # setting the x component of the goal
            log = "plese enter the y coordinate of the goal"
            rospy.loginfo(log)
            goal.target_pose.pose.position.y = float(input()) # setting the y component of the goal
            act_clnt.send_goal(goal) # sending the goal to the action server
            count = 0 # you can clear and display again the menu
        elif (f == "d" and (act_clnt.get_state()) == 1): # in this way you're deleting the goal while there is one active
            act_clnt.cancel_goal() # delete the goal
            n_deleted = rospy.get_param('n_deleted') # get the actual number of goal deleted
            n_deleted = n_deleted + 1 # increase it
            rospy.set_param('n_deleted', n_deleted) # load it in the paramserver
            count = 0 # you can clear and display again the menu
        elif (f == "i"): # get information about the goal
            info = srv_info()
            log = "number of Goal reached: %f; number of Goal deleted: %f" % (info.n_goal, info.n_deleted)
            rospy.loginfo(log)
            count = 10 # leave the infromation on screen for 5 seconds

if __name__ == "__main__":
    main()