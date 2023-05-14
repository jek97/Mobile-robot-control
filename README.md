# ROS bug0 controll of a robot #
===================================
this repository contain the implementation of an autonomous mobile robot able to move following the bug0 algorithm to the given target.
The implementation use the Gazebo simulator together with the Rviz visualizer to give a direct rappresentation of what's going on, in detail the differential robot is provided by a lidar, to perform an external measure of the environment, and encoders, to perform an internal measure and so the odometry, all these component are then all combined in a ROS architecture.
The starting code can be downloaded at the link: [GitHub repository](https://github.com/CarmineD8/assignment_2_2022)

## How to run the project ##
Note that to run the project it's necessary to have both ROS (Noetic) and Python3 installed on your machine.
1. Download the package from github: [GitHub repository](https://github.com/jek97/RT1.git)
2. instal xterm, by the command:
```
sudo apt-get install xterm
```
3. Move to the root folder of the work space with the command:
```
$ cd RT1_ros_ws/src/assignment_2_2022
```
4. Build the environment with the command:
```
$ catkin_make
```
5. Launch the program by the command:
```
$ roslaunch assignment_2_2022 assignment1.launch
```
## project structure ##
the package is organized in the following directories:
* action: it contain the structure of the action used by the program, in it the Planning action is provided, it will allows the user to set a goal and receive the status of that task once concluded.
* config: this directory contain the configuration of the robot, in our case we will use the sim2.rvis one in which the robot with its own encoders and a lidar is described.
* launch: in it there are two launch file for the project, the first one (assignment1.launch) will be used to launch our simulation while the second will launch another project still in developement in which the robot will use the oin-board camera.
* msg: it contains the custom messages designed for our project, the custom message will be used to communicate the actual position and velocity of the robot and will be investigated more in detail hereafter.
* scripts: this folder contain the scripts of all the nodes needed in this project, as we can see all the nodes have been implemented using Python3.
* srv: inside this folder there are the custom services with the description of the request and response messages, in our case only one custom service has been desined, the Ngoal.srv, it will be used to retrieve the number of goal reached and deleted and will be further discussed in the following.
* urdf: this folder contain the description of the robot for the Gazebo simulator, as for the config folder two robots description are present, one for each project.
* world: in it a description of the world is given for the simulation, rapresenting the dimensions and displacement of the environment.

the project is divided in eleven nodes as described in the figure:
![Flowchart](./rograph.png)
in detail:
* the joint_state_publisher and the robot_state publisher: are provided by gazebo for the control of any possible arm mounted on the robot, in our case the robot has no need for them and they will not be used.
* gazebo_gui: this node will show the simulation allowing us to interact if needed, for what regards the settings.
* gazebo: it's the real simulation engine of the program, it will simulate the physics of the world and of the robot, giving back the robot feedbacks like the odometry and the lidar scan data, allowing also to command the robot.
* go_to_point: it implement the simple behavior of the robot of, given a goal position, reach it moving in straight line, it will be integrated with the other nodes to perform the bug0 algorithm.
by looking at the code:
```
def main():
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
```
the node will, after initialization, create a publisher to the topic *'/cmd_vel'* of type Twist, it's the topic used to communicate with the robot giving it the velocity, both linear and angular at each step; after that the node will subscribe to the topic *'/odom'* of type Odomentry, that will give a feedback to the algorithm giving the actual position and velocity of the robot;
moreover the node will check the service *'go_to_point'* switch that will be used by the *'bug_action_server'* node to switch between the behavior of this node and the one of *'wall_follower'*.
Thanks to the above mentioned subscription to the topic *'/odom'* all the times the robot receive a new message it will update its position and orientation by the callback *'clbk_odom()'*.
finished this initial phase the node will check the actual goal position in the ros parameter server under the names *'des_pos_x'* and *'des_pos_y'*, initially will be in state 0 and so will call the function *'fix_yaw'*:
```
def fix_yaw(des_pos):
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
```
this method will evaluate the heading error between the robot and the goal and if over a threshold will pulish a rotational speed to allign with it; otherwise it will change the current state of the node by the function *'change_state'* that will notify the changes.
moving in the state 1 in the next iteration the function *'go_straight_ahead()'* will be called:
```
def go_straight_ahead(des_pos):
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
```
this method will use a PID controller to move the robot in a straight line to the goal, passed as argument, then if this motion increase the misalignment of the robot over a threshold the state will be changed back to 0 to correct this fact; otherwise if the distance between the robot and the goal fall under a certain threshold the node will switch to the new state 2, calling at the next iteration of the main the *'done()'* method that will simply set the velocity to zero and publish it.
* wall_follower: it implement the behavior of, one an obstacle is encountered along the path, following the wall perimeter, maintaining the wall on the robot right side at a given distance.
in detail the node after initialization will connect as a publisher to the topic *'/cmd_vel'*, to controll the robot motion, subscribe to the topic *'/scan'*, to get the information of the lidar and, as done before by the *'go_straight_ahead'* node will connect to the analogous service *'wall_follower_switch'* to give the possibility to the node to activate or deactivate this node:
```
def main():
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
```
all the times the node will receive new data from the topic *'/scan'* it will call the relative callback:
```
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()
```
that will check for the closer wall in the 5 circular sector in front of the robot ad cal the method *'take_action()'*:
```
def take_action():
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
```
that based on the position of the closest wall will change the state of the node.
if there is no wall closer then the threshold in front of the robot, or there is one but on the left or on both left and right the node will move in teh state 0 that, with the next iteration of the main will call the method *'find_wall()'*:
```
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg
```
that will set the message to publish on *'/cmd_vel'* to move the robot along a circular trajectory till it find a wall.
if instead there is a wall infront of the robot, or in front and on the right, or in front and on the left or all around the robot, the node will switch to the state 1, that at the next iteration will call the method *'turn_left()'*:
```
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg
```
which will set the message to publish on *'/cmd_vel'* to turn the robot on the left.
if the robot has a wall on it's right it will switch on the state 2 and on the next iteration will call the method *'follow_the_wall()'*:
```
def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg
```
that will set the message to publish on *'/cmd_vel'* to move straight.
after the creation of the message based on the current state it will be published by the *'main()'* function.
* bug_action_service: this node, based on a state machine architecture, together with the two above mentioned node go_to_point and wall_follower will menage the implementation of the bug0 algorithm, indeed based on the lidar information will decide when to switch between the go_to_point behavior and the wall_follower one.
by looking at the code:
```
def main():
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
```
initially 2 second are waite dto allow the other nodes to start, then the node is initialized, the goal is initially set as the initial position, the node subscribe to the topic *'/scan'* for the scan data and the topic *'/odom'* to obtain the position informations of the robot.
the node also create a publisher to the topic *'/cmd_vel'* to control the robot motion and connect to both the services *'/go_to_point_switch'* and *'/wall_follower_switch'* to activate them when needed.
finally the node create an action server under the topic *'/reaching_goal'* to control the motion of the robot till the goal position.
all the times the node receives some information form the topic *'/scan'*: 
```
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
```
it checks which is the minimum distance between the robot and the environment objects in the five circular sectors in front of the robot and set that distance value under one name for sector.
instead when it receive information from the topi *'/odom'*:
```
def clbk_odom(msg):
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
```
it aquire the position information and put them in the variables *'position'*, *'pose_'* and *'yaw_'* in this way they are ready to use for the rest of the algorithm.
except these two callbacks the algorithm is mainly controlled by the action server, which one started (in the main right after its definition) will call the method *'planning'*:
```
def planning(goal):
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
```
it initially set the goal received from the action client in the ros parameter server, then getting inside the while loop it compute the distance respect the goal.
after that initially the algorithm check if the action client required to delete the goal, and in this case the right feedback message is set and pusblished, the goal is preempted, the state is changed to 2 (the change state method will be analized right after), and the robot speed is set to zero.
otherwise if the distance to the goal is less then 0.5 we assume it to be reached, the state is chaged to 2, as before the right feedback message is set and published and tho robot is blocked in its position.
in case no wall is detected (so the array *'regions_'* is void) the algorithm just contineu.
then if the state is set to 0, so the motion is controlled by the *'go_to_point'* behavior, the node send a feedback and if there is an obstacle nearer then 0.2 in front of the robot it changes teh state to 1.
instead if the state in which we are is the 1, so the motion is controlled by the *'wall_follower'* behavior, the node send a feedback and controll the misalignment to the goal, if there is no obstacle in front of the robot closer then 1 and the misalignment is under the threshold 0.05 the state is changed back to 0.
finally if the state is the 2 this method just break.
if none of these case are fulfilled then the robot is in an unknown state, an error log message is displayed and the method wait.
if in all of the cases the robot reach the goal, and so the variable success is set to true, the node display a log message and set the goal as succeeded.
the states are controlled by the method *'change_state()'*:
```
def change_state(state):
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
```
this method basically change the state of the robot between the *'go_to_point'* behavior and the *'wall_follower'* one based on the state set by the rest of the algorithm
* node_a: it works as an adapter converting the odometry information received in the custom message Conf.msg type:
```
def main():
    rospy.init_node('node_a', anonymous=True) # init the node
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_cb) # sub to odom to obtain position and velocity

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()
```
the node initialize itself, create a publisher to the topic *'/configuration'* (outside of the main in such a way it can be used by the callback), subscribe to the topic *'/odom'* and wait for the master to be launched.
then at each message received the node call the callback *'odom_cb()'*:
```
def odom_cb(msg):
    # received the message retrieve the position and velocity and fulfill the custom message
    confy.x = msg.pose.pose.position.x
    confy.y = msg.pose.pose.position.y
    confy.Vx = msg.twist.twist.linear.x
    confy.Vy = msg.twist.twist.linear.x

    # publish the message on the given topic
    pub_conf.publish(confy) 
```
which simpluy aquire the message fulfill the custom message confy and pusblish it under the topic *'/configuration'*.
* node_c: it will use the information received to print on the terminal the actual distance to the goal and the average speed of tyhe robot:
```
def main():
    rospy.init_node('node_c', anonymous=True) # init the node
    sub_confy = rospy.Subscriber('/configuration', assignment_2_2022.msg.Conf, confy_cb) # sub to odom to obtain position and velocity

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()
```
the node initialize itself, subscribe to the topic *'/configuration'* to aquire the information abou the robot position and velocity and wait for the master to be launched, after that the node will spin.
anytime the node receive a message it will call the associated callback *'confy_cb()'*:
```
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
```
in which first the goal is retrieved form the ros parameter server, the distance between the robot position and the goal is evaluated together with the average speed, all controlled by the counter *'n'* used for the average operation, if it's a multiple of 40 then the two information are displaied.
* info_server: this node will provide the system with a service, used by the user_interface for counting the number of goal reached and the number of goal deleted upon request.
in details:
```
def info_server():
    rospy.init_node('info_server')
    rospy.Service('/info', Ngoal, ret_info)
    rospy.spin()
```
the node inialize, create the srvice *'/info'* and then spin, everytime the service it's called the method *'ret_info()'* is called:
```
def ret_info(req):
    return [rospy.get_param('n_goal'), rospy.get_param('n_deleted')]
```
in it the message is fulfilled with the number of goal achived and deleted, obtained from the ros parameter server, and sent back to the client.
* user_interface: it menage the interaction with the simulation, through it it's possible to set a new goal, delete the current one, call the service provided by info_server to see how many goal have been reached and how many have been deleted.
```
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
```
the node first initialize itself, then create the action client for teh action *'/reaching_goal'*, needed to set a new goal or delete it, and wait for the action server launch;
after that it connect as a client to the service *'/info'* provided by the *'info_server'* node to aquire the number of goal reached and deleted upon request, and wait for its launch.
then some variable are initialized and the number of goal reached and deleted is initialized in the ros parameters server.
getting inside the while loop the node check if the goal has been reached, by checking the state of the action server and that it's the first time it has been checked since the goal was achived, if this is the case the number of goal is retrieved from the ros parameters server, increased and set back in the ros parameters server.
otherwise the goal flag *'gf'* is set to 0 again, to avoid to increase the number of goal in the time periods in which the goal has been reached and we're waiting for a new one.
after that the counter *'count'* is decreased, if it's bigger then 0, this counter will be used to clear the terminal only when needed.
done that if the counter reach zero the terminal is cleared, the menu message is displayed and the node wait for a user input in non blocking mode, based on it:
1. if the user type *'n'* the node ask to set the new goal position in it's x and y coordinates, the new goal is sended to the action server and the counter is set to zero.
2. if the user type *'d'* and the robot is actually tring to reach a goal, checked by the action server state, the node will delete the goal and increase the number of goal deleted in te ros parameters server.
3. if the user type *'i'* the node will use the *'/info'* service to aquire the number of goal achieved and deleted and will display it on the terminal, the counter will be then set to 10 to maintain the message on the terminal for almost 5 seconds.
