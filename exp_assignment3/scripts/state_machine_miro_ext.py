#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
import std_msgs
import geometry_msgs
from rospy.numpy_msg import numpy_msg


from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib_msgs.msg
import actionlib.msg
import motion_plan.msg 
import time
from  std_msgs.msg import Int32, String 
from actionlib_msgs.msg import GoalID


# INSTALLATION
# - create ROS package in your workspace:
#          $ catkin_create_pkg smach_tutorial std_msgs rospy
# - move this file to the 'smach_tutorial/scr' folder and give running permissions to it with
#          $ chmod +x state_machine.py
# - run the 'roscore' and then you can run the state machine with
#          $ rosrun smach_tutorial state_machine.py
# - install the visualiser using
#          $ sudo apt-get install ros-kinetic-smach-viewer
# - run the visualiser with
#          $ rosrun smach_viewer smach_viewer.py
# source ~/my_ros/devel/setup.bash 

rec_cmd = 'goNormal'
tgx = 0
tgy = 0

global detected
detected = 0
global status
status = 0
global xc
global yc

global found_green
global found_black
global found_red
global found_magenta
global found_yellow
global found_blue
found_green = 0
found_black = 0
found_red = 0
found_magenta = 0
found_yellow = 0
found_blue = 0
global pose_green 
global pose_black 
global pose_red 
global pose_magenta 
global pose_yellow 
global pose_blue 

pose_green = Pose()
pose_black = Pose()
pose_red = Pose()
pose_magenta = Pose()
pose_yellow = Pose()
pose_blue = Pose()

global pose_
global location_goal
location_goal = ''
def detectedCallback(data):
    """Callback to read an integer from a topic
        this integer tells if the ball has been detected or not

    
    """
    global detected
    detected = data.data

def clbk_odom(msg):
    global position_
    global yaw_
    global pose_
    pose_= msg

    # position
    position_ = msg.pose.pose.position

    global xc
    xc = position_.x
    global yc
    yc = position_.y

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



def random_coord():
    """Generates a random coordinate

    Returns:
        [int]: [integer coordinate]
    """
    return random.choice([-6, -4, -2, 0, 2, 4, 6])

def cmdCallback(data):
    global status
    if data.status_list:
        statuslist =  data.status_list
        #print(statuslist)
        #print(statuslist[0].status)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s",  statuslist[1])
        status = statuslist[0].status
    else:
        status = 0
    
def ballcCallback(data):
    global ball_color
    global position_
    ball_color = data
    if ball_color == "green": 
        found_green = 1
        pose_green = pose_
        print (ball_color, "found")
    elif ball_color == "black":
        pose_black = pose_
        found_black = 1
        print (ball_color, "found")
    elif  ball_color == "blue":
        found_blue = 1
        pose_blue = pose_
        print (ball_color, "found")
    elif   ball_color == "red":
        found_red = 1
        pose_red = pose_
        print (ball_color, "found")
    elif  ball_color == "magenta":
        found_magenta = 1
        pose_magenta = pose_
        print (ball_color, "found")
    elif  ball_color == "yellow":
        found_yellow = 1
        pose_yellow = pose_
        print (ball_color, "found")
def locationGoalCallback(data):
    global location_goal
    location_goal = data


# define state Sleep
class Sleep(smach.State):
    """Defines the state SLEEP

    
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['goSleep','goNormal'],
                             input_keys=['sleep_counter_in', 'previous_state'],
                             output_keys=['sleep_counter_out', 'current_state'])
        self.pub = rospy.Publisher('miro_state', std_msgs.msg.Int32 , queue_size=1)
        self.pubgoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        global status
        
    def execute(self, userdata):
        userdata.current_state = 'goSleep'
        # function called when exiting from the node, it can be blacking
        status_id = 0
        #rospy.loginfo(status_id)
        self.pub.publish(status_id)
        plan = PoseStamped()
        plan.pose.position.x = 0
        plan.pose.position.y = 0
        plan.pose.orientation.w = 1
        plan.header.frame_id='map'
        self.pubgoal.publish(plan)
        rospy.loginfo('SLEEP, miro is tired, commands will be ignored for some time')
        #time.sleep(15)
        while status!=3:
            #rospy.loginfo("status is %d"%status)
            time.sleep(1)
        time.sleep(5)
        
        userdata.sleep_counter_out = 0
        return 'goNormal'
    

# define state NORMAL
class Normal(smach.State):
    """Defines the state NORMAL

    
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goSleep','goNormal', 'goPlay', 'goTrack'],
                             input_keys=['sleep_counter_in', 'previous_state'],
                             output_keys=['sleep_counter_out', 'current_state'])
        self.Normal_counter = 0
        self.pub = rospy.Publisher('miro_state', std_msgs.msg.Int32 , queue_size=1)
        #rospy.Subscriber("ext_command", assignment1.msg.Command, cmdCallback)
        global detected
        self.pubgoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/move_base/status",  actionlib_msgs.msg.GoalStatusArray, cmdCallback)
        global status
        self.canc_goalpub = rospy.Publisher('/move_base/cancel', GoalID , queue_size=1)

        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        global still_exploring 
        still_exploring = 1
    
    def execute(self, userdata):
        plan = PoseStamped()
        """ plan.pose.position.x = random_coord()
        plan.pose.position.y = random_coord() """
        plan.pose.position.x = random_coord()
        plan.pose.position.y = random_coord()
        plan.pose.orientation.w = 1
        plan.header.frame_id='map'
        #rospy.loginfo('going to point')
        self.pubgoal.publish(plan)
        while not rospy.is_shutdown():  
            userdata.current_state = 'goNormal'
            rospy.Subscriber("object_detection", Int32, detectedCallback)
            #check to see if previous planning action has suceeded
            print(detected, status)
            if still_exploring == 0:
                if detected == 0 and (status==0 or status ==3):
                    print ("ciao")
                
                    plan = PoseStamped()
                    """ plan.pose.position.x = random_coord()
                    plan.pose.position.y = random_coord() """
                    plan.pose.position.x = random_coord()
                    plan.pose.position.y = random_coord()
                    plan.pose.orientation.w = 1
                    plan.header.frame_id='map'
                    #rospy.loginfo('going to point')
                    self.pubgoal.publish(plan)
                    time.sleep(2)

            if detected == 1:
                #cancel the actual command and go to play state
                canc = GoalID()
                
                self.canc_goalpub.publish(canc)
                return  'goTrack'
            global location_goal
            if location_goal == 'play':
                print ("going by the human")                
                plan = PoseStamped()
                """ plan.pose.position.x = random_coord()
                plan.pose.position.y = random_coord() """
                plan.pose.position.x = -5
                plan.pose.position.y = 7
                plan.pose.orientation.w = 1
                plan.header.frame_id='map'
                #rospy.loginfo('going to point')
                self.pubgoal.publish(plan)
                return  'goPlay'


            status_id = 1
            #rospy.loginfo(status_id)
            self.pub.publish(status_id)
            time.sleep(1)
            userdata.sleep_counter_out = userdata.sleep_counter_in + 1
            if (userdata.sleep_counter_in+1>1200):
                return  'goSleep'

            self.Normal_counter += 1
            self.rate.sleep

class Play(smach.State):
    """Defines the state PLAY

    
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goSleep','goNormal', 'goPlay', 'goTrack', 'goFind'],
                             input_keys=['sleep_counter_in', 'previous_state'],
                             output_keys=['sleep_counter_out', 'current_state'])
        self.Play_counter = 0
        self.pub = rospy.Publisher('miro_state', std_msgs.msg.Int32 , queue_size=1)
        rospy.Subscriber("location_goal", String , locationGoalCallback)
        self.rate = rospy.Rate(20)  

    def execute(self, userdata):
        global location_goal
        time.sleep(2)
        userdata.current_state = 'goPlay'
        status_id = 2
        #rospy.loginfo(status_id)
        self.pub.publish(status_id)
        rospy.loginfo('PLAY')
        while status!=3 or location_goal == 'play':
            self.rate.sleep
            self.Play_counter = self.Play_counter +1; 
            if self.Play_counter > 60:
                self.Play_counter = 0
                return  'goNormal'
        while not rospy.is_shutdown():  
            
            time.sleep(1)
            userdata.sleep_counter_out = 0

            if location_goal == "green": 
                if found_green == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_green
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
            elif location_goal == "black":
                if found_black == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_black
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
                
            elif  location_goal == "blue":
                if found_blue == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_blue
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
            elif   location_goal == "red":
                if found_red == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_red
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
            elif  location_goal == "magenta":
                if found_magenta == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_magenta
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
            elif  location_goal == "yellow":
                if found_yellow == 1:	
                    plan = PoseStamped()		
                    plan.pose = pose_yellow
                    plan.header.frame_id='map'
                    time.sleep(2)
                    while status!=3 or location_goal != 'play':
                        self.rate.sleep
                    return 'goPlay'
            else: 
                return 'goFind'
                
           
            
            
           
            
            self.rate.sleep

class Track(smach.State):
    """Defines the state Track

    
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goNormal', 'goPlay', 'goFind'],
                             input_keys=['sleep_counter_in', 'previous_state'],
                             output_keys=['sleep_counter_out', 'current_state'])
        self.Track_counter = 0
        self.pub = rospy.Publisher('miro_state', std_msgs.msg.Int32 , queue_size=1)
        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        rospy.Subscriber("ball_color", String , ballcCallback)

    def execute(self, userdata):
        while not rospy.is_shutdown(): 
            
            prev = userdata.previous_state
            userdata.current_state = 'goTrack' 
            status_id = 3
            userdata.sleep_counter_out = 0
            #rospy.loginfo(status_id)
            self.pub.publish(status_id)
            rospy.loginfo('Track, chasing ball')
            time.sleep(1)
            """ userdata.sleep_counter_out = userdata.sleep_counter_in + 1
            if (userdata.sleep_counter_in+1>120):
                return  'goSleep' """
            """ #if the ball stops being detected count and after some time if nothing is detected go back to normal state
            if detected == 0:
                self.Play_counter = self.Play_counter +1
            if detected == 1:
                self.Play_counter = 0
            
            
            if self.Play_counter > 5:
                self.Play_counter = 0
                return  'goNormal' """
            while (detected == 1):
                print ("ball found")
                self.rate.sleep
            return prev
            
            self.rate.sleep


# define state FIND
class Find(smach.State):
    """Defines the state FIND

    
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goNormal', 'goTrack', 'goPlay'],
                             input_keys=['sleep_counter_in', 'previous_state'],
                             output_keys=['sleep_counter_out', 'current_state'])
        self.Normal_counter = 0
        self.pub = rospy.Publisher('miro_state', std_msgs.msg.Int32 , queue_size=1)
        #rospy.Subscriber("ext_command", assignment1.msg.Command, cmdCallback)
        global detected
        self.pubgoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/move_base/status",  actionlib_msgs.msg.GoalStatusArray, cmdCallback)
        global status
        self.canc_goalpub = rospy.Publisher('/move_base/cancel', GoalID , queue_size=1)

        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        global still_exploring 
        still_exploring = 1
    
    def execute(self, userdata):
        while not rospy.is_shutdown():  
            status_id = 4
            #rospy.loginfo(status_id)
            self.pub.publish(status_id)
            prev = userdata.previous_state
            userdata.current_state = 'goFind'
            rospy.Subscriber("object_detection", Int32, detectedCallback)
            print(detected, status)
            global location_goal
            # check if a new "play" command has been generated from the user
            if location_goal == "play"
                return "goPlay"
            #check if the ball has been found in the previous state track
            #if it has, go back to track
            if prev == 'goTrack':                
            	if location_goal == "green": 
                    if found_green == 1:	
                        location_goal = 'play'
                        return 'goPlay'
                elif location_goal == "black":
                    if found_black == 1:	
                        location_goal = 'play'
                        return 'goPlay'	
                elif  location_goal == "blue":
                    if found_blue == 1:	
                        location_goal = 'play'
                        return 'goPlay'
                elif   location_goal == "red":
                    if found_red == 1:	
                        location_goal = 'play'
                        return 'goPlay'
                elif  location_goal == "magenta":
                    if found_magenta == 1:	
                        location_goal = 'play'
                        return 'goPlay'
                elif  location_goal == "yellow":
                    if found_yellow == 1:	
                        location_goal = 'play'
                        return 'goPlay'
                    
            
            if still_exploring == 0:
                if detected == 0 and (status==0 or status ==3):
                    print ("ciao")
                
                    plan = PoseStamped()
                    """ plan.pose.position.x = random_coord()
                    plan.pose.position.y = random_coord() """
                    plan.pose.position.x = random_coord()
                    plan.pose.position.y = random_coord()
                    plan.pose.orientation.w = 1
                    plan.header.frame_id='map'
                    #rospy.loginfo('going to point')
                    self.pubgoal.publish(plan)
                    time.sleep(2)

            if detected == 1:
                #cancel the actual command and go to play state
                canc = GoalID()
                
                self.canc_goalpub.publish(canc)
                return  'goTrack'
            
           


            time.sleep(1)
            userdata.sleep_counter_out = userdata.sleep_counter_in + 1
            if (userdata.sleep_counter_in+1>60):
                return  'goNormal'

            self.find_counter += 1
            self.rate.sleep



def main():
    """Initialization of the finite state machine
    """
    rospy.init_node('miro_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0
    sm.userdata.prev_state = 'NORMAL'


   

    # Open the container
    with sm:
       
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'goSleep':'SLEEP', 
                                            'goNormal':'NORMAL',
                                            'goPlay':'PLAY',
                                            'goTrack':'TRACK'},
                               remapping={'sleep_counter_in':'sm_counter', 
                                          'sleep_counter_out':'sm_counter', 
                                          'previous_state':'prev_state',
                                          'current_state':'prev_state'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goSleep':'SLEEP', 
                                            'goNormal':'NORMAL'},
                               remapping={'sleep_counter_in':'sm_counter',
                                          'sleep_counter_out':'sm_counter',
                                          'previous_state':'prev_state',
                                          'current_state':'prev_state'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goSleep':'SLEEP', 
                                            'goNormal':'NORMAL',
                                            'goPlay':'PLAY',
                                            'goTrack':'TRACK',
                                            'goFind':'FIND'},
                               remapping={'sleep_counter_in':'sm_counter',
                                          'sleep_counter_out':'sm_counter',
                                          'previous_state':'prev_state',
                                          'current_state':'prev_state'})
        smach.StateMachine.add('TRACK', Track(), 
                               transitions={'goNormal':'NORMAL',
                                            'goPlay':'PLAY',
                                            'goFind':'FIND'},
                               remapping={'sleep_counter_in':'sm_counter',
                                          'sleep_counter_out':'sm_counter',
                                          'previous_state':'prev_state',
                                          'current_state':'prev_state'})
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'goNormal':'NORMAL',
                                            'goPlay':'PLAY',
                                            'goTrack':'TRACK'},
                               remapping={'sleep_counter_in':'sm_counter',
                                          'sleep_counter_out':'sm_counter',
                                          'previous_state':'prev_state',
                                          'current_state':'prev_state'})
        
        
                                                    
                                    

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_miro', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
   

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

