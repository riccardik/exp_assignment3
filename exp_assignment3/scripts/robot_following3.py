#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import std_msgs
from  std_msgs.msg import Float64, String 
from  std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

VERBOSE = False

global reached
global mirostate
global xc
global yc
xc = 0
yc = 0
mirostate = 2
reached = 0

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

class image_feature:

    def __init__(self):
        '''Initialize ros publishers, ros subscribers'''
        rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)
        self.ballcolor_pub = rospy.Publisher("ball_color",
                                       String , queue_size=1)
        self.ang_pub = rospy.Publisher("joint1_position_controller/command",
                                           Float64,  queue_size=3)
        self.detection_pub = rospy.Publisher("object_detection",
                                           Int32,  queue_size=3)
        self.explore_abilitation_pub = rospy.Publisher("explore_abilitation",
                                           Int32,  queue_size=3)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        
        self.substate = rospy.Subscriber("miro_state",
                                           Int32, self.statecallback,  queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

    def clbk_odom(self, msg):
        global position_
        global yaw_

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

    def statecallback(self, data):
        """Callback to miro read state from a topic

    
        """
        global mirostate
        mirostate = data.data

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected
        If the ball is detected a message will be setn to the state machine
        when the state machine will commute to the PLAY state the function will send
        cmd_vel msg to the robot'''
        global reached
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        #range of color for the segmentation
       

        blackLower = (0, 0, 0)
        blackUpper = (5,50,50)

        redLower = (0, 50, 50) 
        redUpper = (5, 255, 255)

        yellowLower = (25, 50, 50) 
        yellowUpper = (35, 255, 255) 

        greenLower = (50, 50, 50) 
        greenUpper = (70, 255, 255)

        blueLower = (100, 50, 50) 
        blueUpper = (130, 255, 255)

        magentaLower = (125, 50, 50) 
        magentaUpper = (150, 255, 255)

        """ blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        ballcolor = "green"
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts) """

        cntsg = find_centrodes(greenLower, greenUpper, np_arr, image_np)
        cntsbk = find_centrodes(blackLower, blackUpper, np_arr, image_np)
        cntsr = find_centrodes(redLower, redUpper, np_arr, image_np)
        cntsy = find_centrodes(yellowLower, yellowUpper, np_arr, image_np)
        cntsbl = find_centrodes(blueLower, blueUpper, np_arr, image_np)
        cntsm = find_centrodes(magentaLower, magentaUpper, np_arr, image_np)


        arr =np.array( [[cntsg, len(cntsg), 'green'], 
            [cntsbk, len(cntsbk), 'black'],
            [cntsr, len(cntsr), 'red'],
            [cntsy, len(cntsy), 'yellow'],
            [cntsbl, len(cntsbl), 'blue'],
            [cntsm, len(cntsm), 'magenta']] )
        
        #print(arr)
        """ np.sort(arr, axis=1)
        arr[arr[:,1].argsort()] """
        # arr[np.lexsort(arr.T[1])]
        sortedArr = arr[arr[:,1].argsort()]

        #print(arr)
        #print("lun",  len(cntsr))
        #print (sortedArr)
        global found_green
        global found_black
        global found_red
        global found_magenta
        global found_yellow
        global found_blue

        cnts = sortedArr[5][0]
        ballcolor = sortedArr[5][2]
        #if len(cnts)>0:
            #print (cnts, ballcolor, 'detected')

        if ballcolor == "green": 
            if found_green == 1:
                cnts =""
                print (ballcolor, "already found")
        elif ballcolor == "black":
            if found_black == 1:
                cnts = ""
                print (ballcolor, "already found")
        elif  ballcolor == "blue":
            if found_blue == 1:
                cnts = ""
                print (ballcolor, "already found")
        elif   ballcolor == "red":
            if found_red == 1:
                cnts = ""
                print (ballcolor, "already found")
        elif  ballcolor == "magenta":
            if found_magenta == 1:
                cnts = ""
                print (ballcolor, "already found")
        elif  ballcolor == "yellow":
            if found_yellow== 1:
                cnts = ""
                print (ballcolor, "already found")
        
            
            
        center = None

        
        # only proceed if at least one contour was found
        if len(cnts) > 0 :
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            detected = Int32()
            detected.data = 1
            self.detection_pub.publish(detected)
            #print ('radius', radius, 'mirostate', mirostate)
            #print ('mirostate: [%d]' % mirostate)
            # only proceed if the radius meets a minimum size
            print ('radius0 ', radius)
            if radius > 32 and mirostate == 2:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)
                #print(reached)
                global xc
                global yc
                """ if reached !=1:
                    explore_abilitation.data = 0
                    if radius <50:
                        xfront
                    else:
                        self.vel_pub.publish(vel)
                        explore_abilitation = Int32()
                        
                        self.explore_abilitation_pub.publish(explore_abilitation) """
                if reached !=1:
                    explore_abilitation = Int32()
                    explore_abilitation.data = 0
                    self.vel_pub.publish(vel)
                    
                    
                    self.explore_abilitation_pub.publish(explore_abilitation)
 
                
                if radius > 70:
                    print ('color', ballcolor)
                    vel.angular.z = 0
                    vel.linear.x = 0
                    self.vel_pub.publish(vel)
                    time.sleep(1)
                    print ('position: "%f, %f"' % (xc, yc))
                    reached = 1
                    explore_abilitation = Int32()
                    explore_abilitation.data = 1
                    self.explore_abilitation_pub.publish(explore_abilitation)
                    if ballcolor == "green": 
                        found_green = 1
                        print (ballcolor, "found")
                    elif ballcolor == "black":
                        found_black = 1
                        print (ballcolor, "found")
                    elif  ballcolor == "blue":
                        found_blue = 1
                        print (ballcolor, "found")
                    elif   ballcolor == "red":
                        found_red = 1
                        print (ballcolor, "found")
                    elif  ballcolor == "magenta":
                        found_magenta = 1
                        print (ballcolor, "found")
                    elif  ballcolor == "yellow":
                        found_yellow = 1
                        print (ballcolor, "found")
                    ballc_m = String()
                    ballc_m.data = ballcolor
                    self.ballcolor_pub.publish(ballc_m)
        
                #rospy.loginfo('%d'%reached)
                """ if  vel.angular.z < 0.05 and vel.angular.z > -0.05 and vel.linear.x < 0.05 and vel.linear.x > -0.05 and reached == 0:
                    rospy.loginfo('Moving head')
                    print ('find ball: "%s"' % ballcolor)
                    global xc
                    global yc
                    print ('position: "%f, %f"' % (xc, yc))
                    vel.angular.z = 0
                    vel.linear.x = 0
                    self.vel_pub.publish(vel)
                    ang1 = Float64()
                    ang1.data = 0
                    self.ang_pub.publish(ang1)
                    time.sleep(2)
                    ang1.data = -0.7
                    self.ang_pub.publish(ang1)
                    time.sleep(2)
                    ang1.data = 0
                    self.ang_pub.publish(ang1)
                    time.sleep(1)
                    ang1.data = 0.7
                    self.ang_pub.publish(ang1)
                    time.sleep(2)
                    ang1.data = 0
                    self.ang_pub.publish(ang1)
                    #set reached to 1 to do the head movement just one time
                    reached = 1
                elif  vel.angular.z > 0.05 and vel.linear.x > 0.05 :
                    
                    reached = 0

            else:
                if  mirostate == 2:
                    #aligned
                    vel = Twist()
                    vel.linear.x = 0.5
                    self.vel_pub.publish(vel)
                    reached = 0 """

        else:
            #green not found
            """  vel = Twist()
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)
            reached = 0 """
            detected = Int32()
            detected.data = 0
            self.detection_pub.publish(detected)
            reached = 0
            """ explore_abilitation = Int32()
            explore_abilitation.data = 1
            self.explore_abilitation_pub.publish(explore_abilitation) """

        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()

def find_centrodes(lower, upper, np_arr, image_np):
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #ballcolor = "green"
    #cv2.imshow('mask', mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    #print (cnts)
    return cnts


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
