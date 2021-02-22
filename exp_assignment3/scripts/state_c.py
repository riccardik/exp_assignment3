#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import time
import random



def talker():
    """ 
    initialize a ROS publisher. 
  
    The function takes from the shell input (or generates randomly) a command and publish it into the goal topic 
  
  
    """
    pub = rospy.Publisher('location_goal', String, queue_size=10)
    rospy.init_node('cmd_generator', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    sendcmd = 0
    

    while not rospy.is_shutdown():
        rospy.loginfo('do you want to send a command? (y, n):')
        cmdt = raw_input()
        if cmdt=='y':
            cmd = String()
            cmd.data = 'play'
            pub.publish(cmd)
            time.sleep(1)
        
            rospy.loginfo('choose location to reach: ([e]ntrance, [c]loset, [l]ivingroom, [k]itchen, [b]athroom. be[d]room):')
            pcmd = cmd
            cmdt = raw_input()
            if cmdt == 'e':
                cmd.data = 'blue'
                sendcmd = 1
            if cmdt == 'c':
                cmd.data = 'red'
                sendcmd = 1
            if cmdt == 'l':
                cmd.data = 'green'
                sendcmd = 1
            if cmdt == 'k':
                cmd.data = 'yellow'
                sendcmd = 1
            if cmdt == 'b':
                cmd.data = 'magenta'
                sendcmd = 1
            if cmdt == 'd':
                cmd.data = 'black'
                sendcmd = 1
                
                
            
            
        else:
            sendcmd = 0
            rospy.loginfo('that room doesn\' esists!')

    #""" hello_str1 = "hello world %s" % hello_str
     #   rospy.loginfo(hello_str1)
#
  #      cmd = Command()
 #       cmd.command = 'reach' """
        
        if sendcmd == 1:
            pub.publish(cmd)
            sendcmd = 0
            """  time.sleep(1)
            cmd.data = ''
            pub.publish(cmd) """
        
        
        """ cmd.command = ''
        pub.publish(cmd) """
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass