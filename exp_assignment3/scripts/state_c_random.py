#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import time
import random
import string


def random_cmd():
    """Generates a random coommand

    Returns:
        [char]: [id of a command]
    """
    return random.choice(['y','n', 'n'])
def random_room():
    """Generates a random letter

    Returns:
        [char]: [id of a possible room]
    """
    return random.choice(string.ascii_lowercase)
def random_big_wait():
    """Generates a random coommand

    Returns:
        [char]: [id of a command]
    """
    return random.choice([40, 50, 60, 90])

def random_little_wait():
    """Generates a random coommand

    Returns:
        [char]: [id of a command]
    """
    return random.choice([2, 5, 10, 20, 30])

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
        cmdt = random_cmd()
        if cmdt=='y':
            cmd = String()
            cmd.data = 'play'
            pub.publish(cmd)
            time.sleep(random_little_wait())
        
            rospy.loginfo('choose location to reach: ([e]ntrance, [c]loset, [l]ivingroom, [k]itchen, [b]athroom. be[d]room):')
            pcmd = cmd
            cmdt = random_room()
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
        if sendcmd == 1:
            pub.publish(cmd)
            sendcmd = 0
            """  time.sleep(1)
            cmd.data = ''
            pub.publish(cmd) """
        
        
        """ cmd.command = ''
        pub.publish(cmd) """
        time.sleep(random_big_wait())

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass