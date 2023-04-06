#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def move_base(velocity, turn, time):

    # Initializing the Node 
    rospy.init_node('move_base',log_level=rospy.DEBUG, anonymous=True)
    rospy.loginfo("Node %s initialized", 'move_base')
  
    # Defining the publisher for the 'cmd_vel' like topic for TIAGo
    rospy.logdebug("Publishing to the %s custom topic with the motion_callback function", '/mobile_base_controller/cmd_vel')
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    rate = 3
    r = rospy.Rate(3)

    rospy.logdebug("%s message, initializing...", 'Twist')
    pub_msg = Twist()
    pub_msg.linear.x = velocity
    pub_msg.linear.y = 0
    pub_msg.linear.z = 0

    pub_msg.angular.x = 0
    pub_msg.angular.y = 0
    pub_msg.angular.z = turn

    # publishing the goal according to the time duration of the 
    # client's request

    for i in range(0, time * rate):
      rospy.logdebug("%s message, Sending...", 'pub_msg')
      pub.publish(pub_msg)
      r.sleep()

if __name__ == '__main__':
    move_base(0.5, 0, 2) #linear velocity, angular velocity, time
    move_base(0, 0.4, 2) 