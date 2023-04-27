#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def move_base(distance, direction):
    """
    this function sends velocity commands to the TIAGo mobile base controller
    :distance: meters or radians to move
    :direction: 0 for x-direction, 1 for y-direction, 2 for z-direction/turning
    """

    # Initializing the Node 
    rospy.init_node('move_base',log_level=rospy.DEBUG, anonymous=True)
    rospy.loginfo("Node %s initialized", 'move_base')
  
    # Defining the publisher for the 'cmd_vel' like topic for TIAGo
    rospy.logdebug("Publishing to the %s custom topic with the motion_callback function", '/mobile_base_controller/cmd_vel')
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    rate = 3
    r = rospy.Rate(3)
    
    # Initializing variables
    rospy.logdebug("%s message, initializing...", 'Twist')
    pub_msg = Twist()
    pub_msg.linear.x = 0
    pub_msg.linear.y = 0
    pub_msg.linear.z = 0
    pub_msg.angular.x = 0
    pub_msg.angular.y = 0
    pub_msg.angular.z = 0
    
    # Calculating variables to reach goal
    lin_velocity = 0.2 #m/s
    ang_velocity = 0.4 #rad/s
    if direction == 0:
        pub_msg.linear.x = lin_velocity
        time = distance/lin_velocity
    if direction == 1 :
        pub_msg.linear.y = lin_velocity
        time = distance/lin_velocity
    if direction == 2 :
        pub_msg.angular.z = ang_velocity
        time = distance/ang_velocity

    # Publishing the goal according to the time duration of the 
    # client's request
    for i in range(0, int(time) * rate):
      rospy.logdebug("%s message, Sending...", 'pub_msg')
      pub.publish(pub_msg)
      r.sleep()

if __name__ == '__main__':
    try:
        move_base(0.4, 0) 
        move_base(0.8, 2) 
    except rospy.ROSInterruptException:
        pass
