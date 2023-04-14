#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class tiago_move_test:

    def __init__(self):
        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5) #rad/s
        self.rate = rospy.get_param("~rate", 3)

        # Initializing move_base Node to publish to 'cmd_vel' topic
        rospy.init_node('move_base',log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("Node %s initialized", 'move_base')
        rospy.logdebug("Publishing to the %s custom topic with the motion_callback function", '/mobile_base_controller/cmd_vel')
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # Subscribe to 'odom' topic 
        # rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

    def move_base(self, distance, direction):
        """
        this function sends velocity commands to the TIAGo mobile base controller
        :distance: meters or radians to move
        :direction: 0 for x-direction, 1 for y-direction, 2 for z-direction/turning
        """

        r = rospy.Rate(self.rate)
        
        # Initializing variables
        rospy.logdebug("%s message, initializing...", 'Twist')
        pub_msg = Twist()

        # Calculating variables to reach goal
        if direction == 0:
            time = round(distance/self.linear_speed)
            pub_msg.linear.x = distance/time #slightly vary linear speed
        if direction == 1 :
            time = round(distance/self.linear_speed)
            pub_msg.linear.y = distance/time #slightly vary linear speed
        if direction == 2 :
            time = round(distance/self.angular_speed)
            pub_msg.angular.z = -distance/time #slightly vary angular speed 

        # Publishing the goal according to the time duration of the 
        # client's request
        for i in range(0, int(time) * self.rate):
            rospy.logdebug("%s message, Sending...", 'pub_msg')
            self.cmd_vel_pub.publish(pub_msg)
            r.sleep()

    def odom_callback(self, odom_msg):
        rospy.loginfo(str(odom_msg.pose))

if __name__ == '__main__':
    try:
        tiago = tiago_move_test()

        # Start at center
        
        ####-------Start of Interaction--------####
        # -------- --Face the user ---------------#
        rospy.loginfo("Facing the user")
        # publish
        radians = math.asin(1.25/(1.75-0.275)) + (-0.5) #radian_offset=-0.5rad
        tiago.move_base(radians, 2)
        rospy.sleep(1)
        # reset
        radians_prev = radians
        tiago.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

        #------------- Introduce ------------------#
        rospy.loginfo("Introducing myself")
        rospy.sleep(1)

		#----------- Wait for instruction ---------#
        rospy.loginfo("Waiting for instruction")
        rospy.sleep(1)

        #----------Move to inventory table---------#
        rospy.loginfo("Moving to table right corner")
        # publish
        radians = (math.pi - radians_prev) + (-0.75) #radian_offset=-0.75rad
        distance = (1.75 - 0.58) + (-0.7) #distance_offset=-0.7m
        tiago.move_base(radians, 2)
        rospy.sleep(1)
        tiago.move_base(distance, 0)
        rospy.sleep(1)
        # reset
        radians_prev = radians; distance_prev = distance
        tiago.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
