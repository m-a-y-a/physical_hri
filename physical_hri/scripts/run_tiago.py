#! /usr/bin/env python
import rospy
import math
import sys, moveit_commander
from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
from control_msgs.msg import PointHeadActionGoal
import numpy as np

class run_tiago:

    def __init__(self):

        self.cnt = 0
        self.gripper_pub = rospy.Publisher("/parallel_gripper_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.arm_pub = rospy.Publisher("/arm_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.head_pub = rospy.Publisher('/head_controller/point_head_action/goal', PointHeadActionGoal, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pose_pub = rospy.Publisher("/cart_pose", Pose2D, queue_size=5)
        self.way_pub = rospy.Publisher("/cart_goal_position", Pose2D, queue_size=5, latch=True)
        self.pose_conv_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.mode = 1
        self.mode_saved = False
        self.rate = rospy.Rate(1)

        self.group_arm_torso = moveit_commander.MoveGroupCommander("arm_right_torso")
        self.torso_arm_joint_goal = self.group_arm_torso.get_current_joint_values()

        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5) #rad/s
        self.distance_prev = 0
        self.radians_prev = 0


    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0: # do nothing
                pass
            elif self.mode == 1: # straight done, turn
                rospy.loginfo("Mode is %s" % str(self.mode))

                #Move base 
                rospy.loginfo("Facing user")
                radians = math.asin(1.25/(1.75-0.275)) + (-0.45) #radian_offset=-0.45rad
                self.move_base(radians, 2)
                rospy.sleep(1)
                self.radians_prev = radians
                self.base_pub.publish(Twist())
                rospy.sleep(1)

                #Move arm
                rospy.loginfo("Lifting arm")
                self.lift_arm()
                rospy.sleep(2)

                #Gripper to intial position
                rospy.loginfo("Init gripper")
                self.move_gripper(0.09)
                rospy.sleep(2)

                #Close gripper
                rospy.loginfo("Close gripper")
                self.move_gripper(-0.01)
                rospy.sleep(2)

                #Open gripper
                rospy.loginfo("Opening gripper")
                self.move_gripper(0.09)

                self.mode = 0
                self.mode_saved = False

            self.rate.sleep()
    
    def move_base(self, distance, direction):
        """
        this function sends velocity commands to the TIAGo mobile base controller
        :distance: meters or radians to move
        :direction: 0 for x-direction, 1 for y-direction, 2 for z-direction/turning
        """
        rate = 3
        r = rospy.Rate(rate)

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
        for i in range(0, int(time) * rate):
            rospy.logdebug("%s message, Sending...", 'pub_msg')
            self.base_pub.publish(pub_msg)
            r.sleep()
        rospy.loginfo("Published base command")


    def move_gripper(self, pos):
        #gripper is -0.09 to close and 0.09 to open
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["parallel_gripper_joint"]

        p = JointTrajectoryPoint()
        p.positions = [pos] 
        p.time_from_start = rospy.rostime.Duration(1)
        joint_msg.points.append(p)
        self.gripper_pub.publish(joint_msg)
        rospy.loginfo("Published grip command")

    def lift_arm(self):
        moveit_commander.roscpp_initialize(sys.argv)

        target = [0.15, 1.5, 0.58, 0.06, 1.0, -1.70, 0.0, 0.0]

        #Pass values for joint angles...and move the robot
        joint_goal = self.group_arm_torso.get_current_joint_values()
        print(joint_goal)
        for i in range(len(target)):
            rospy.loginfo("\t" + str(i) + " goal position: " + str(target[i]))
            joint_goal[i] = target[i]
        
        self.group_arm_torso.go(joint_goal, wait=True)
        self.group_arm_torso.stop()	

        moveit_commander.roscpp_shutdown()
    
def main():
    rospy.init_node('tiago_server')
    rospy.loginfo("Initialize node and server")

    tiago = run_tiago()
    rospy.loginfo("Node and server initialized")
    tiago.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    

    

