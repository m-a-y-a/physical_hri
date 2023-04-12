#! /usr/bin/env python3

# Libraries
import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from speech_rec.msg import MoveSpeechAction, MoveSpeechGoal , MoveSpeechActionFeedback

# Linear velocity value to be added or subtracted from the current speed value.
vel_adder=0.20

# Standard linear velocity value
vel=0.5

class MoveSpeechClient:

    def __init__(self,name):

        self._action_name = name
        rospy.loginfo("%s service, initializing server", 'move_spc')
        self.client = actionlib.SimpleActionClient(self._action_name, MoveSpeechAction)

        # boolean variable to determine if the current action goal has to be cancelled.
        self.stop_motion=False

        # Set of boolean variables to check if a certain command cannot be executed for safety reasons.
        self.can_go_left = True
        self.can_go_right = True
        self.can_go_straight = True

        self.prev_can_go_left= self.can_go_left
        self.prev_can_go_right= self.can_go_right
        self.prev_can_go_straight= self.can_go_straight

        # Goal message to be sent to the server of the custom action "MoveSpeechAction"
        self.goal = MoveSpeechGoal()

    def callback_feedback(self,feedback):
        '''
        Callback retrieving the feedback values form the server.
        This function will stop any goal running if the stop_motion variable turns True.

        Arguments:
        * feedback (int): int value retrieved from the server.

        Returns:
        * None 

        '''
        if self.stop_motion:
            print("STOP")

            # Canceling any goal running
            self.client.cancel_all_goals()
            rospy.loginfo("Goals Cancelled")
            # Resetting the variable to False
            self.stop_motion=False

    def callback_laser(self,laser):
        '''
        This callback function will be the subscriber function to the /scan_raw topic.
        This sub will retrieve the LaserScan message comming from the laser array of the TiaGo robot. 
        From the message contained in the laser argument, the function wil extract the distance information about the surroundings of the robot.
        If the robot will get too close to a certain object in space, the function will stop the robot from moving by cancelling all the curent active goals.

        Arguments: 
         * laser(LaserScan): laser scan message containing all the information abour intensity 
        range and other features of the TiaGo's laser-array. 

        Returns:
         * None 
        '''


        #retrieving the minimum frontal and lateral distances, if the distance is greater or equal to 1 meter, then that distance will be set to 1
        right = min(min(laser.ranges[40:122]), 1)
        front = min(min(laser.ranges[263:504]), 1)
        left = min(min(laser.ranges[545:622]), 1)

        #if the value is not '1', it means that it is lower and therefore the robot should not be able to go towards that direction
        if right != 1:
            self.can_go_right = False

            # If the previous value of the bool variable is different than the current one, 
            # the robot will need to stop its movement by cancelling the goal.
            # The same concept applies for every other cases where a certain 
            # array section reaches the 1 meter distance limit.

            if(self.can_go_right != self.prev_can_go_right):
                self.client.cancel_all_goals()
                rospy.loginfo("MOTION RIGHT STOPPED: Goal cancelled due to obstacle")
            

        else:
            self.can_go_right =True

        if left != 1:
            self.can_go_left =False
            if(self.can_go_left != self.prev_can_go_left):
                self.client.cancel_all_goals()
                rospy.loginfo("MOTION LEFT STOPPED: goal cancelled due to obstacle")
            
        else:
            self.can_go_left =True

        if front != 1:
            self.can_go_straight = False
            if(self.can_go_straight != self.prev_can_go_straight):
                self.client.cancel_all_goals()
                rospy.loginfo("MOTION STRAIGHT STOPPED: goal cancelled due to obstacle")
            
        else:
            self.can_go_straight =True

        # Assigning the current values of the bool variables to the previous ones.
        self.prev_can_go_right=self.can_go_right
        self.prev_can_go_left=self.can_go_left
        self.prev_can_go_straight=self.can_go_straight

    def motion_callback(self,msg):
        '''
        This callback function will subscribe to the /moving custom topic which will
        retrive from the UI.py script the int32 value related to the required goal. 
        This value will be sent to the server of the custom action to make the robot move towards 
        a certain direction the TiaGo robot.

        Arguments: 
         * msg (int32): int value stored in the data field related to the certain goal asked to be achived.

        Returns:
         * None 

        '''
        global vel
        global vel_adder

        # Waiting for server to be ready
        rospy.logdebug("Waiting for client")
        self.client.wait_for_server()

        # If the message retrived is equal to a certain number identified in this state 
        # machine, the goal relted to that certain number will be sent.

        if msg.data == 1 and self.can_go_straight:

            self.goal.velocity = vel
            self.goal.turn = 0
            self.goal.time = 10
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Straight goal sent: go forward for %f s, velocity: %f m/s " % (self.goal.time, self.goal.velocity))

        if msg.data == 9 :
            print("GO BACKWARDS")
            self.goal.velocity = -vel
            self.goal.turn = 0
            self.goal.time = 10
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Backward goal sent: go backward for %f s, velocity: %f m/s " % (self.goal.time, self.goal.velocity))

        if msg.data == 2 and self.can_go_left:
            print("TURN LEFT")
            self.goal.velocity = 0
            self.goal.turn = 0.785
            self.goal.time = 2
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Left goal sent: turn left for %f s, angular velocity: %f rad/s " % (self.goal.time, self.goal.turn))

        if msg.data == 3 and self.can_go_right:
            print("TURN RIGHT")
            self.goal.velocity = 0
            self.goal.turn = -0.785
            self.goal.time = 2
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Right goal sent: turn right for %f s, angular velocity: %f rad/s " % (self.goal.time, self.goal.turn))

        if msg.data == 4 and self.can_go_right and self.can_go_straight:
            print("GO STRAIGHT-RIGHT")
            self.goal.velocity = vel
            self.goal.turn = -0.785
            self.goal.time = 2
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Straight-Right goal sent: go forward and turn Right for %f s, velocity: %f m/s, angular velocity: %f rad/s " % (self.goal.time, self.goal.velocity, self.goal.turn))

        if msg.data == 5 and self.can_go_left and self.can_go_straight:
            print("GO STRAIGHT-LEFT")
            self.goal.velocity = vel
            self.goal.turn = +0.785
            self.goal.time = 2
            self.client.send_goal(self.goal,None,None,self.callback_feedback)
            rospy.loginfo("Straight-Left goal sent: go forward and turn Left for %f s, velocity: %f m/s, angular velocity: %f rad/s " % (self.goal.time, self.goal.velocity,self.goal.turn))

        # The following numbers are related to the encreasing decreasing and resetting
        # of the robot's linear velocity.
        if msg.data == 6:
            vel=vel+vel_adder
            rospy.loginfo("Increased velocity of %f, current velocity: %f" % (vel_adder,vel))
        if msg.data == 7:
            if (vel-vel_adder)>0:
                vel=vel-vel_adder
                rospy.loginfo("Decreased velocity of %f, current velocity: %f" % (vel_adder,vel))
            else:
                print("Cannot decrease more!")
        if msg.data == 8:
            vel=1.0
            rospy.loginfo("Reset velocity, current: %f" % (vel))

        # If the stop command is called this if statement will stop any active motion
        # by cancelling the goal.
        if msg.data == -1:
            self.stop_motion=True
            rospy.loginfo("Motion stopped by the user!")


if __name__ == '__main__':

    # Init of the node
    rospy.init_node('move_speech_client', log_level=rospy.DEBUG)
    rospy.loginfo("Node %s initialized", 'move_speech_client')

    # Definition of the client
    rospy.logdebug("%s service, initializing server", 'move_spc')
    client = MoveSpeechClient('move_spc')

    # Subscription to the /moving topic with the motion_callback function
    rospy.logdebug("Subscription to the %s custom topic with the motion_callback function", 'moving')
    moving_sub = rospy.Subscriber('/moving', Int32, client.motion_callback)

    # Subscription to the /scan_raw topic with the callback_laser
    rospy.logdebug("Subscription to the %s topic with the callback_laser function", 'scan_raw')
    scan_raw_sub = rospy.Subscriber('/scan_raw', LaserScan, client.callback_laser)
    rospy.spin()
