#! /usr/bin/env python3

########### Moving_Sever ##########

# Libraries
import roslib
roslib.load_manifest('speech_rec')
import rospy
import actionlib
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from speech_rec.msg import *


class MoveSpeechServer:

  """

  Class to generate a MoveSpeechServer object, which will handle the moving custom 
  action client's goal requests. It returns a feeedback about the goal status of moving the 
  robot's base by publishing velocity messages on '/mobile_base_controller/cmd_vel' topic.

  """

  # status feedback object
  _status_feedback = MoveSpeechActionFeedback() 
  # time duration feedback object
  _feedback = MoveSpeechFeedback() 

  # status result object
  _status_result = MoveSpeechActionResult() 
  #
  _result = MoveSpeechResult()

  def __init__(self,name):

    # Itiliazing the custom SimpleActionserver 

    self._action_name = name
    self.server = actionlib.SimpleActionServer(self._action_name, MoveSpeechAction, self.execute, False)
    self.server.start()

  def execute(self, goal):

    rate = 10
    r = rospy.Rate(rate)
    
    self._feedback.time = 0

    # Defining the velocity and rotation of the robot according to 
    # the desired goal

    rospy.logdebug("%s message, initializing...", 'Twist')
    pub_msg = Twist()
    pub_msg.linear.x = goal.velocity
    pub_msg.linear.y = 0
    pub_msg.linear.z = 0

    pub_msg.angular.x = 0
    pub_msg.angular.y = 0
    pub_msg.angular.z = goal.turn

    rospy.loginfo("Duration of the action: "+str(goal.time * rate))
    #print("Duration of the action: "+str(goal.time * rate))

    # publishing the goal according to the time duration of the 
    # client's request

    for i in range(0, goal.time * rate):

      # Managing the premption state of the goal
      if self.server.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self.server.set_preempted()
        success = False
        break

      # setting the time action feedback
      self._feedback.time = i
     
      # publishing the feedback
      self.server.publish_feedback(self._feedback)
      # publiching the goal 

      rospy.logdebug("%s message, Sending...", 'pub_msg')
      pub.publish(pub_msg)
      r.sleep()

    success = True
    self.server.set_succeeded()


if __name__ == '__main__':

  # Initializing the Node 
  rospy.init_node('moving_server',log_level=rospy.DEBUG)
  rospy.loginfo("Node %s initialized", 'moving_server')
  
  # Defining the publisher for the 'cmd_vel' like topic for TIAGo
  rospy.logdebug("Publishing to the %s custom topic with the motion_callback function", '/mobile_base_controller/cmd_vel')
  pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

  # initializing the custom server object
  rospy.logdebug("%s service, initializing server", 'move_spc')
  server = MoveSpeechServer('move_spc')
  