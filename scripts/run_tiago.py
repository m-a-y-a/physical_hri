#! /usr/bin/env python
""" Tiago Physical HRI Code File

This file handles all parts of the physical HRI task. It creates a class called run_tiago that intiates all ROS publishers, subscribers, 
services, action clients, and global variables such as locations used by the code. It then runs the initial base movement
code to get TIAGo in front of the inventory table to get the AruCo tags. Once these tag positions are retrieved, the TIAGo base is 
moved to the center of the room to begin the interaction with the user. Google speech recognition API detects whichever command the user 
gives. The user may choose to engage in a short conversation with the robot (using the PAL Text-to-Speech) and introduce themselves or they may ask for an item from the 
table. If an item is requested, TIAGo's base is moved in front of the inventory table and its torso is raised while its arm is extended.
The base is then moved to align the hand with the requested object's AruCo marker position and the grasp service proxy is used to grab 
the item. The base is then moved to in front of the dropoff table location, the torso is lowered to the table height, and the grasp service
proxy opens the hand to release the item. The user may then either thank the robot or tell it that the wrong item was retrieved, to which 
TIAGo will either express "You're welcome" or remorse for making a mistake. The TIAGo base is then moved back to the center of the space 
to await further requests of interactions.

Usage:
    1. Import the necessary modules:
        import rospy
        import math
        import sys
        import cv2
        from std_msgs.msg import Int16
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadActionGoal
        from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
        from sensor_msgs.msg import Image, CameraInfo
        import numpy as np
        from actionlib import SimpleActionClient
        from pal_interaction_msgs.msg import TtsAction, TtsGoal
        from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
        from pal_common_msgs.msg import DisableGoal, DisableActionGoal
        from std_msgs.msg import String
        from std_srvs.srv import Empty
        from cv_bridge import CvBridge
        import speech_recognition as sr
        from aruco_msgs.msg import MarkerArray
    2. Run the code in the terminal using the command:
        python run_tiago.py 
        
"""
import rospy
import math
import sys
import cv2
from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadActionGoal
from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from pal_common_msgs.msg import DisableGoal, DisableActionGoal
from std_msgs.msg import String
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import speech_recognition as sr
from aruco_msgs.msg import MarkerArray

class run_tiago:
    """Class initializer for run_tiago class instance
    
    Initializes the publishers, subscribers, action clients, service proxies, speech recognizer, and global variables 
    used within an instance of the run_tiago class.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
    
    Returns:
        run_tiago: returns an instance of the run_tiago class with all of its class variables set from the initializer
    
    """
    def __init__(self):
        # Initialize publishers used by ROS
        self.gripper_pub = rospy.Publisher("/parallel_gripper_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.arm_pub = rospy.Publisher("/arm_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.head_pub = rospy.Publisher('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pose_pub = rospy.Publisher("/cart_pose", Pose2D, queue_size=5)
        self.way_pub = rospy.Publisher("/cart_goal_position", Pose2D, queue_size=5, latch=True)
        self.pose_conv_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.rate = rospy.Rate(1)

        # Initialize client for playing preset motions
        self.ac = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion...")

        # Initialize clients for moving specific TIAGo joints
        self.arm = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_right = SimpleActionClient('/arm_right_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso = SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head = SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Class variables   
        self.aruco_dictionary = {"water bottle": 8, "medicine bottle": 12, "oats": 11, "mixed nuts": 10, "mixed vitamin": 9}
        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2)     #speed in m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.3)   #speed in rad/s
        self.global_origin = np.array([0, 0, 0])        #Origin of the global coordinates of the 'room'
        self.current_state = np.array([-0.35, 0, 0])    #Starting position of TIAGo (set to -.35 X since TIAGo starts outside the bounds of the 'room'
        self.markerPoses = None     #array of poses of AruCo markers relative to TIAGo's base
        
        # Preset room locations in global coordinate system of 2.5 m x 3.5 m room
        self.aruco_pos = [1.80, 1.45]      #location to go to to check aruco
        self.free_space = [1.25, 1.45]     #center of the "room"
        self.drop_off_pos = [1.80, 1.15]   #place to drop off object

        # Preset joint values
        self.right_arm_full_extension = [1.5, 0.46, 0.09, 0.39, -1.45, 0.03, -0.00]
        self.tuck_arm = [-1.10, 1.47, 2.71, 1.71, -1.57, 1.37, -2.24]
        self.torso_height_table = 0.24
        self.torso_height_aruco = 0.30
        self.torso_height_dropoff_table = 0.05
        self.torso_height_home = 0.14
        self.head_rot_table = [0.00, -0.50]
        # Preset TIAGo robot measurements
        self.body_to_midline = 0.225    #distance from 'shoulder' to center of 'body'
        self.center_to_palm = 0.7       #distance from center of 'body' to center of 'palm'

        # Speech recogniser
        self.listener = sr.Recognizer()

    """Performs initial TIAGo movements
    
    Moves TIAGo to the location to view the AruCo tags, moves the head joint into position for viewing the tags, and 
    stores their positions using the marker pose node. Moves to the center of the room and turns to the approximate 
    location of the user, then calls the function to start waiting for speech detection/commands.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
    
    Returns:
        None
    
    """
    def run(self):
        
        # Enter the scene
        rospy.loginfo("Entering the Scene")

        # Move to center of the room 
        self.move_to([self.free_space[0], 0, 0], 0)                         # move forward
        self.move_to([self.free_space[0], 0, -90], 2)                       # turn right
        self.move_to([self.free_space[0], self.aruco_pos[1], -90], 0)       # move forward
        rospy.loginfo("Arrived at free space")

        # Move to get aruco measures
        self.move_to([self.aruco_pos[0], self.aruco_pos[1], -90], 1)        # move sideways to table
        rospy.loginfo("Arrived at Table")

        rospy.loginfo("Please start the pose subscriber")
        # Move torso and head into position to view markers
        self.move_torso(self.torso_height_aruco)
        self.move_head_to_position(self.head_rot_table)
        rospy.sleep(7)

        # Get positions of aruco markers from camera feed
        rospy.loginfo("Trying to read markers")
        msg = rospy.wait_for_message('aruco_marker_publisher/markers', MarkerArray)
        self.save_pose(msg)
        rospy.loginfo("End the pose subscriber")
        rospy.sleep(12)
            
        # Move back to center of room
        self.move_to([self.aruco_pos[0], self.aruco_pos[1], -180], 2)       # turn right
        self.move_to([self.free_space[0], self.aruco_pos[1], 180], 0)       # move forward
        self.move_to([self.free_space[0], self.aruco_pos[1], 60], 2)        # turn to user
        rospy.loginfo("Arrived at free space")
    
        # Move torso back into lower position for movements
        self.move_torso(self.torso_height_home)
        
        # Call for speech recognition
        self.send_cmd()
        self.rate.sleep()
    
    """Plays preset motions of TIAGo
    
    Takes the name of a preset motion that TIAGo can do and plays it using the motion goal simple action client.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        motion_name: the string name of motion that TIAGo is preset with
        block: boolean of whether or not the played action should block other actions from being done when playing; preset to false
        
    Returns:
        None
    
    """
    def play_motion(self, motion_name, block=False):
        # Create a play motion goal for the preset motion and wait for the action client to respond
        goal = PlayMotionGoal()
        goal.motion_name = motion_name
        self.ac.wait_for_server()

        # Send the goal to the action client and wait
        if block:
            self.ac.send_goal_and_wait(goal)
        else:
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            
    """Moves TIAGo's torso
    
    Moves TIAGo's torso to a specific height using a simple action client that takes a follow trajectory point.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        height: the specific height (as a float value) to move the torso joint to (joint name is 'torso_lift_joint')
    
    Returns:
        None
    
    """
    def move_torso(self, height):
        # Wait for the simple action client to respond before sending goal
        self.torso.wait_for_server()

        # Declare joint trajectory point for the specific torso height
        jtp = JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration(2)
        jtp.positions.append(height)

        # Declare follow joint trajectory goal for the torso joint and it's desired trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        # Send the goal to action client and wait for response
        self.torso.send_goal(goal)
        self.torso.wait_for_result()
        
    """Moves TIAGo's right arm
    
    Moves TIAGo's right arm joints to a specific positions using a simple action client that takes a follow trajectory point.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        pos: the array of floats for each of the joint positions of TIAGo's right arm 
            (joint names are ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'] for array)
    
    Returns:
        None
    
    """
    def move_arm(self, pos):
        # Wait for the simple action client to respond before sending goal
        self.arm_right.wait_for_server()
        
        # Declare joint trajectory points for the specific arm positions and the speed at which to move them
        jtp = JointTrajectoryPoint()
        jtp.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        jtp.positions = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
        jtp.time_from_start = rospy.Duration(2.0)
        
        # Declare follow joint trajectory goal for the arm joints and their desired trajectories
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)
        
        # Send the goal to action client and wait for response
        self.arm_right.send_goal(goal)
        self.arm_right.wait_for_result()
    
    """Say text using PAL's TTS
    
    Uses simple action client for PAL's Text-to-Speech to have TIAGo say a line of text.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        text: the string that TIAGo should say using TTS
    
    Returns:
        None
    
    """
    def say(self, text):
        # Create the simple action client for TTS
        client = SimpleActionClient('/tts', TtsAction)
        # Wait for the client to respond before sending goal
        client.wait_for_server()
        
        # Declare the TTS goal, the text for TTS to say, and the language the text is in/voice to use for TTS
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"  #en-GB is British English voice/language for TTS
        
        # Send goal to client and wait
        client.send_goal_and_wait(goal)
        return
        
    """Perform movements and actions based on command given
    
    Called when speech recognition gets a command. If the command is "thank you" or "this is wrong", TIAGo moves back to the
    center of the room and says a response using TTS. If the command is "i am done", TIAGo waves and says goodbye, indicating 
    the interactions are over. If the command is none of these, the command is the name of an object to be retrieved. In this 
    case, TIAGo moves in front of the inventory table, extends its right arm to a safe distance and height, then moves so its 
    palm is at the location of the AruCo marker for the detected object. It then grasps the object, moves to the dropoff table,
    lowers to a safe height, and releases the item for the user.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        msg: the string of the command received from speech recognition
        
    Returns:
        None
    
    """
    def do_cmd(self, msg):
        if (msg == "thank you"):
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 180], 2)      # turn left
            self.move_to([self.free_space[0], self.drop_off_pos[1], 180], 0)        # move forward
            self.current_state[2] = -180                                            # change sign convention (-180==180 deg)
            self.move_to([self.free_space[0], self.drop_off_pos[1], -90], 2)        # turn
            self.move_to([self.free_space[0], self.aruco_pos[1], -90], 0)           # move forward
            self.move_to([self.free_space[0], self.aruco_pos[1], 60], 2)            # turn to user
            rospy.loginfo("Arrived at free space")
            
            # Reply
            self.say("You’re welcome. What else can I get you?")
            
        elif (msg == "this is wrong"):
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 180], 2)      # turn left
            self.move_to([self.free_space[0], self.drop_off_pos[1], 180], 0)        # move forward
            self.current_state[2] = -180                                            # change sign convention (-180==180 deg)
            self.move_to([self.free_space[0], self.drop_off_pos[1], -90], 2)        # turn
            self.move_to([self.free_space[0], self.aruco_pos[1], -90], 0)           # move forward
            self.move_to([self.free_space[0], self.aruco_pos[1], 60], 2)            # turn to user
            rospy.loginfo("Arrived at free space")
            
            # Reply
            self.say("I’m sorry for my mistake. Can I get you something else?")

        elif (msg == "i am done"):
           rospy.loginfo("End of interaction")
           # Reply
           self.say("Okay, have a wonderful day!")
           # Wave goodbye
           self.play_motion('wave', block = True)
           
        else:
            # Command was the name of an item requested
            rospy.loginfo("Trying to look for item {0}".format(msg))
            try:
                # Get AruCo ID for object requested's name
                item_id = self.aruco_dictionary[msg]
                # Get object's AruCo pose from stored poses
                pose = self.get_marker_pose(item_id)
                rospy.loginfo("Got the marker pose for {0}".format(msg))

                # Calculate the position of the AruCo marker in the global coordinates (the pose is in relation to TIAGo's base initially, so add TIAGo's global position to the marker pose)
                global_x = pose.position.x + self.aruco_pos[1]
                global_y = pose.position.y + self.aruco_pos[0]

                # Reply
                self.say("Yes, I will get you the " + msg)

                # Calculate the position TIAGo should move to to grab the object at that AruCo marker (by adjusting for the width and arm length of the robot since the marker is measured from the robot base's center)
                local_y = global_x - self.center_to_palm     # used as y value for robot
                local_x = global_y + self.body_to_midline    # used as x value for robot

                rospy.loginfo("Trying to move back to the table")
                
                # Move to table
                self.move_to([self.free_space[0], self.aruco_pos[1], 0], 2)      # turn right
                self.move_to([self.aruco_pos[0], self.aruco_pos[1], 0], 0)       # move forward
                self.move_to([self.aruco_pos[0], self.aruco_pos[1], -90], 2)     # turn to face table
                rospy.loginfo("Arrived at Table")

                # Move back a little bit for the arm to prevent it from hitting other objects on the table
                offset = 0.35  #move 35 cm back for arm 
                self.move_to([self.aruco_pos[0], self.aruco_pos[1] - offset, -90], 0) # move backwards

                # Offer arm to preset position for easier arm adjustments
                self.move_torso(self.torso_height_table)
                self.play_motion('offer_right', block = True)

                # Move torso up
                self.move_torso(self.torso_height_table)
                        
                # Move arm into exact position from preset position 'offer_right'
                self.move_arm(self.right_arm_full_extension)

                # Move to position calculated for grabbing the object at the AruCo marker
                self.move_to([local_x, self.aruco_pos[1] - offset, -90], 1) # moving to adjusted position calculation side-to-side
                self.move_to([local_x, local_y, -90], 0)                    # moving to adjusted position calculation forwards
                rospy.loginfo("Moved sideways by %s, Moved forward by %s", local_x, local_y)

                # Grab item
                # Wait for grasping service to respond before sending grasp request
                rospy.wait_for_service('/parallel_gripper_right_controller/grasp')
                self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/grasp', Empty)
                rospy.sleep(2)
                result = self.grasp()

                # Move backwards from table to prevent arm from hitting other objects on table
                self.move_to([local_x, self.aruco_pos[1] - offset, -90], 0)                   # move backwards
                self.move_to([self.aruco_pos[0], self.aruco_pos[1] - offset , -90], 1)        # move sideways
                   
                # Move to dropoff table to bring item to user
                self.move_to([self.aruco_pos[0], self.aruco_pos[1] - offset, 90], 2)           # turn 180 degrees
                self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 90], 0)              # move forward
                self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 60], 2)              # turn to drop off table

                rospy.loginfo("Arrived at drop off")
                        
                # Lower torso to dropoff table height to drop item
                self.move_torso(self.torso_height_dropoff_table)
                        
                # Release item
                # Wait for grasping service to respond before sending command
                rospy.wait_for_service('/parallel_gripper_right_controller/grasp')
                self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/grasp', Empty)
                rospy.sleep(2)
                result = self.grasp()

                # Raise torso to normal height 
                self.move_torso(self.torso_height_table)
                
                # Tuck arm back in to prevent collisions
                self.move_arm(self.tuck_arm)
                
                # Reply
                self.say("Here you go")

                # Turn to face 90 degrees again to get into position for next movements
                self.move_torso(self.torso_height_home)                                 # raise torso to normal height
                self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 90], 2)       # turn to front

            except Exception as e:
                print(e)
    
    """Calculates distances and rotations to move
    
    Takes an x, y location in the global coordinate system that the robot should move to or a degrees (180, 90, 0, -90 being the four sides of the project space)
    that the robot sould turn towards. Calculates the distance to move/radians to turn by subtracting the robot's current location from the 
    location it has to move to.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        desired_state: the array of two floats and one integer (x float, y float, degrees integer) to move the robot to
        desired_dir: integer value for if the robot should move in its local x direction (0), its y direction (1), or turn (2)
        
    Returns:
        None
    
    """
    def move_to(self, desired_state, desired_dir):
        # Calculate desired movement in global coordinate frame by getting change in position/angle
        desired_state = np.array(desired_state)
        x_d = desired_state[0] - self.current_state[0] 
        y_d = desired_state[1] - self.current_state[1]
        th_d = math.radians((desired_state[2] - self.current_state[2]))

        # Transform to robot local frame
        # When robot turns, its local frame may not match the global one so check the angle the robot faces to account for this when choosing what distance to move
        if self.current_state[2] == 0:
            # global: +x, -x, +y, -y 
            # local: +x, -x, -y, +y
            y_d = -y_d
        elif abs(self.current_state[2]) == 180:
            # global: +x, -x, +y, -y 
            # local: -x, +x, +y, -y 
            x_d = -x_d 
        elif self.current_state[2] < 0 : #CW
            # global: +x, -x, +y, -y 
            # local: +y, -y, +x, -x
            y_d, x_d = x_d, y_d 
        elif self.current_state[2] > 0: #CCW
            # global: +x, -x, +y, -y 
            # local: -y, +y, -x, +x
            y_d, x_d = x_d, y_d 
            x_d = -x_d
            y_d = -y_d

        # Execute movement of base
        if desired_dir != None:
            if desired_dir == 0:
                self.move_base(x_d, 0) #move in x
                #rospy.loginfo("Moved in local x by %s" % str(x_d))
            elif desired_dir == 1:
                self.move_base(y_d, 1) #move in y
                #rospy.loginfo("Moved in local y by %s" % str(y_d))
            elif desired_dir == 2:
                self.move_base(th_d, 2) #turn in theta
                #rospy.loginfo("Turned by %s" % str(math.degrees(th_d)))

        # Update current robot location to match where it just moved to
        self.current_state = desired_state
        #rospy.loginfo("Now at: [%s]" % ",".join(str(x) for x in self.current_state))
    
    """Send movement command to TIAGo's base
    
    Takes the distance to move or radians to turn, calculates the seconds it would take to move that distance given the preset speed
    of 0.2 m/s or 0.4 rad/s and adjusts the speed using the rounded time since the base controller only takes integer time values.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        distance: the distance float value for either the x or y direction or the radians float value to turn
        direction: the integer value for what direction the robot should move the distance- x direction (0), y direction (1), turn (2)
        
    Returns:
        None
    
    """
    def move_base(self, distance, direction):
        # Set the publishing rate so the movement commands are published fast enough that TIAGo moves smoothly
        rate = 35
        r = rospy.Rate(rate)

        # Initializing variables
        rospy.logdebug("%s message, initializing...", 'Twist')
        pub_msg = Twist()

        # Calculating variables to reach goal
        ''' 
        Base controller takes time to move and speed to move at
        Get the time to move given the preset movement speeds and round it to a whole number (base controller only accepts integer time values)
        Round up the time to prevent dividing by 0 errors
        Divide distance by the rounded time to get the adjusted speed to move at for the rounded time value
        '''
        if direction == 0:
            time = math.ceil(abs(distance)/self.linear_speed)
            if time == 0:
                time = 1
            pub_msg.linear.x = distance/time #slightly vary linear speed
        if direction == 1 :
            time = math.ceil(abs(distance)/self.linear_speed)
            if time == 0:
                time = 1 
            pub_msg.linear.y = distance/time #slightly vary linear speed
        if direction == 2 :
            time = math.ceil(abs(distance)/self.angular_speed)
            if time == 0:
                time = 1
            pub_msg.angular.z = distance/time #slightly vary angular speed 

        # Publishing the goal according to the time duration of the client's request
        for i in range(0, int(time) * rate):
            rospy.logdebug("%s message, Sending...", 'pub_msg')
            self.base_pub.publish(pub_msg)
            r.sleep()
        rospy.loginfo("Published base command")

        rospy.sleep(3) #pause for 3 seconds to make sure the published command has time to process before sending the next one
        
    """Use Google Speech API
    
    Uses the run_tiago class instance's Google API recognizer to get sounds picked up by the microphone, adjusts for any ambient noise
    in the room, and returns the text string detected by the speech recognizer from the audio
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        
    Returns:
        cmd: the string of the speech detected from the audio feed
    
    """
    def get_voice_cmd(self):
        try:
            rospy.loginfo("In get_voice_cmd")
            self.say("Hi, my name is Tiago. What can I get for you?")

            # Use the system's microphone for the audio feed
            with sr.Microphone() as src:
                # Remove ambient noise from the audio feed to get just speech
                self.listener.adjust_for_ambient_noise(src)
                
                rospy.loginfo("Trying to listen")
                # Detect speech from audio
                audio = self.listener.listen(src, timeout=5)
                
                rospy.loginfo("Trying google recognition")
                # Convert audio detected to text given the language is set to English ('en-EN')
                cmd = self.listener.recognize_google(audio, language = "en-EN")
                # Convert the text to lowercase to prevent having to check for proper capitalization
                cmd = cmd.lower()   
                print("cmd is: {0}".format(cmd))
                return cmd
        except Exception as e:
            rospy.logerr("Exception %s occurred", str(e))
    
    """Check which speech command was detected
    
    Starts a loop that keeps running the speech recognizer function. If speech is detected, it checks if the string matches one of the preset
    commands coded for and if it does, it calls the proper function (do_cmd) to handle the robot's actions in response to the given command. 
    If the command does not match a preset command, the robot replies that it does not recognize the command using TTS.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        
    Returns:
        None
    
    """
    def send_cmd(self):
        # Declare the command variable
        cmd = ""
        end = False     #boolean to check if loop should be stopped
        '''
        Loop keeps calling the speech recognizer to keep checking if the user said a command.
        If the user says the command "I am done", the loop exits to indicate that the interaction with the robot is
        over and no more commands can be sent.
        '''
        while end == False:
            rospy.loginfo("Social interaction will begin shortly")
            # Get the voice command from the audio feed
            cmd = self.get_voice_cmd()
            try:
                #Split command to get each word
                cmd_list = cmd.split()
                # Perform action based on preset command
                if (cmd == "hello"):
                    self.play_motion('wave', block = True)
                    self.say("Hi, my name is Tiago. What’s yours?")
                elif(cmd == "good"):
                    self.say("That’s great! How can I help you today?")
                elif (cmd == "bad"):
                    self.say("I’m sorry to hear that. How can I help you today?")
                elif (len(cmd) >= 10):
                    # Checks if a long command is given (either an object request, long response, or the user giving their name
                    if (cmd[0:9] == "my name is"):
                        if (len(cmd_list) >= 3):
                            name = cmd_list[3]
                            self.say("It’s nice to meet you " + name + ". How has your day been?")
                    elif (cmd == "this is wrong"):
                        # Calls do_cmd to return to center of room
                        self.do_cmd(cmd)
                    else:
                        # Checks if the name of object was requested and calls do_cmd to get that object 
                        if "medicine" in cmd_list:
                            item = "medicine bottle"
                            self.do_cmd(item)
                        elif "water" in cmd_list:
                            item = "water bottle"
                            self.do_cmd(item)
                        elif "nuts" in cmd_list:
                            item = "mixed nuts"
                            self.do_cmd(item)
                        elif "vitamin" in cmd_list:
                            item = "mixed vitamin"
                            self.do_cmd(item)
                        elif "oats" in cmd_list:
                            item = "oats"
                            self.do_cmd(item)
                        else:
                            # In case user requests an invalid item name
                            self.say("I'm sorry, I don't know that item.")
                elif (cmd == "thank you"):
                    # Call do_cmd to move the robot back to the center of the room
                    self.do_cmd(cmd)
                elif (cmd == "i am done"):
                    # Ends the loop and plays TIAGo end message/actions in do_cmd
                    self.do_cmd(cmd)
                    end = True
                else:
                    # In case command given isn't recognized as a coded for response
                    self.say("Sorry, I don't recognize that command. Please try again.")
            except Exception as e:
                rospy.logerr("Exception %s occurred", str(e))
                # Continue if error occurs so commands can still be received
                continue
    
    """Stops TIAGo's constant head motion
    
    Starts the head manager publisher to disable the constant head motion that TIAGo does in order to get a static image for AruCo detection
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        
    Returns:
        None
    
    """
    def keep_head_still(self):
        # Create publisher to head manager and sleep before sending to it to make sure its fully set up
        head_mgr_client = rospy.Publisher('/pal_head_manager/disable/goal', DisableActionGoal, queue_size=1)
        rospy.sleep(3)
        
        # Create the goal to disable head motion for 60.0 seconds so AruCo detection has time to work
        motion = DisableActionGoal()
        rospy.loginfo("Disabling head manager")
        motion.goal.duration = 60.0
        head_mgr_client.publish(motion)

    """Move head into position for AruCo detection
    
    Uses a simple action client with a follow joint trajectory goal to move the head down so it can see all of the markers for 
    AruCo marker detection.
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        pos: the array of float positions to move the head joints to (the joint names are ['head_1_joint', 'head_2_joint'])
        
    Returns:
        None
    
    """
    def move_head_to_position(self, pos):
        # Disable head movement
        self.keep_head_still()

        # Create a publisher to send joint trajectory commands
        self.head.wait_for_server()
        rospy.sleep(1)

        # Create a JointTrajectory message for the head joint names and positions to move to
        jointTraj = JointTrajectory()
        jointTraj.joint_names = ['head_1_joint', 'head_2_joint']
        jointTraj.points.append(JointTrajectoryPoint())
        jointTraj.points[0].positions = [pos[0], pos[1]]
        jointTraj.points[0].time_from_start = rospy.Duration(3)

        # Set the joint names for the trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = jointTraj
        goal.goal_time_tolerance = rospy.Duration(0)

        # Send the goal to the action client and wait for a response that it worked
        self.head.send_goal(goal)
        self.head.wait_for_result()
        rospy.loginfo("Moved head to position")

    """Store the poses of detected AruCo markers
    
    Take the marker pose array returned by the AruCo detection node and store it as a class variable for access in 
    other code areas
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        array: the array of poses returned by the AruCo detection node (consists of x, y, and z positions relative to TIAGo's base)
        
    Returns:
        None
    
    """
    def save_pose(self, array):
        self.markerPoses = array.markers

    """Gets specific AruCo marker ID's pose 
    
    Returns the x, y, and z position of the specific AruCo marker (the marker with the given ID) relative to TIAGo's base
    
    Args:
        self: the instance of the object (does not need to be listed as an argument when declaring instance of run_tiago)
        id: the integer ID of the AruCo marker to get the pose for
        
    Returns:
        marker.pose.pose: the x, y, z positions in a pose object for the AruCo marker with the given ID (if the marker was found via detection)
        None: no object is returned if the marker ID was not found during AruCo detection
    
    """
    def get_marker_pose(self, id):
        # Loop over the array of markers returned during AruCo detection until either all markers are checked or a marker with an ID matching the input is found
        for marker in self.markerPoses:
            if marker.id == id:
                rospy.loginfo("Found aruco marker id number {0}".format(id))
                print(marker.pose)
                return marker.pose.pose
            else:
                continue
        return None 

"""Main Function
    
    Initiatializes a TIAGo server node so the code can send to publisher and receive from subscriber nodes, then creates an
    instance of the run_tiago class to instantiate all class variables. Once instantiated, it then calls the run function of 
    the run_tiago class instance to begin the robot retrieval process.
    
    Args:
        None
        
    Returns:
        None
    
    """                
def main():
    # Start ROS node to communicate with other nodes
    rospy.init_node('tiago_server')
    rospy.loginfo("Initialize node and server")

    # Instantiate class and begin retrieval code
    tiago = run_tiago()
    rospy.loginfo("Node and server initialized")
    tiago.run()

"""Main Function Called by Python
    
    When python code is run in the terminal, this function is automatically called first (it is the python 'main' function).
    Tries to call the main function to begin the object retrieval process or throws an exception is an error occurs during 
    code execution/node setup.
    
    Args:
        None
        
    Returns:
        None
    
    """
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass