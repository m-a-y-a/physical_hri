#! /usr/bin/env python
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
from pal_common_msgs.msg import DisableGoal, DisableAction
from std_msgs.msg import String
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import speech_recognition as sr

class run_tiago:

    def __init__(self, mode=1):

        self.cnt = 0
        self.gripper_pub = rospy.Publisher("/parallel_gripper_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.arm_pub = rospy.Publisher("/arm_right_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.head_pub = rospy.Publisher('/head_controller/point_head_action/goal', PointHeadActionGoal, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pose_pub = rospy.Publisher("/cart_pose", Pose2D, queue_size=5)
        self.way_pub = rospy.Publisher("/cart_goal_position", Pose2D, queue_size=5, latch=True)
        self.pose_conv_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.rate = rospy.Rate(1)

        # Camera info
        self.img_raw_sub = rospy.Subscriber('xstation/rgb/image_raw', Image, self.get_cv_image(data))
        self.cam_intrinsic_sub = rospy.Subscriber('xstation/rgb/camera_info/camera_intrinsic', CameraInfo, self.get_camera_info(data))
        self.bridge = CvBridge()

        # Client for preset motions
        self.ac = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion...")

        self.arm = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_right = SimpleActionClient('/arm_right_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso = SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head = SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Class variables
        self.tags = {"medicine bottle": 100, "water bottle": 200, "vitamin": 300, "mixed nuts": 400, "oats": 500, "table": 600}
        self.tags = {100: [0, 0, 0], 200: [0, 0, 0], 300: [0, 0, 0], 400: [0, 0, 0], 500: [0, 0, 0], 600: [0, 0, 0]}    
        self.camera_info = None
        self.dist_coeffs = None
        self.K = None
        self.cv_image = None
        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.3) #rad/s
        self.global_origin = np.array([0, 0, 0])
        self.current_state = np.array([-0.35, 0.25, 0])
        
        self.free_space = [1.25, 1.75]   # center of the "room"
        self.table_pos =  [1.925, 2.10]  # table to pick up objects
        self.aruco_pos = [1.925, 2.60]   # place to check aruco
        self.drop_off_pos = [2.25, 1.15] # place to drop object

        self.right_arm_full_extension = [1.5, 0.46, 0.09, 0.39, -1.45, 0.03, -0.00]
        self.torso_height_table = 0.24
        self.torso_height_aruco = 0.30
        self.torso_height_dropoff_table = 0.05
        self.head_rot_table = [0.00, -0.50]

        # Speech recogniser
        self.listener = sr.Recognizer()
        
        self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/command', Empty)

    def run(self):
        # Enter the scene
        rospy.loginfo("Entering the Scene")
        
        # Move to center of the room
        self.move_to([self.free_space[0], 0, 0], 0)                         # move in robot x
        self.move_to([self.free_space[0], 0, -90], 2)                       # turn right
        self.move_to([self.free_space[0], self.free_space[1], -90], 0)        # move in robot y
        rospy.loginfo("Arrived at free space")
        
        # Move to get aruco measures
        self.move_to([self.free_space[0], self.free_space[1], 0], 2)    # turn right
        self.move_to([self.aruco_pos[0], self.free_space[1], 0], 0)     # move forward
        self.move_to([self.aruco_pos[0], self.free_space[1], -90], 2)   # turn to face table
        self.move_to([self.aruco_pos[0], self.aruco_pos[1], -90], 0)      # move to in front of table
        rospy.loginfo("Arrived at Table")
        
        # Take photo
        self.move_head_to_position(self.head_rot_table)
        self.move_torso(self.torso_height_aruco)
        self.tag_distances = get_aruco_distance()

        # Move back to center
        self.move_to([self.aruco_pos[0], self.aruco_pos[1], 180], 2)       # turn right
        self.move_to([self.free_space[0], self.aruco_pos[1], 180], 0)      # move in robot x
        self.move_to([self.free_space[0], self.aruco_pos[1], 90], 2)       # turn right
        self.move_to([self.free_space[0], self.free_space[1], 90], 2)      # move forwards
        self.move_to([self.free_space[0], self.free_space[1], 60], 2)      # turn to user
        rospy.loginfo("Arrived at free space")
        
        self.play_motion('home', block = True)
        
        # Call for speech recognition
        self.send_cmd()

        self.rate.sleep()
        
    def play_motion(self, motion_name, block=False):
        g = PlayMotionGoal()
        g.motion_name = motion_name
        self.ac.wait_for_server()

        if block:
            self.ac.send_goal_and_wait(g)
        else:
            self.ac.send_goal(g)
            self.ac.wait_for_result()
    
    def move_torso(self, height):
        self.torso.wait_for_server()

        jtp = JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration(2)
        jtp.positions.append(height)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        self.torso.send_goal(goal)
        self.torso.wait_for_result()
        
    def move_arm(self, pos):
        self.right_arm.wait_for_server()
        
        jtp = JointTrajectoryPoint()
        jtp.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        jtp.positions = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
        jtp.time_from_start = rospy.Duration(2.0)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)
        self.right_arm.send_goal(goal)
        self.right_arm.wait_for_result()
        
    def say(self, text):
        client = SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)
        return
        
    def do_cmd(msg):
        if (msg == "thank you"):
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 90], 0)       # move backwards in x direction to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 180], 2)      # turn to left
            self.move_to([self.free_space[0], self.free_space[1], 180], 0)        # move in y dir back to center
            self.move_to([self.free_space[0], self.free_space[1], 60], 2)         # face user
            rospy.loginfo("Arrived at free space")
            
            self.play_motion('home', block = True)
            
            # Reply
            self.say("You’re welcome. What else can I get you?")
            
        elif ("this is wrong"):
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 90], 0)       # move backwards in x direction to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 180], 2)      # turn to left
            self.move_to([self.free_space[0], self.free_space[1], 180], 0)        # move in y dir back to center
            self.move_to([self.free_space[0], self.free_space[1], 60], 2)         # face user
            rospy.loginfo("Arrived at free space")
            
            self.play_motion('home', block = True)
            
            # Reply
            self.say("I’m sorry for my mistake. Can I get you something else?")

        elif ("i am done"):
           rospy.loginfo("End of interaction")
           # Reply
           self.say("Okay, have a wonderful day!")
           self.play_motion('wave', block = True)
           
           # Send shutdown signal
           
        else:
            rospy.loginfo("Request item")
            # Reply
            self.say("Yes, I will get you the " + msg)
            
            # Move torso up
            self.move_torso(0.24)
            
            # Offer arm
            self.play_motion('offer_right', block = True)
            
            # Move arm
            self.move_arm([1.50, 0.46, 0.09, 0.39, -1.45, 0.03, 0.00])

            # Move to table
            self.move_to([self.free_space[0], self.free_space[1], 0], 2)     # turn right
            self.move_to([self.table_pos[0], self.free_space[1], 0], 0)      # move forward
            self.move_to([self.table_pos[0], self.free_space[1], -90], 2)    # turn to face table
            self.move_to([self.table_pos[0], self.table_pos[1], -90], 0)     # move to in front of table
            rospy.loginfo("Arrived at Table")
            
            # Call AruCo tag identifier
            item_id = self.tags(msg)
            table_pos = self.tag_distances(600)
            item_pos = self.tag_distances(item_id)
            
            x_diff = table_pos[0][0] - item_pos[0][0]
            y_diff = table_pos[0][1] - item_pos[0][1]
            
            self.move_to([self.table_pos[0], self.table_pos[1] + (x_diff - .0225), -90], 1) # move side to side
            self.move_to([self.table_pos[0] + (y_diff - .0225), self.table_pos[1], -90], 0) # move forward

            # Grab item
            self.grasp

            # Move back to table center
            self.move_to([self.table_pos[0] - (y_diff - .0225), self.table_pos[1], -90], 0) # move backwards
            self.move_to([self.table_pos[0], self.table_pos[1] - (x_diff - .0225), -90], 1) # move side to side

            # Bring item to user
            self.move_to([self.table_pos[0], self.table_pos[1], 0], 2)           # turn left
            self.move_to([self.drop_off_pos[0], self.table_pos[1], 0], 0)        # move left to drop off x
            self.move_to([self.drop_off_pos[0], self.table_pos[1], 90], 2)       # turn left
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 90], 0)    # move to side of drop off table
            rospy.loginfo("Arrived at drop off")
            
            # Lower torso
            self.move_torso(0.05)
            
            # Release item
            self.grasp
            
            # Reply
            self.say("Here you go")
    
    def move_to(self, desired_state, desired_dir):
        # Calculate amount to move in x, y, or radial directions
        # Send move command to move_base() function
        # Updates current state

        # Calculate desired movement in global coordinate frame
        desired_state = np.array(desired_state)
        x_d = desired_state[0] - self.current_state[0] 
        y_d = desired_state[1] - self.current_state[1]
        th_d = math.radians((desired_state[2] - self.current_state[2]))

        # transform to robot local frame
        if self.current_state[2] == 0:
            # global: +x, -x, +y, -y 
            # local: +x, -x, -y, +y
            y_d = -y_d
        elif self.current_state[2] == 180:
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

        self.current_state = desired_state
        #rospy.loginfo("Now at: [%s]" % ",".join(str(x) for x in self.current_state))

    
    def move_base(self, distance, direction):
        """
        This function sends velocity commands to the TIAGo mobile base controller
        :distance: meters or radians to move
        :direction: 0 for x-direction, 1 for y-direction, 2 for z-direction/turning
        """
        rate = 30
        r = rospy.Rate(rate)

        # Initializing variables
        rospy.logdebug("%s message, initializing...", 'Twist')
        pub_msg = Twist()

        # Calculating variables to reach goal
        if direction == 0:
            time = math.ceil(abs(distance)/self.linear_speed)
            pub_msg.linear.x = distance/time #slightly vary linear speed
        if direction == 1 :
            time = math.ceil(abs(distance)/self.linear_speed)
            pub_msg.linear.y = distance/time #slightly vary linear speed
        if direction == 2 :
            time = math.ceil(abs(distance)/self.angular_speed)
            pub_msg.angular.z = distance/time #slightly vary angular speed 

        # Publishing the goal according to the time duration of the client's request
        for i in range(0, int(time) * rate):
            rospy.logdebug("%s message, Sending...", 'pub_msg')
            self.base_pub.publish(pub_msg)
            r.sleep()
        rospy.loginfo("Published base command")

        rospy.sleep(3) #pause for 3 second
        
    def get_voice_cmd(self):
        try:
            rospy.loginfo("in get_voice_cmd")
            self.say("I am listening")

            with sr.Microphone() as src:
                self.listener.adjust_for_ambient_noise(src)
                
                rospy.loginfo("trying to listen")
                audio = self.listener.listen(src, timeout=5)
                rospy.loginfo("trying google recognition")
                cmd = self.listener.recognize_google(audio, language = "en-EN")
                cmd = cmd.lower()   
                print(cmd)
                return cmd
        except Exception as e:
            rospy.logerr("Exception %s occurred", str(e))
            
    def send_cmd(self):
        cmd = ""
        while end == False:
            cmd = self.get_voice_cmd(self)
            rospy.logdebug("Voice detected %s", cmd)
            
            # Perform action based on word
            if (cmd == "hello"):
                play_motion('wave', block = True)
                self.say("Hi, my name is Tiago. What’s yours?")
            elif(cmd == "good"):
                self.say("That’s great! How can I help you today?")
            elif (cmd == "bad"):
                self.say("I’m sorry to hear that. How can I help you today?")
            elif (len(cmd) >= 10):
                if (cmd[0:9] == "my name is"):
                    #Split command to get name
                    name = cmd.split()[3]
                    self.say("It’s nice to meet you " + name + ". How has your day been?")
                elif (len(cmd) >= 18):
                    if (cmd[0:17] == "can you get me the"):
                        #Split command to get item
                        cmd_list = cmd.split()
                        if "medicine" in cmd_list:
                            item = "medicine bottle"
                            self.do_cmd(item)
                        if "water" in cmd_list:
                            item = "water bottle"
                            self.do_cmd(item)
                        if "nuts" in cmd_list:
                            item = "mixed nuts"
                            self.do_cmd(item)
                        if "vitamin" in cmd_list:
                            item = "mixed vitamin"
                            self.do_cmd(item)
                        if "oats" in cmd_list:
                            item = "oats"
                            self.do_cmd(item)
                        else:
                            self.say("I'm sorry, I don't know that item.")
                            
            elif (cmd == "thank you"):
                self.do_cmd(cmd)
            elif (cmd == "this is wrong"):
                self.do_cmd(cmd)
            elif (cmd == "i am done"):
                self.do_cmd(cmd)
                end = True
            else:
                self.say("Sorry, I don't recognize that command.")
    
    def keep_head_still(self):
        head_mgr_client = rospy.Publisher('/pal_head_manager/disable', DisableAction)
        motion = DisableGoal()
        rospy.loginfo("disabling head manager")
        motion.duration = 5.0
        head_mgr_client.publish(action)
        
        rospy.sleep(5)

    def move_head_to_position(self, pos):
        # Create a publisher to send joint trajectory commands
        self.head. wait_for_server()

        # Create a JointTrajectory message
        jtp = JointTrajectoryPoint()
        jtp.positions = [pos[0], pos[1]]
        jtp.time_from_start = rospy.Duration(2.0)

        # Set the joint names for the trajectory
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        self.head.send_goal(goal)
        self.head.wait_for_result()

        # Disable head movement
        self.keep_head_still()

    def get_cv_image(self, data):
        # Convert image data to OpenCV format
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        print(self.cv_image)
        
    def get_camera_info(self, data):
        # Access the camera intrinsic parameters
        self.camera_info = data.camera_info
        
        # Construct the camera matrix from intrinsic parameters
        self.K = np.array(self.camera_info.K).reshape(3, 3)
        
        # Construct the distortion coefficients
        self.dist_coeffs = np.array(self.camera_info.D)
        
    def get_aruco_distance(self, item):
        # Define the ArUco dictionary and parameters
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        aruco_params = cv2.aruco.DetectorParameters_create()
        marker_size = 0.08  # marker is 8 cm
        
        # Detect the markers in the image
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=aruco_params)
        rospy.loginfo("ids found {0}".format(ids))
        
        # Check if any markers were detected
        if ids is not None:
            # Loop over all detected markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.K, self.dist_coeffs)
            for i in range(len(ids)):
                self.tag_distances(ids[i]) = tvecs[i]      
                
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
            