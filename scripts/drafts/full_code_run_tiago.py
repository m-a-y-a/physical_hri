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
from std_msgs.msg import String
from std_srvs.srv import Empty
from cv_bridge import CvBridge

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
        
        # Subscribe to topic
        # Camera info
        self.img_raw_sub = rospy.Subscriber('xstation/rgb/image_raw', Image, self.get_aruco)
        self.cam_intrinsic_sub = rospy.Subscriber('xstation/rgb/camera_info/camera_intrinsic', CameraInfo, self.get_aruco)
        self.bridge = CvBridge()

        # Speech recogniser
        self.listener = sr.Recognizer()
        
        # Client for preset motions
        self.ac = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion...")
        
        self.arm = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso = SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.camera_info = None
        self.dist_coeffs = None
        self.K = None
        self.cv_image = None
        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.3) #rad/s
        self.global_origin = np.array([0, 0, 0])
        self.current_state = np.array([0, 0, 0])
        self.free_space = [1.25, 1.75]      # center of the "room"
        self.table_pos =  [1.925, 2.30]     # table to pick up objects
        self.drop_off_pos = [2.00, 0.90]    # drop off table

        self.right_arm_full_extension = [1.5, 0.46, 0.09, 0.39, -1.45, 0.03, -0.00]
        self.right_arm_front_items = [1.46, 1.02, 0.09, 1.32, -1.48, -0.34, 0.00]
        self.torso_height_table = 0.25
        self.torso_height_dropoff_table = 0.05
        self.head_rot_table = [0.03, -0.47]
        
        self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/command', Empty)
        
    def run(self):
        # Enter the scene
        rospy.loginfo("Entering the Scene")

        # Move to center of the room
        self.move_to([self.free_space[0], 0, 0], 0)                         # move in robot x
        self.move_to([self.free_space[0], self.free_space[1], -90], 2)      # turn right
        self.move_to([self.free_space[0], self.free_space[1], 0], 1)        # move in robot y
        self.move_to([self.free_space[0], self.free_space[1], 150], 2)      # turn to user
        rospy.loginfo("Arrived at free space")
        
        # Call for speech recognition
        self.send_cmd()

        self.rate.sleep()
        
    def play_motion(self, motion_name, block=True):
        goal = PlayMotionGoal()
        goal.motion_name = motion_name

        if block:
            self.ac.send_goal_and_wait(goal)
        else:
            self.ac.send_goal(goal)
            
    def move_gripper(self, pos):
        # Gripper is -0.09 to close and 0.09 to open
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["parallel_gripper_joint"]

        jtp = JointTrajectoryPoint()
        jtp.positions = [pos] 
        jtp.time_from_start = rospy.rostime.Duration(1)
        joint_msg.points.append(jtp)
        self.gripper_pub.publish(joint_msg)
        rospy.loginfo("Published grip command")
    
    def move_torso(self, height):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [height]
        jtp.velocities = [0.1]
        jtp.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(jtp)
        self.torso.send_goal(goal)
        self.torso.wait_for_result()
        
    def move_arm(self, pos):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
        jtp.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        jtp.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(jtp)
        self.arm.send_goal(goal)
        self.arm.wait_for_result()
        
    def say(self, text):
        client = SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)
        
    def do_cmd(msg):
        if (msg == "thank you"):
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 90], 2)  # turn to left
            self.move_to([self.free_space[0], self.drop_off_pos[1], 90], 0)    # move in x direction to center
            self.move_to([self.free_space[0], self.drop_off_pos[1], 180], 2)   # turn to left
            self.move_to([self.free_space[0], self.free_space[1], 270], 0)     # move in y dir back to center
            self.move_to([self.free_space[0], self.free_space[1], 360], 2)     # face user
            rospy.loginfo("Arrived at free space")
            
            # Reply
            self.say("You’re welcome. What else can I get you?")
            
        elif ("this is wrong"):
            # Reset: move back to center
            self.move_to([self.free_space[0], self.drop_off_pos[1], 0], 0)   # move backwards in x direction to center
            self.move_to([self.free_space[0], self.drop_off_pos[1], 90], 2)  # turn to left
            self.move_to([self.free_space[0], self.free_space[1], 0], 0)     # move in y dir back to center
            self.move_to([self.free_space[0], self.free_space[1], -130], 2)  # face user
            rospy.loginfo("Arrived at free space")
            
            # Reply
            self.say("I’m sorry for my mistake. Can I get you something else?")

        elif ("i am done"):
           rospy.loginfo("End of interaction")
           # Reply
           self.say("Okay, have a wonderful day!")
           
           # Send shutdown signal
           
        else:
            rospy.loginfo("Request item")
            # Reply
            self.say("Yes, I will get you the " + msg)
            
            # Move torso up
            self.move_torso(torso_height_table) # add here later
            
            # Offer arm
            self.play_motion('offer_right', block = True)
            
            # Move arm
            self.move_arm([])

            # Move to table
            self.move_to([self.free_space[0], self.free_space[1], 90], 2)   # turn to right
            self.move_to([self.table_pos[0], self.free_space[1], 0], 0)     # move forward
            self.move_to([self.table_pos[0], self.free_space[1], -90], 2)   # turn to face table
            
            self.move_to([self.table_pos[0], self.table_pos[1], -180], 0)   # move to in front of table
            rospy.loginfo("Arrived at table")
            
            # Call AruCo tag identifier
            x_diff, y_diff = self.get_aruco_distance(msg)
            self.move_to([self.table_pos[0], self.table_pos[1] + x_diff], 1) # move side to side
            self.move_to([self.table_pos[0] + y_diff, self.table_pos[1]], 0) # move forward

            # Grab item
            self.move_gripper(-.044)

            # Move back to table center
            self.move_to([self.table_pos[0], self.table_pos[1] - x_diff], 1) # move side to side
            self.move_to([self.table_pos[0] - y_diff, self.table_pos[1]], 0) # move backwards

            # Bring item to user
            self.move_to([self.table_pos[0], self.table_pos[1], -90], 2)       # turn left
            self.move_to([self.table_pos[0], self.drop_off_pos[1], 0], 0)      # move left to drop off x
            self.move_to([self.table_pos[0], self.drop_off_pos[1], 90], 2)     # turn left
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 0], 0)   # move to side of drop off table
            rospy.loginfo("Arrived at drop off")
            
            # Lower torso
            self.move_torso(torso_height_dropoff_table) # add here later
            
            # Release item
            self.move_gripper(.09)
            
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
    
    def move_gripper(self, pos):
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["parallel_gripper_joint"]

        p = JointTrajectoryPoint()
        p.positions = [pos] 
        p.time_from_start = rospy.rostime.Duration(1)
        joint_msg.points.append(p)
        self.gripper_pub.publish(joint_msg)
        rospy.loginfo("Published grip command")
        
    def get_voice_cmd(self):
        try:
            with sr.Microphone as src:
                self.listener.adjust_for_ambient_noise(src)
                audio = self.listener.listen(src, timeout = 1.5)
                cmd = self.listener.recognize_google(audio, language = "en-EN")
                cmd = cmd.lower()
                
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
                        item = cmd.split()[5]
                        self.do_cmd(item)
            elif (cmd == "thank you"):
                self.do_cmd(cmd)
            elif (cmd == "this is wrong"):
                self.do_cmd(cmd)
            elif (cmd == "i am done"):
                sel.do_cmd(cmd)
                end = True
            else:
                self.say("Sorry, I don't recognize that command.")
    
    def get_aruco(self, data):
        # Convert image data to OpenCV format
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        # Access the camera intrinsic parameters
        self.camera_info = data.camera_info
        
        # Construct the camera matrix from intrinsic parameters
        self.K = np.array(camera_info.K).reshape(3, 3)
        
        # Construct the distortion coefficients
        self.dist_coeffs = np.array(camera_info.D)
        
    def get_aruco_distance(self, item):
        # Define the ArUco dictionary and parameters
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Detect the markers in the image
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=aruco_params)
        
        # Check if any markers were detected
        if ids is not None:
            table_p_c = np.array([0, 0, 0])
            # Loop over all detected markers
            for i in range(len(ids)):
                if (i == ):
                    # Access the corners of the ith detected marker
                    marker_corners = corners[i][0]
                    
                    # Compute the center of the marker
                    marker_center = np.mean(marker_corners, axis=0)
                    
                    # Convert the center point to a normalized ray in camera coordinates
                    cx = camera_info.width / 2.0
                    cy = camera_info.height / 2.0
                    fx = K[0, 0]
                    fy = K[1, 1]
                    x_c = (marker_center[0] - cx) / fx
                    y_c = (marker_center[1] - cy) / fy
                    z_c = 1.0
                    x_c, y_c = cv2.undistortPoints(np.array([[x_c, y_c]]), K, dist_coeffs)[0]
                    
                    # Convert the point to 3D coordinates in the camera frame
                    table_p_c = np.array([x_c, y_c, z_c])
                    
                if (i == item):
                    # Calculate distance
                    # Access the corners of the ith detected marker
                    marker_corners = corners[i][0]
                    
                    # Compute the center of the marker
                    marker_center = np.mean(marker_corners, axis=0)
                    
                    # Convert the center point to a normalized ray in camera coordinates
                    cx = camera_info.width / 2.0
                    cy = camera_info.height / 2.0
                    fx = K[0, 0]
                    fy = K[1, 1]
                    x_c = (marker_center[0] - cx) / fx
                    y_c = (marker_center[1] - cy) / fy
                    z_c = 1.0
                    x_c, y_c = cv2.undistortPoints(np.array([[x_c, y_c]]), K, dist_coeffs)[0]
                    
                    # Convert the point to 3D coordinates in the camera frame
                    p_c = np.array([x_c, y_c, z_c])
                    
                    # Calculate distance between table center and object
                    x_diff = table_p_c[0] - p_c[0]
                    y_diff = table_p_c[1] - p_c[1]
                    return (x_diff, y_diff)
                    
                else:
                    # No object found
                    # Reply
                    self.say("Sorry, I don't recognize that object")
                    return (None, None)
                
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
            