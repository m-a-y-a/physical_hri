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
        self.mode = mode
        self.mode_saved = False
        self.rate = rospy.Rate(1)

        # Camera info
        self.img_raw_sub = rospy.Subscriber('xstation/rgb/image_raw', Image, self.get_aruco)
        self.cam_intrinsic_sub = rospy.Subscriber('xstation/rgb/camera_info/camera_intrinsic', CameraInfo, self.get_aruco)
        self.bridge = CvBridge()

        # Client for preset motions
        self.ac = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion...")

        self.arm = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_right = SimpleActionClient('/arm_right_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso = SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Class variables
        self.camera_info = None
        self.dist_coeffs = None
        self.K = None
        self.cv_image = None
        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.3) #rad/s
        self.global_origin = np.array([0, 0, 0])
        self.current_state = np.array([-0.35, 0.25, 0])
        
        self.free_space = [1.25, 1.75] # center of the "room"
        self.table_pos =  [1.925, 2.00] # table to pick up objects
        self.aruco_pos = [1.925, 2.60] # place to check aruco
        self.drop_off_pos = [2.25, 0.90]

        self.right_arm_full_extension = [1.5, 0.46, 0.09, 0.39, -1.45, 0.03, -0.00]
        self.torso_height_table = 0.25
        self.torso_height_dropoff_table = 0.05
        self.head_rot_table = [0.03, -0.47]

        # Speech recogniser
        self.listener = sr.Recognizer()
        
        self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/command', Empty)


    def run(self):
        # while not rospy.is_shutdown():
        if self.mode == 0: # do nothing
            pass
        elif self.mode == 1: #gripper and arm test
            rospy.loginfo("Mode is %s" % str(self.mode))

            # Lift arm
            rospy.loginfo("Lifting arm")
            self.play_motion('offer_right', block=True)
            rospy.sleep(2)

            # Move torso up to inventory table
            self.move_torso(self.torso_height_table)

            # maybe move forward here

            # ----------- Pick up object at table ------------
            # Look down
            self.move_head_to_position(self.head_rot_table)
            # Move arm
            self.move_arm(self.right_arm_full_extension)

            # Grasp
            self.grasp()
            # ------------------------------------------------

            # Move torso down to little table
            self.move_torso(self.torso_height_dropoff_table)

            self.mode = 0
            self.mode_saved = False
        elif self.mode == 2: #aruco test
            rospy.loginfo("Mode is %s" % str(self.mode))
            
            # Move to center of the room
            self.move_to([self.free_space[0], 0, 0], 0)                         # move in robot x
            self.move_to([self.free_space[0], 0, -90], 2)                       # turn right
            self.move_to([self.free_space[0], self.free_space[1], 0], 0)        # move in robot y
            self.move_to([self.free_space[0], self.free_space[1], 60], 2)       # turn to user
            rospy.loginfo("Arrived at free space")
            
            # Move close to table
            self.move_to([self.free_space[0], self.free_space[1], 0], 2)    # turn right
            self.move_to([self.aruco_pos[0], self.free_space[1], 0], 0)     # move forward
            self.move_to([self.aruco_pos[0], self.free_space[1], -90], 2)   # turn to face table
            self.move_to([self.aruco_pos[0], self.aruco_pos[1], 0], 0)      # move to in front of table
            rospy.loginfo("Arrived at Table")
            
            x_diff, y_diff = self.get_aruco_distance(100)
            rospy.loginfo("Dist x is %s" % str(x_diff))
            rospy.loginfo("Dist y is %s" % str(y_diff))
            
        elif self.mode == 3: #speech recognition test
            rospy.loginfo("Mode is %s" % str(self.mode))
            
            end = False

            while (end != True):
                word = self.send_cmd()
                if (word == "stop"):
                    end = True

            self.mode = 0
            self.mode_saved = False

        elif self.mode == 5:

            # -------------- works: -------------------

            rospy.loginfo("offering right")
            self.play_motion('offer_right')

            rospy.loginfo("moving torso")
            self.move_torso(self.torso_height_table)

            # -------------- end of works -------------------

            rospy.loginfo("moving arm right extend")
            self.move_joint(self.right_arm_full_extension)
            self.mode = 0
            self.mode_saved = False

        elif self.mode == 6:
            rospy.loginfo("Mode is %s" % str(self.mode))
            self.test_major_moves()

    def test_major_moves(self):
        # Enter the scene
        rospy.loginfo("Entering the Scene")

        # Move to center of the room
        self.move_to([self.free_space[0], 0, 0], 0)                         # move in robot x
        self.move_to([self.free_space[0], 0, -90], 2)                       # turn right
        self.move_to([self.free_space[0], self.free_space[1], -90], 0)      # move in robot y
        self.move_to([self.free_space[0], self.free_space[1], 60], 2)       # turn to user
        rospy.loginfo("Arrived at free space")

        # For every object on the inventory table:
        n = 2 # (2 times to test)
        for i in range(1, n+1):
            # Request item
            rospy.loginfo("Request item")

            # Move to table
            self.move_to([self.free_space[0], self.free_space[1], 60], 2)    # turn right
            self.move_to([self.table_pos[0], self.free_space[1], 60], 0)     # move forward
            self.move_to([self.table_pos[0], self.free_space[1], -90], 2)    # turn to face table
            self.move_to([self.table_pos[0], self.table_pos[1], -90], 0)     # move to in front of table
            rospy.loginfo("Arrived at Table")

            # Perform Pick and Place

            # Bring item to user
            self.move_to([self.table_pos[0], self.table_pos[1], 0], 2)           # turn left
            self.move_to([self.drop_off_pos[0], self.table_pos[1], 0], 0)        # move left to drop off x
            self.move_to([self.drop_off_pos[0], self.table_pos[1], 90], 2)       # turn left
            self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 90], 0)    # move to side of drop off table
            rospy.loginfo("Arrived at drop off")
    
            # Reset: move back to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 90], 0)       # move backwards in x direction to center
            self.move_to([self.drop_off_pos[0], self.free_space[1], 180], 2)      # turn to left
            self.move_to([self.free_space[0], self.free_space[1], 180], 0)        # move in y dir back to center
            self.move_to([self.free_space[0], self.free_space[1], 60], 2)         # face user
            rospy.loginfo("Arrived at free space")
        
        # End sequence
        rospy.loginfo("End of interaction")

        self.mode = 0
        self.mode_saved = False

    def move_to(self, desired_state, desired_dir):
        # calculate amount to move in x, y, or radial directions
        # send move command to move_base() function
        # updates current state

        # calculate desired movement in global coordinate frame
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

        # execute movement of base
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
        this function sends velocity commands to the TIAGo mobile base controller
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
            if time == 0:
                time = 1.0
            pub_msg.linear.x = distance/time #slightly vary linear speed #error here after arrive at free space
        if direction == 1 :
            time = math.ceil(abs(distance)/self.linear_speed)
            pub_msg.linear.y = distance/time #slightly vary linear speed
        if direction == 2 :
            time = math.ceil(abs(distance)/self.angular_speed)
            pub_msg.angular.z = distance/time #slightly vary angular speed 

        # Publishing the goal according to the time duration of the 
        # client's request
        for i in range(0, int(time) * rate):
            rospy.logdebug("%s message, Sending...", 'pub_msg')
            self.base_pub.publish(pub_msg)
            r.sleep()
        rospy.loginfo("Published base command")

        rospy.sleep(3) #pause for 3 second


    def move_gripper(self, pos):
        #gripper is -0.09 to close and 0.09 to open
        self.move_joint(["parallel_gripper_joint"], [pos])
        # joint_msg = JointTrajectory()
        # joint_msg.joint_names = ["parallel_gripper_joint"]

        # p = JointTrajectoryPoint()
        # p.positions = [pos] 
        # p.time_from_start = rospy.rostime.Duration(1)
        # joint_msg.points.append(p)
        # self.gripper_pub.publish(joint_msg)
        # rospy.loginfo("Published grip command")

    def say(self, text):
        client = SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)
        return
    
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
                return cmd
        except Exception as e:
            rospy.logerr("Exception %s occurred", str(e))

    def send_cmd(self):
        rospy.loginfo("in send_cmd")
        cmd = self.get_voice_cmd()
        rospy.logdebug("Voice detected %s", cmd)
        if (cmd == "hello"):
            self.say(cmd)
        elif(cmd == "good job"):
            self.say("thanks")
        elif (cmd == "thank you"):
            self.say("you are welcome")

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

        # Callbacks are not handled right now
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

    def keep_head_still(self):
        '''
        node = '/pal_head_manager' 
        is_running = rosnode.rosnode_ping(node, max_count = 10)
        # print("Is the node " + node_name + " running?" + str(is_running))
        if is_running:
        # if the node is running, shut it down
            rosnode.kill_nodes([node_name])
        '''
        head_mgr_client = SimpleActionClient('/pal_head_manager/disable', DisableAction)
        action = DisableGoal()
        rospy.loginfo("disabling head manager")
        action.duration = 0.0
        head_mgr_client.send_goal(action)


    def move_head_to_position(self, pos):
        head_1 = pos[0]
        head_2 = pos[1]

        # Create a publisher to send joint trajectory commands
        pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        pub_head_manager = rospy.Publisher('/pal_head_manager')

        # Create a JointTrajectory message
        traj_msg = JointTrajectory()

        # Set the joint names for the trajectory
        joint_names = ['head_1_joint', 'head_2_joint']
        traj_msg.joint_names = joint_names

        # Create a trajectory point with the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = [head_1, head_2]
        point.time_from_start = rospy.Duration(1.0)  # Move to the desired position in 1 second

        # Add the trajectory point to the trajectory message
        traj_msg.points.append(point)

        # Publish the trajectory message
        pub.wait_for_server(timeout=rospy.Duration(5))
        pub.publish(traj_msg)

        # disable head movement
        self.keep_head_still()

    def get_aruco(self, data):
        # Convert image data to OpenCV format
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        print(self.cv_image)
        
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
        marker_size = 0.05  # marker is 5 cm
        
        # Detect the markers in the image
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=aruco_params)
        rospy.loginfo("ids found {0}".format(ids))
        
        # Check if any markers were detected
        if ids is not None:
            table_pos = np.array([0, 0, 0])
            # Loop over all detected markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.K, self.dist_coeffs)
            for i in range(len(ids)):
                if (ids[i] == 600):
                    table_pos = tvecs[i]
                if (ids[i] == item):
                    item_pos = tvecs[i]
                    if (len(table_pos) != 0):
                        # Calculate distance between table center and object
                        x_diff = table_pos[0][0] - item_pos[0][0]
                        y_diff = table_pos[0][1] - item_pos[0][1]
                        return x_diff, y_diff
                    
                else:
                    # No object found
                    # Reply
                    self.say("Sorry, I don't recognize that object")
                    return (None, None)
    
def main():
    rospy.init_node('tiago_server')
    rospy.loginfo("Initialize node and server")

    tiago = run_tiago(mode=6)
    # self.test_major_moves()
    # self.test_arm_movement()

    rospy.loginfo("Node and server initialized")
    tiago.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass