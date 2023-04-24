#! /usr/bin/env python
import rospy
import math
import sys
# moveit_commander
from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
from control_msgs.msg import PointHeadActionGoal
import numpy as np
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import speech_recognition as sr
from std_msgs.msg import String
from std_srvs.srv import Empty

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

        self.ac = SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion...")

        self.arm = SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso = SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # self.group_arm_torso = moveit_commander.MoveGroupCommander("arm_right_torso")
        # self.torso_arm_joint_goal = self.group_arm_torso.get_current_joint_values()

        self.linear_speed = rospy.get_param("~start_linear_speed", 0.2) #m/s
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.3) #rad/s
        self.global_origin = np.array([0, 0, 0])
        self.current_state = np.array([0, 0, 0])
        # total (2.5, 3.5) to the table
        # self.interact_pos = [1.925, 1.75] # inwards 1.925 m and upwards 1.75
        self.free_space = [1.25, 1.75] # center of the "room"
        self.table_pos =  [1.925, 2] # table to pick up objects
        self.drop_off_pos = [2.0, 0.3] # drop off table

        self.right_arm_full_extension = [1.5, 0.46, 0.09, 0.39, -1.45, 0.03, -0.00]
        self.torso_height_table = 0.25
        self.torso_height_dropoff_table = 0.05
        self.head_rot_table = [0.03, -0.47]

        self.listener = sr.Recognizer()
        
        self.grasp = rospy.ServiceProxy('/parallel_gripper_right_controller/command', Empty)


    def run(self):
        # while not rospy.is_shutdown():
        if self.mode == 0: # do nothing
            pass
        elif self.mode == 1: # major moves test
            
            rospy.loginfo("Mode is %s" % str(self.mode))

            # rate should be higher moves really patchy
            # Enter the scene
            rospy.loginfo("Entering the Scene")

            # center of the room
            self.move_to([self.free_space[0], 0, 0], 0)                         # move in robot x
            self.move_to([self.free_space[0], self.free_space[1], 0], 1)        # move in robot y
            self.move_to([self.free_space[0], self.free_space[1], 68], 2)       # turn to user
            rospy.loginfo("Arrived at free space")
            #self.say("Start of interaction")                #say out loud

            # For every object on the inventory table:
            n = 2 # (2 times to test)
            for i in range(1, n+1):
                # Request item
                rospy.loginfo("Request item")
                #self.say("Ask user for item")               #say out loud

                # Move to table
                self.move_to([self.free_space[0], self.free_space[1], 0], 2)  # turn to tv
                self.move_to([self.table_pos[0], self.free_space[1], 0], 0) # move forward
                self.move_to([self.table_pos[0], self.free_space[1], -90], 2) # turn to face table
                self.move_to([self.table_pos[0], self.table_pos[1], -90], 0) # move to in front of table
                rospy.loginfo("Arrived at Table")
                #self.say("Pick up item")                    #say out loud

                # Perform Pick and Place

                # Bring item to user
                self.move_to([self.table_pos[0], self.free_space[1], -90], 0) # back to center
                self.move_to([self.table_pos[0], self.free_space[1], 90], 2) # turn to front of room
                self.move_to([self.table_pos[0], self.drop_off_pos[1], 90], 0) # move to y = 0.25
                self.move_to([self.table_pos[0], self.drop_off_pos[1], 0], 2) # turn to drop off table
                self.move_to([self.drop_off_pos[0], self.drop_off_pos[1], 0], 0) # move to x = 2.15 in front of drop off table
                # self.move_to([self.interact_pos[0], self.interact_pos[1], 30], 2)   # turn
                rospy.loginfo("Arrived at drop off")
                #self.say("Hand item to user")               #say out loud

                # reset: move back to center
                self.move_to([self.free_space[0], self.drop_off_pos[1], 0], 0) # move backwards in x direction to center
                self.move_to([self.free_space[0], self.drop_off_pos[1], -90], 2) # turn to face table
                self.move_to([self.free_space[0], self.free_space[1], -90], 0) # move in y dir back to center
                self.move_to([self.free_space[0], self.free_space[1], 68], 2) # face user
                rospy.loginfo("Arrived at free space")


            
            # End sequence
            rospy.loginfo("End of interaction")
            #self.say("End of interaction")                  #say out loud

            self.mode = 0
            self.mode_saved = False

        elif self.mode == 2: #gripper and arm test
            rospy.loginfo("Mode is %s" % str(self.mode))

            # #Move arm
            # rospy.loginfo("Lifting arm")
            # self.lift_arm()
            # rospy.sleep(2)

            # #Grasp
            # self.grasp()

            '''
            #Gripper to intial position
            rospy.loginfo("Init gripper")
            self.move_gripper(0.09)
            rospy.sleep(1)

            #Close gripper
            rospy.loginfo("Close gripper halfway")
            self.move_gripper(0.044)
            rospy.sleep(1)

            #Open gripper
            rospy.loginfo("Open gripper")
            self.move_gripper(0.09) 
            '''
            # ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

            self.move_torso(0.25)
            self.move_arm

            self.mode = 0
            self.mode_saved = False

        elif self.mode == 3: #speech recognition test
            rospy.loginfo("Mode is %s" % str(self.mode))

            for i in range(0,1):
                rospy.sleep(3)
                # self.say("I am listening")
                self.send_cmd()

            self.mode = 0
            self.mode_saved = False

        elif self.mode == 4: #wave test
            rospy.loginfo("Mode is %s" % str(self.mode))

            self.play_motion('wave', block=True)

            self.mode = 0
            self.mode_saved = False


            self.rate.sleep()

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
            pub_msg.linear.x = distance/time #slightly vary linear speed
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
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["parallel_gripper_joint"]

        p = JointTrajectoryPoint()
        p.positions = [pos] 
        p.time_from_start = rospy.rostime.Duration(1)
        joint_msg.points.append(p)
        self.gripper_pub.publish(joint_msg)
        rospy.loginfo("Published grip command")

    # def lift_arm(self):
    #     moveit_commander.roscpp_initialize(sys.argv)

    #     target = [0.15, 1.5, 0.58, 0.06, 1.0, -1.70, 0.0, 0.0]

    #     #Pass values for joint angles...and move the robot
    #     joint_goal = self.group_arm_torso.get_current_joint_values()
    #     print(joint_goal)
    #     for i in range(len(target)):
    #         rospy.loginfo("\t" + str(i) + " goal position: " + str(target[i]))
    #         joint_goal[i] = target[i]
        
    #     self.group_arm_torso.go(joint_goal, wait=True)
    #     self.group_arm_torso.stop()   

    #     moveit_commander.roscpp_shutdown()

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
                # self.listener.pause_threshold=1.2
                self.listener.energy_threshold = 1932
                self.listener.dynamic_energy_threshold = True
                
                rospy.loginfo("trying to listen")
                # audio = self.listener.listen(src, timeout=10)
                audio = self.listener.listen(src)
                # audio = self.listener.record(source=mic, duration=10)
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
            self.say("")
        elif (cmd == "thank you"):
            self.say("you are welcome")

    def play_motion(self, motion_name, block=True):
        g = PlayMotionGoal()
        g.motion_name = motion_name

        if block:
            self.ac.send_goal_and_wait(g)
        else:
            self.ac.send_goal(g)

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

    def keep_head_still(self):
        node = '/pal_head_manager' 
        is_running = rosnode.rosnode_ping(node, max_count = 10)
        # print("Is the node " + node_name + " running?" + str(is_running))
        if is_running:
        # if the node is running, shut it down
            rosnode.kill_nodes([node_name])


    def move_head_to_position(self, head_1, head_2):
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
        pub.publish(traj_msg)

        # disable head movement
        keep_head_still()


    
def main():
    rospy.init_node('tiago_server')
    rospy.loginfo("Initialize node and server")

    tiago = run_tiago(mode=3)
    rospy.loginfo("Node and server initialized")
    tiago.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
