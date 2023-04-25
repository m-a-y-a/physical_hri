import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pal_common_msgs.msg import DisableActionGoal

class run_tiago: 

    def __init__(self, mode=5):
        self.rate = rospy.Rate(1)
        self.mode = mode
        # self.img_raw_sub = rospy.Subscriber('xstation/rgb/image_raw', Image, self.image_callback)
        # self.cam_intrinsic_sub = rospy.Subscriber('xstation/rgb/camera_info/camera_intrinsic', CameraInfo, image_callback)

        '''
        TODO:
        ok so currently this line is getting an error, invalid characters and requires an argument queue or something
            - he suggested we try debugging this with the simulation and can also continue working on this tomorrow
        ''' 
        s
        self.alive_pub = rospy.Publisher('pal_head_manager/disable/goal ', DisableActionGoal)

        # self.bridge = CvBridge()

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0: # do nothing
                pass
            elif self.mode == 5: #camera test
                self.move_head_to_position(0.03, -0.47)
                print("")
                # points_3d = self.image_listener()
                print('Detected ', len(points_3d), ' ArUco markers')

                #return to doing nothing
                self.mode = 0

            self.rate.sleep()

    # def image_listener(self):
    #     # Subscribe to the image topic
    #     rospy.Subscriber('xstation/rgb/image_raw', Image, self.image_callback)
    
    #     # Spin until shutdown
    #     rospy.spin()

    # def image_callback(self, data):
    #     # Convert image data to OpenCV format
    #     cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
    #     # Access the camera intrinsic parameters
    #     camera_info = data.camera_info
        
    #     # Construct the camera matrix from intrinsic parameters
    #     K = np.array(camera_info.K).reshape(3, 3)
        
    #     # Construct the distortion coefficients
    #     dist_coeffs = np.array(camera_info.D)
        
    #     # Define the ArUco dictionary and parameters
    #     aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    #     aruco_params = cv2.aruco.DetectorParameters_create()
        
    #     # Detect the markers in the image
    #     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)
        
    #     # Check if any markers were detected
    #     if ids is not None:
    #         # Initialize an empty list to store the 3D points
    #         points_3d = []
            
    #         # Loop over all detected markers
    #         for i in range(len(ids)):
    #             # Access the corners of the ith detected marker
    #             marker_corners = corners[i][0]
                
    #             # Compute the center of the marker
    #             marker_center = np.mean(marker_corners, axis=0)
                
    #             # Convert the center point to a normalized ray in camera coordinates
    #             cx = camera_info.width / 2.0
    #             cy = camera_info.height / 2.0
    #             fx = K[0, 0]
    #             fy = K[1, 1]
    #             x_c = (marker_center[0] - cx) / fx
    #             y_c = (marker_center[1] - cy) / fy
    #             z_c = 1.0
    #             x_c, y_c = cv2.undistortPoints(np.array([[x_c, y_c]]), K, dist_coeffs)[0]
                
    #             # Convert the point to 3D coordinates in the camera frame
    #             p_c = np.array([x_c, y_c, z_c])
                
    #             # Convert the point to 3D coordinates in the world frame
    #             R_cw = np.eye(3)  # Assume the camera is aligned with the world frame
    #             t_cw = np.zeros((3, 1))  # Assume the camera is at the origin
    #             p_w = np.dot(R_cw, p_c) + t_cw
                
    #             # Append the 3D point to the list of points
    #             points_3d.append(p_w)
            
    #         # Return the list of 3D points
    #         return points_3d
                
    #     else:
    #         # No markers detected
    #         sys.stdout.write('\rWaiting for ArUco marker...')
    #         sys.stdout.flush()

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
        
        # disable = DisableActionGoal()
        # disable.goal.duration = 10.0
        # pub.publish(disable)

if __name__ == '__main__':
    rospy.init_node('tiago_server')
    tiago = run_tiago(mode=5)
    tiago.run()    