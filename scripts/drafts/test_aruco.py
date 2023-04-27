import cv2
import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import sys
import matplotlib.pyplot as plt


def get_aruco_distance():
    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()


    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(2.0)

    while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 1000 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=1000)

        cv2.imshow("Frame", frame)
        cv2.waitKey(0)

        # detect ArUco markers in the input frame
        (corners, ids, rejected_img_points) = cv2.aruco.detectMarkers(frame,
            aruco_dict, parameters=aruco_params)

        print("ids found: {0}".format(ids))

    # Detect the markers in the image
    # corners, ids, rejected_img_points = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=aruco_params)
        camera_info_width = 640
        camera_info_height = 480
        D = [0.011434452409443142, -0.06828667970669487, -0.0019653945487388834, 0.0049774108597738205, 0.0]
        dist_coeffs = np.array(D)
        K = np.array([[520.9903919408287, 0.0, 339.3749038438434], 
                    [0.0, 520.3005424418425, 238.66492050564054], 
                    [0.0, 0.0, 1.0]])

        # Check if any markers were detected
        # if ids is not None:
        if len(corners) > 0:
            table_p_c = np.array([0, 0, 0])
            # Loop over all detected markers
            all_markers = [100, 200, 300, 400, 500, 600]
            for i in range(len(ids)):
                if (ids[i] == 600): 
                    # Access the corners of the ith detected marker
                    marker_corners = corners[i][0]
                    
                    # Compute the center of the marker
                    marker_center = np.mean(marker_corners, axis=0)
                    
                    # Convert the center point to a normalized ray in camera coordinates
                    cx = camera_info_width / 2.0
                    cy = camera_info_height / 2.0
                    fx = K[0][0]
                    fy = K[1][1]
                    x_c = (marker_center[0] - cx) / fx
                    y_c = (marker_center[1] - cy) / fy
                    z_c = 1.0

                    [[[x_c, y_c]]] = cv2.undistortPoints(src=np.array([x_c, y_c], dtype=np.float32), cameraMatrix=K, distCoeffs=dist_coeffs)
                    
                    # Convert the point to 3D coordinates in the camera frame
                    table_p_c = np.array([x_c, y_c, z_c])
                    print(table_p_c)

                    # plt.imshow(frame)
                    # plt.plot(x_c, y_c)
                    # plt.show()

                    
                if (i != 6):
                    # Calculate distance
                    # Access the corners of the ith detected marker
                    marker_corners = corners[i][0]
                    
                    # Compute the center of the marker
                    marker_center = np.mean(marker_corners, axis=0)
                    
                    # Convert the center point to a normalized ray in camera coordinates
                    cx = camera_info_width / 2.0
                    cy = camera_info_height / 2.0
                    fx = K[0][0]
                    fy = K[1][1]
                    x_c = (marker_center[0] - cx) / fx
                    y_c = (marker_center[1] - cy) / fy
                    z_c = 1.0
                    [[[x_c, y_c]]] = cv2.undistortPoints(np.array([x_c, y_c]), K, dist_coeffs)
                    
                    # Convert the point to 3D coordinates in the camera frame
                    p_c = np.array([x_c, y_c, z_c])
                    
                    # Calculate distance between table center and object
                    x_diff = table_p_c[0] - p_c[0]
                    y_diff = table_p_c[1] - p_c[1]
                    print("id: {0}".format(ids[i]))
                    print("x_diff, y_diff: {0}, {1}".format(x_diff, y_diff))
                    # return (x_diff, y_diff)
                    
        else:
            # No object found
            # Reply
            # self.say("Sorry, I don't recognize that object")
            print("no object found")
            return (None, None)

        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        return 
        

if __name__ == "__main__":
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-t", "--type", type=str,
    #     default="DICT_ARUCO_ORIGINAL",
    #     help="type of ArUCo tag to detect")
    # args = vars(ap.parse_args())

    get_aruco_distance()







