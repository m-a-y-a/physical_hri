import cv2
import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import sys


def get_aruco_distance():
    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()


    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(10.0)

    while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 1000 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=1000)
        cv2.imshow("Frame", frame)

        # detect ArUco markers in the input frame
        (corners, ids, rejected_img_points) = cv2.aruco.detectMarkers(frame,
            aruco_dict, parameters=aruco_params)

    

    # Detect the markers in the image
    # corners, ids, rejected_img_points = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=aruco_params)

        # Check if any markers were detected
        # if ids is not None:
        if len(corners) > 0:
            table_p_c = np.array([0, 0, 0])
            # Loop over all detected markers
            for i in range(len(ids)):
                if (i == 0): # IDK ABOUT THIS @CAROLYNE
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
                    # self.say("Sorry, I don't recognize that object")
                    print("no object found")
                    return (None, None)
            # show the output frame
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

if __name__ == "__main__":
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-t", "--type", type=str,
    #     default="DICT_ARUCO_ORIGINAL",
    #     help="type of ArUCo tag to detect")
    # args = vars(ap.parse_args())

    get_aruco_distance()







