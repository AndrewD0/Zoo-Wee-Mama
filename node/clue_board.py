#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from line_following import Imag_Convert
from PyQt5 import QtCore, QtGui, QtWidgets

class Clueboard_detection:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.hsv_callback, queue_size=3) # Image subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)
        
        self.bridge = CvBridge() # CvBridge initialization

    def hsv_callback(self, data):
        #variables
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 150])

        img_style = 'bgr8'
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)
        
        blur = cv2.blur(frame, (3, 3))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        words = cv2.inRange(hsv, lower_blue, upper_blue)

        cv2.imshow("image",words)
        cv2.waitKey(2)

        self.SLOT_query_camera(words)

    def Clue_detection():
        pass


    def SLOT_query_camera(self, words):
        template_path = "/home/fizzer/Downloads/0.png"

        #TODO run SIFT on the captured frame
        img = cv2.imread(template_path)
        self.feacture_match(words, img)

    def feacture_match(self, frame, img):
        sift = cv2.SIFT_create()

        kp_image, desc_image = sift.detectAndCompute(img, None) # get keypoints and descriptors of loaded image
        # Feature matching
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params) # find matching features

        kp_grayframe, desc_grayframe = sift.detectAndCompute(frame, None)

        draw = cv2.drawKeypoints(frame, kp_grayframe,frame)
        # cv2.imshow("draw",draw)

        matches = flann.knnMatch(desc_image, desc_grayframe, k=2)
        good_points = []
        for m, n in matches:
            if m.distance < 0.6 * n.distance:
                good_points.append(m)

        self.homography(frame, kp_image, kp_grayframe, good_points, img)

    def homography(self, frame, kp_image, kp_grayframe, good_points, img):
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
        
        try:
            print("!!!!")
            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            # Perspective transform
            h, w= img.shape
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, matrix)

            homography = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            cv2.imshow("Homography", homography)
        except:
            homography = frame

def main():

    clue_detector = Clueboard_detection()
    rospy.init_node('clueboard_detection_node', anonymous=True) # Initialize node
    rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()