#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String

class clueDetector:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.hsv_callback, queue_size=3) # Image subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)
        
        self.bridge = CvBridge() # CvBridge initialization

        self.board = False
        self.frame_counter = 1
        self.blur = []
        self.board_count = 0

        # img = cv2.imread("/home/fizzer/ros_ws/src/Zoo-Wee-Mama/00.png")
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # hsv_pixel =hsv[200, 200]
        # print("HSV values of pixel at ({}, {}): H={}, S={}, V={}".format(100, 100, hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]))
        
    def hsv_callback(self, data):
        #variables
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # edge_blue = np.array([120, 255, 102])

        img_style = 'bgr8'
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        words = cv2.inRange(hsv, lower_blue, upper_blue)
        self.blur = cv2.GaussianBlur(words, (3, 3), 0)
        
        # blur_canny = cv2.GaussianBlur(hsv, (3, 3), 0)
        # canny = cv2.inRange(blur_canny, edge_blue, edge_blue)
        # edges = cv2.Canny(canny, 50, 150)

        if np.any(self.blur[300] == 255) and np.any(self.blur[485] == 255): # if white
            self.board = True
            self.frame_counter -= 1
        else:
            self.board = False

        # cv2.imshow("image",self.blur)
        cv2.waitKey(2)

        self.SLOT_query_camera(frame)


    def SLOT_query_camera(self, frame):

        template_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/000.png"
        img = cv2.imread(template_path)

        if self.board and self.frame_counter == 0:
            self.feacture_match(frame, img)
            self.frame_counter = 5

    def feacture_match(self, frame, img):
        sift = cv2.SIFT_create()

        kp_image, desc_image = sift.detectAndCompute(img, None) # get keypoints and descriptors of loaded image
        # Feature matching
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params) # find matching features

        kp_grayframe, desc_grayframe = sift.detectAndCompute(frame, None)

        # draw = cv2.drawKeypoints(frame, kp_grayframe,frame)
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
            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            # Perspective transform
            h, w, _ = img.shape
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)

            dst = cv2.perspectiveTransform(pts, matrix)

            homography = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            #cv2.imshow("Homography", homography)
            cv2.waitKey(2)

           # Get the bounding box of the transformed points
            min_x = max(int(np.min(dst[:, 0, 0])), 0)
            min_y = max(int(np.min(dst[:, 0, 1])), 0)
            max_x = min(int(np.max(dst[:, 0, 0])), w)
            max_y = min(int(np.max(dst[:, 0, 1])), h)

            # Crop the region from the homography image based on the adjusted coordinates
            cropped_roi = self.blur[min_y:max_y, min_x:max_x]
            cropped_roi = cv2.resize(cropped_roi, (1280, 720))

            # Save the cropped ROI as a new image
            cv2.imwrite("cropped_roi.jpg", cropped_roi)

            # Display the cropped ROI (optional)
            # cv2.imshow("Cropped ROI", cropped_roi)
            # cv2.waitKey(2)

            self.words_trim(cropped_roi)
            self.board_count +=1

        except Exception as e:
            # rospy.logerr("Homography error: %s", str(e))
            # homography = frame
            pass
            
    def words_trim(self, cropped):
        

        contours, _ = cv2.findContours(cropped, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        trim_img = cropped.copy()
        iteration = 0

        for contour in contours:
            
            x, y, w, h = cv2.boundingRect(contour)
            
            if w<650 and w>55 and h<650 and h>55:
                cv2.rectangle(trim_img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                #cv2.imshow("contour", trim_img)
                cv2.waitKey(2)

                string_img = trim_img[y:y+h, x:x+w]
                self.character_trim(string_img, iteration)
                iteration += 1
                # cv2.imwrite("Characters.jpg", character)
                # cv2.imshow("Character", string_img)
                # cv2.waitKey(2)

    def character_trim(self, string_img, iteration):
        h, w = string_img.shape
        space = 80
        folder_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/Characters/"

        if int(w//space) == 0:
            num = 1
        else:
            num = int(w//space)
        
        for character in range(num):
            character_img = string_img[0:h, space * character : space * (character+1)]
            file_name = str(self.board_count) + str(iteration) + str(character) + '.jpg'
            full_path = folder_path + file_name
            cv2.imwrite(full_path, character_img)

            # cv2.imshow("char", character_img)
            # cv2.waitKey(2)


def main():

    clueDetection = clueDetector()
    rospy.init_node('clueboard_detection_node', anonymous=True) # Initialize node
    rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()