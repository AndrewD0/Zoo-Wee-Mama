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
        self.board_count = 0
 
    def hsv_callback(self, data):
        #variables
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        img_style = 'bgr8'
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        # blur = cv2.GaussianBlur(mask_blue, (3, 3), 0)

        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        board_blue = []
        min_area = 20000
        max_area = 22000

        # con = frame.copy()

        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) == 4 and cv2.contourArea(contour) > min_area and cv2.contourArea(contour) < max_area and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(contour)
                trim = frame[y: y+h, x: x+w]
                # trim = cv2.cvtColor(trim, cv2.COLOR_BGR2GRAY)
                trim = cv2.resize(trim, (1280, 720), interpolation=cv2.INTER_LANCZOS4)
                self.DO_SIFT(trim)
                # cv2.drawContours(con, [contour], -1, (0, 255, 0), 2)
                
                board_blue.append(approx)
        
            
        cv2.imshow("draw", frame)
        cv2.waitKey(2)
        
        
        


    def DO_SIFT(self, frame):

        template_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/000.png"
        img = cv2.imread(template_path)

        cv2.imshow("gray", frame)
        cv2.waitKey(2)

        self.feacture_match(frame, img)

    def feacture_match(self, frame, img):
        sift = cv2.SIFT_create()

        kp_image, desc_image = sift.detectAndCompute(img, None) # get keypoints and descriptors of loaded image
        # Feature matching
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params) # find matching features

        kp_grayframe, desc_grayframe = sift.detectAndCompute(frame, None)

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

            # homography = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)
            # cv2.imshow("Homography", homography)
            # cv2.waitKey(2)

           # Get the bounding box of the transformed points
            min_x = max(int(np.min(dst[:, 0, 0])), 0)
            min_y = max(int(np.min(dst[:, 0, 1])), 0)
            max_x = min(int(np.max(dst[:, 0, 0])), w)
            max_y = min(int(np.max(dst[:, 0, 1])), h)

            # Crop the region from the homography image based on the adjusted coordinates
            cropped_roi = frame[min_y:max_y, min_x:max_x]
            cropped_roi = cv2.resize(cropped_roi, (1280, 720))

            # Save the cropped ROI as a new image
            # cv2.imwrite("cropped_roi.jpg", cropped_roi)

            # Display the cropped ROI (optional)
            # cv2.imshow("Cropped ROI", cropped_roi)
            # cv2.waitKey(2)

            self.words_trim(cropped_roi)
            self.board_count +=1

        except Exception as e:
            # rospy.logerr("Homography error: %s", str(e))
            # homography = frame
            pass
            
    def words_trim(self, crop):
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        cropped = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        # _, cropped = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)
        
        cv2.imshow("cropp", cropped)
        cv2.waitKey(2)
        
        contours, _ = cv2.findContours(cropped, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        trim_img = cropped.copy()
        iteration = 0

        for contour in contours:
            
            x, y, w, h = cv2.boundingRect(contour)
            
            if w<650 and w>55 and h<650 and h>55:
                cv2.rectangle(trim_img, (x-1, y-1), (x+1 + w, y+1 + h), (255, 0, 255), 1)
                cv2.imshow("char", trim_img)
                cv2.waitKey(2)

                string_img = trim_img[(y-1):(y+h), (x-1):(x+w)]
                self.character_trim(string_img, iteration)
                iteration += 1

    def character_trim(self, string_img, iteration):
        h, w = string_img.shape
        space = 100
        folder_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/Characters/"

        if w < space:
            space = w
        else: 
            space = 100

        num = int(w//space)
        
        for character in range(num):
            character_img = string_img[0:h, space * character : space * (character+1)]
            file_name = str(self.board_count) + str(iteration) + str(character) + '.jpg'
            full_path = folder_path + file_name
            character_img = cv2.GaussianBlur(character_img, (3, 3), 0)
            # character_img = cv2.resize(character_img, (100, 100))
            cv2.imwrite(full_path, character_img)


def main():

    clueDetection = clueDetector()
    rospy.init_node('clueboard_detection_node', anonymous=True) # Initialize node
    # rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()