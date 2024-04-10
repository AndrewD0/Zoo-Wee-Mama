#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from cluePredictor import cluePrediction

class clue_Detector:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.hsv_callback, queue_size=3) # Image subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)
        
        self.bridge = CvBridge() # CvBridge initialization
        self.board_count = 0
        self.all_data = []
        self.lastCall_time = 0
        self.oneBoard_chars = []
        self.Prediction = cluePrediction()

    def getBoardCount(self):
        return self.board_count
    
    def getData(self):
        return self.all_data
 
    def hsv_callback(self, data):

        #variables
        lower_blue = np.array([85, 50, 80])
        upper_blue = np.array([130, 255, 255])
        sky_blue_low = np.array([100, 45, 110])
        sky_blue_up = np.array([115, 125, 235])
        img_style = 'bgr8'
        
        # img = cv2.imread("/home/fizzer/Downloads/sky1.png")
        # hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # # Calculate the mean HSV values of the entire image
        # mean_hsv = cv2.mean(hsv_image)

        # # Print the mean HSV values
        # print("Mean HSV values:", mean_hsv[:3])

        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)
        
        self.boardUpdate()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_region = cv2.bitwise_and(frame, frame, mask=mask_blue)
        sky_mask = cv2.inRange(hsv, sky_blue_low, sky_blue_up)
        no_sky = cv2.bitwise_not(sky_mask)
        filtered = cv2.bitwise_and(blue_region, blue_region, mask=no_sky)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        #cv2.imshow("blue_filter", gray)
        #cv2.waitKey(1)

        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 18000 # <22000
        max_area = 26000

        sorted_contour = sorted(contours, key=cv2.contourArea, reverse=True)

        if  min_area < cv2.contourArea(sorted_contour[0]) < max_area:
            x, y, w, h = cv2.boundingRect(sorted_contour[0])
            trim = frame[y: y+h, x: x+w]
            trim = cv2.resize(trim, (1280, 720), interpolation=cv2.INTER_CUBIC)
            cv2.drawContours(frame, [sorted_contour[0]], -1, (0, 255, 0), 2)

           # cv2.imshow("trim", trim)
            # cv2.imshow("drawcontour", frame)
            cv2.waitKey(2)    
            self.whiteBoard(trim)
                        
    def whiteBoard(self, trim):
        hsv = cv2.cvtColor(trim, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])  # Example lower HSV values for blue
        upper_blue = np.array([130, 255, 255])  # Example upper HSV values for blue

        # Create a mask to get rid of blue regions
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Invert the mask to keep non-blue regions
        inverse_mask = cv2.bitwise_not(mask)
        contours, _ = cv2.findContours(inverse_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        sorted_contour = sorted(contours, key=cv2.contourArea, reverse=False)

        for contour in sorted_contour:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) == 4 and  cv2.isContourConvex(approx) and cv2.contourArea(contour) > 20000:
                x, y, w, h = cv2.boundingRect(contour)
                whiteboard = trim[y: y+h, x: x+w]
                cv2.drawContours(trim, [contour], -1, (0, 255, 0), 2)
                whiteboard = cv2.resize(whiteboard, (1280, 720))

                # pts1 = np.float32([[x, y], [x+w, y], [x, y+h], [x+w, y+h]])
                # pts2 = np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]])
                # M = cv2.getPerspectiveTransform(pts1, pts2)
                # dst = cv2.warpPerspective(whiteboard, M, (1280, 720))

                # # Get the bounding box of the transformed points
                # min_x = max(int(np.min(dst[:, 0, 0])), 0)
                # min_y = max(int(np.min(dst[:, 0, 1])), 0)
                # max_x = min(int(np.max(dst[:, 0, 0])), 1280)
                # max_y = min(int(np.max(dst[:, 0, 1])), 720)

                # # Crop the region from the homography image based on the adjusted coordinates
                # cropped_roi = whiteboard[min_y:max_y, min_x:max_x]
                # cropped_roi = cv2.resize(cropped_roi, (1280, 720))
                # cv2.imshow("cropped", cropped_roi)
                # cv2.waitKey(2)

                # lower_blue = np.array([90, 50, 50])
                # upper_blue = np.array([130, 255, 255])
                
                # hsv = cv2.cvtColor(cropped_roi, cv2.COLOR_BGR2HSV)
                # cropped_roi = cv2.inRange(hsv, lower_blue, upper_blue)

                # self.words_trim(cropped_roi)
                # break

                self.DO_SIFT(whiteboard)   
        

    def DO_SIFT(self, frame):

        template_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/000.png"
        img = cv2.imread(template_path)

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

           # Get the bounding box of the transformed points
            min_x = max(int(np.min(dst[:, 0, 0])), 0)
            min_y = max(int(np.min(dst[:, 0, 1])), 0)
            max_x = min(int(np.max(dst[:, 0, 0])), w)
            max_y = min(int(np.max(dst[:, 0, 1])), h)

            # Crop the region from the homography image based on the adjusted coordinates
            cropped_roi = frame[min_y:max_y, min_x:max_x]
            cropped_roi = cv2.resize(cropped_roi, (1280, 720))

            lower_blue = np.array([90, 50, 50])
            upper_blue = np.array([130, 255, 255])
            
            hsv = cv2.cvtColor(cropped_roi, cv2.COLOR_BGR2HSV)
            cropped_roi = cv2.inRange(hsv, lower_blue, upper_blue)

            self.words_trim(cropped_roi)
            

        except Exception as e:
            pass

            
    def words_trim(self, crop):
        self.oneBoard_chars = []
        self.lastCall_time = rospy.get_time()

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))       
        cropped = cv2.erode(crop, kernel, 1)
        # cropped = cv2.dilate(crop, kernel, iterations=1)
        top = cropped[0:360, 0:1280]
        bottom = cropped[360:720, 0:1280]
        
        contours_top, _ = cv2.findContours(top, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_bottom, _ = cv2.findContours(bottom, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        trim_img = cropped.copy()
        iteration = 0
        sorted_contours_top = sorted(contours_top, key=lambda c: cv2.boundingRect(c)[0])
        sorted_contours_bottom = sorted(contours_bottom, key=lambda c: cv2.boundingRect(c)[0])
        
        for contour in sorted_contours_top:
            x, y, w, h = cv2.boundingRect(contour)
            if w < 650 and w > 55 and h < 650 and h > 55:
                cv2.rectangle(trim_img, (x-1, y-1), (x + w +1, y + h + 1), (255, 0, 255), 1)

                string_img = trim_img[y-1:y+h+1, x-1:x+w+1]
                self.character_trim(string_img, iteration)
                iteration += 1
        
        for contour in sorted_contours_bottom:
            x, y, w, h = cv2.boundingRect(contour)
            if w < 650 and w > 55 and h < 650 and h > 55:
                # Add 360 to y-coordinate
                y += 360
                cv2.rectangle(trim_img, (x, y), (x + w, y + h), (255, 0, 255), 1)
                # cv2.imshow("char", trim_img)
                cv2.waitKey(2)

                string_img = trim_img[y:y+h, x:x+w]
                self.character_trim(string_img, iteration)
                iteration += 1

    def character_trim(self, string_img, iteration):
        h, w = string_img.shape
        space = 110
        folder_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/newCharacters/"

        if w < space:
            space = w
        num = int(w//space)
        
        for character in range(num):
            character_img = string_img[0:h, space * character : space * (character+1)]
            file_name = str(self.board_count) + str(iteration) + str(character) + '.jpg'
            full_path = folder_path + file_name
        
            character_blur = cv2.bilateralFilter(character_img, 9, 75, 75)
            character_resize = cv2.resize(character_blur, (120, 100))
            
            # cv2.imwrite(full_path, character_resize)
            self.oneBoard_chars.append(character_resize)
            self.all_data.append(self.oneBoard_chars)

    def boardUpdate(self):
        good_chars = []
        timePassed = rospy.get_time() - self.lastCall_time
        
        if timePassed > 0.5 and self.all_data: # if all_data is not empty
            good_chars = self.all_data[-1]
            self.board_count +=1
            self.all_data = []
            print("GOOD CHARS !!!!!!!!!!!!!!!!!!!")

            self.call_CNN(good_chars)
    
    def call_CNN(self, good_chars):
        pass
        self.Prediction.predict(good_chars)

    

def main():

    rospy.init_node('clueboard_detection_node', anonymous=True) # Initialize node
    clueDetection = clue_Detector()
    
    rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()