#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from cluePredictor import cluePrediction
import math

class clue_Detector:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.hsv_callback, queue_size=3) # Image subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)
        
        self.bridge = CvBridge() # CvBridge initialization
        self.board_count = 0
        self.all_data = []
        self.lastCall_time = 0
        self.oneBoard_chars = []
        self.half = []
        self.Prediction = cluePrediction()

    def getBoardCount(self):
        return self.board_count
 
    def hsv_callback(self, data):

        #variables
        lower_blue = np.array([85, 50, 80])
        upper_blue = np.array([130, 255, 255])
        sky_blue_low = np.array([100, 45, 110])
        sky_blue_up = np.array([115, 125, 235])
        img_style = 'bgr8'
        
        # img = cv2.imread("/home/fizzer/Downloads/brick.png")
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
        
        # if self.board_count != 9:
        min_area = 25000 # <22000
        min_ratio = 1.2
        max_ratio = 2
        # else:
        #     min_area = 16000
        #     min_ratio = 1
        #     max_ratio = 2

        sorted_contour = sorted(contours, key=cv2.contourArea, reverse=True)

        if  min_area < cv2.contourArea(sorted_contour[0]):
            x, y, w, h = cv2.boundingRect(sorted_contour[0])

            aspect_ratio = float(w) / h
            if min_ratio < aspect_ratio < max_ratio:
                trim = frame[y: y+h, x: x+w]
                trim = cv2.resize(trim, (1280, 720), interpolation=cv2.INTER_CUBIC)
                # cv2.drawContours(frame, [sorted_contour[0]], -1, (0, 255, 0), 2)

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
            if len(approx) == 4 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(contour)

                if 1.2 < w/h < 2 and 1000 > w > 900 and 650 > h > 550:

                    # Extract vertices of the polygon (corners)
                    corners = approx.reshape(-1, 2)
                    centroid = np.mean(corners, axis=0)

                    # Calculate the angles of each corner with respect to the centroid
                    angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])

                    # Sort the corners based on the angles in counterclockwise order
                    sorted_indices = np.argsort(angles)
                    sorted_corners = corners[sorted_indices]

                    # Access the sorted corners as needed
                    x1, y1 = sorted_corners[0]
                    x2, y2 = sorted_corners[1]
                    x3, y3 = sorted_corners[2]
                    x4, y4 = sorted_corners[3]

                    cv2.circle(trim, (x1, y1), 5, (0, 0, 255), -1)
                    cv2.circle(trim, (x2, y2), 5, (0, 255, 0), -1)
                    cv2.circle(trim, (x3, y3), 5, (255, 0, 0), -1)
                    cv2.circle(trim, (x4, y4), 5, (255, 0, 255), -1)
                    # cv2.imshow("trim", trim)
                    # cv2.waitKey(2)

                    pts1 = np.float32([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
                    pts2 = np.float32([[0, 0], [1280, 0], [1280, 720], [0, 720]])
                    M = cv2.getPerspectiveTransform(pts1, pts2)
                    dst = cv2.warpPerspective(mask, M, (1280, 720))

                    # cv2.imshow("dst", dst)
                    # cv2.waitKey(2)

                    self.words_trim(dst)
                    break

                # self.DO_SIFT(whiteboard) 
    def manual_trim(self, image):
        pass
        

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

        self.lastCall_time = rospy.get_time()

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))       
        cropped = cv2.erode(crop, kernel, 9)
        # cropped = cv2.bilateralFilter(cropped, 11, 25, 25)
        top = cropped[0:360, 0:1280]
        bottom = cropped[360:720, 0:1280]
        
        contours_top, _ = cv2.findContours(top, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_bottom, _ = cv2.findContours(bottom, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # trim_img = cropped.copy()
        iteration = 0
        sorted_contours_top = sorted(contours_top, key=lambda c: cv2.boundingRect(c)[0])
        sorted_contours_bottom = sorted(contours_bottom, key=lambda c: cv2.boundingRect(c)[0])
        
        for contour in sorted_contours_top:
            x, y, w, h = cv2.boundingRect(contour)
            if w < 650 and w > 55 and h < 650 and h > 55:
            
                cv2.rectangle(top, (x-1, y-1), (x + w +1, y + h + 1), (255, 0, 255), 1)

                string_img = top[y-1:y+h+1, x-1:x+w+1]
                self.character_trim(string_img, iteration)
                iteration += 1
        
        # print("length", len(self.half))
        cv2.imshow("top", top)
        cv2.waitKey(2)
        self.oneBoard_chars.append(self.half)
        self.half = []
        


        for contour in sorted_contours_bottom:
            x, y, w, h = cv2.boundingRect(contour)
            if w < 650 and w > 55 and h < 650 and h > 55:
           
                # Add 360 to y-coordinate
                # y += 360
                cv2.rectangle(bottom, (x, y), (x + w, y + h), (255, 0, 255), 1)

                string_img = bottom[y-1:y+h+1, x-1:x+w+1]

                self.character_trim(string_img, iteration)
                iteration += 1
        # print("length", len(self.half))
        self.oneBoard_chars.append(self.half)
        self.half = []
       
        self.all_data.append(self.oneBoard_chars)
        self.oneBoard_chars = [] 

        cv2.imshow("bottom", bottom)
        cv2.waitKey(2)


    def character_trim(self, string_img, iteration):
        h, w = string_img.shape
        space = 110
        num = 1
        folder_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/newCharacters/"

        if w < space and w != 0:
            space = w

        elif w > space:
            space = w //2
            num = 2
        
        # num = math.ceil(w/space)
        # space = w//num
        # print("w:", w)
        
        for character in range(num):
            character_img = string_img[0:h, space * character : space * (character+1)]
            file_name = str(self.board_count) + str(iteration) + str(character) + '.jpg'
            full_path = folder_path + file_name
        
            character_resize = cv2.resize(character_img, (120, 100))
            
            # cv2.imwrite(full_path, character_resize)
            self.half.append(character_resize)



    def boardUpdate(self):
        timePassed = rospy.get_time() - self.lastCall_time
        time_threshold = 0.75
        
        if self.board_count == 3 or self.board_count == 8 or self.board_count == 9:
            time_threshold = 0.05
        else:
            time_threshold = 0.75
        
        if timePassed > time_threshold and self.all_data: # if all_data is not empty
            if self.board_count == 3 or self.board_count == 5:
                good_chars = self.all_data[0]
            else:
                good_chars = self.all_data[-1]
            
            self.call_CNN(good_chars)
    
            self.board_count +=1
            self.all_data = []
            print("GOOD CHARS !!!!!!!!!!!!!!!!!!!")

            
    def call_CNN(self, good_chars):
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