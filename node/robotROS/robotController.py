#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from velocityController import velocityController
from stateTracker import stateTracker

from robotFunctions import functions

class robotController:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback, queue_size=3) # Image subscriber
        self.clock = rospy.Subscriber('/clock', Clock, self.clockCallback, queue_size=10) # Clock subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)

        # Velocity control object
        self.velocityController = velocityController()

        # State tracking object
        self.stateTracker = stateTracker()
        
        self.bridge = CvBridge() # CvBridge initialization
        self.previousFrame = np.zeros((720,1280,3), dtype = np.uint8) # Creating a variable that stores the previous frame
        self.previousTime = 0
        self.prevTimeCounter = 0

    
    def clockCallback(self, data):
        # Start timer
        startTime = 0
        duration = 5
        msgStart = 'ZoWeMama,lisndrew,0,START'
        msgStop = 'ZoWeMama,lisndrew,-1,STOP'

        if (rospy.get_time() == startTime):
            self.scoretracker.publish(msgStart)
        elif (rospy.get_time() == duration):
            self.scoretracker.publish(msgStop)


    def callback(self, data):
        # Threshold variables

        # Detecting white
        lowerWhite = np.array([235,235,235])
        upperWhite = np.array([255,255,255])

        # Detecting the road in HSV
        lowerRoad = np.array([0, 0, 0])
        upperRoad = np.array([128, 128, 128])

        # Detecting the soil section
        lowerSoil = np.array([22, 54, 198]) #[184, 134, 11] #mean grass: 27, 60, 205
        upperSoil = np.array([35, 74, 212]) #[143, 188, 143]

        image = cv2.imread("/home/fizzer/Downloads/grass.png")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Calculate the mean HSV values of the entire image
        mean_hsv = np.mean(hsv_image, axis=(0, 1))

        # Print the mean HSV values
        print(f"Average HSV values: H={mean_hsv[0]}, S={mean_hsv[1]}, V={mean_hsv[2]}")

    
        # Detecting red
        lowerRed = np.array([0,0,235])
        upperRed = np.array([20,20,255])

        #Detecting pink
        lowerPink = np.array([200, 0, 200])
        upperPink = np.array([255, 30, 255])

        imgStyle = 'bgr8'

        # Obtain a frame
        try:
            frame = self.bridge.imgmsg_to_cv2(data, imgStyle) # Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)


        # We use HSV for some methods
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        roadHighlight = cv2.inRange(hsvFrame, lowerRoad, upperRoad)
        soilHighlight = cv2.inRange(hsvFrame, lowerSoil, upperSoil)
        blurred = cv2.GaussianBlur(soilHighlight, (5, 5), 0)
        # gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(blurred, 300, 400)
        lines = cv2.HoughLines(edge, rho=1, theta=np.pi / 180, threshold=100)  # Adjust parameters as needed

        croppedROI = soilHighlight.copy()
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(croppedROI, (x1, y1), (x2, y2), (255, 255, 255), 2)  # Draw red lines to connect Hough lines
        croppedROI = cv2.resize(croppedROI[400:720, 0:1280], (1280, 720), 0)
        cv2.imshow("contaus", croppedROI)
        
        whiteHighlight = cv2.inRange(frame, lowerWhite, upperWhite)
        pinkHighlight = cv2.inRange(frame, lowerPink, upperPink)
        redHighlight = cv2.inRange(frame, lowerRed, upperRed)

        self.stateTracker.findState(pinkHighlight, redHighlight)

        print(self.stateTracker.getState())

        if(self.stateTracker.getState() == 'ROAD'):
            self.velocityController.lineFollower(whiteHighlight)
        
        elif(self.stateTracker.getState() == 'PEDESTRIAN'):
            self.velocityController.velocityPublish(0,0)
            
            if(self.prevTimeCounter == 0):
                self.previousTime = rospy.get_time()
                self.prevTimeCounter = 1
            
            if(rospy.get_time() > self.previousTime+0.75):
                if(functions.pedestrianCrossed(frame, self.previousFrame) == True):
                    self.stateTracker.setState('ROAD')

        elif(self.stateTracker.getState() == 'ROUNDABOUT'):
            self.velocityController.RoundAboutFollower(whiteHighlight)
        
        elif(self.stateTracker.getState() == 'GRASS'):
            self.velocityController.lineFollower(croppedROI)
        
        cv2.imshow("road", whiteHighlight)
        cv2.imshow("soil", soilHighlight)

        cv2.waitKey(2)

        self.previousFrame = frame


def spawnPosition(self, position):
    msg = ModelState()
    msg.model_name = 'R1'

    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.x = position[3]
    msg.pose.orientation.y = position[4]
    msg.pose.orientation.z = position[5]
    msg.pose.orientation.w = position[6]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( msg )

    except rospy.ServiceException:
        print ("Service call failed")




def main():

    rospy.init_node('robot_control_node', anonymous=True) # Initialize node
    robotControl = robotController()
    rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()