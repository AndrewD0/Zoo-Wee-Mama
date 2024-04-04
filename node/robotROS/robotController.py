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

        self.previousFrame = np.zeros((720,1280,3), dtype = np.uint8)

    
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
        lowerSoil = np.array([184, 134, 11])
        upperSoil = np.array([143, 188, 143])

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
        
        whiteHighlight = cv2.inRange(frame, lowerWhite, upperWhite)
        pinkHighlight = cv2.inRange(frame, lowerPink, upperPink)
        redHighlight = cv2.inRange(frame, lowerRed, upperRed)

        self.stateTracker.findState(pinkHighlight, redHighlight)

        print(self.stateTracker.getState())

        print(frame.dtype)
        print(self.previousFrame.dtype)

        if(self.stateTracker.getState() == 'ROAD'):
            self.velocityController.lineFollower(roadHighlight)
        
        elif(self.stateTracker.getState() == 'PEDESTRIAN'):
            self.velocityController.velocityPublish(0,0)
            frameDifference = cv2.absdiff(frame, self.previousFrame)
            frameGray = cv2.cvtColor(frameDifference, cv2.COLOR_BGR2GRAY)
            _, binaryDifference = cv2.threshold(frameGray, 50, 255, cv2.THRESH_BINARY)

            kernelSize = 3
            kernel = np.ones((kernelSize, kernelSize), np.uint8)
            binaryDifference = cv2.dilate(binaryDifference, kernel, iterations = 3)
        
        contours, _ = cv2.findContours(binaryDifference, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(frame, contours, -1, (0,255,0), 2)

        cv2.imshow("image", frame)

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

    robotControl = robotController()
    rospy.init_node('robot_control_node', anonymous=True) # Initialize node
    rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()