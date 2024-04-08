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

from robotHelpers import robotFunctions
from robotHelpers import constants

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
        self.started = False

    
    def clockCallback(self, data):
        # Start timer
        startTime = 0
        duration = 240
        msgStart = 'ZoWeMama,lisndrew,0,START'
        msgStop = 'ZoWeMama,lisndrew,-1,STOP'

        if not self.started:
            if (rospy.has_param('launch_called')):
                launch_called = rospy.get_param('launch_called')
                if launch_called:
                    self.scoretracker.publish(msgStart)
                    self.started = True
                    startTime = rospy.get_time()
        
        if (rospy.get_time() == startTime + duration):
            self.scoretracker.publish(msgStop)


    def callback(self, data):

        # Obtain a frame
        try:
            frame = self.bridge.imgmsg_to_cv2(data, constants.IMG_STYLE) # Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)


        # We use HSV for some methods
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   

        soilHighlight = cv2.inRange(hsvFrame, constants.LOWER_SOIL, constants.UPPER_SOIL)
        soilHighlight = cv2.medianBlur(soilHighlight, 5)
        
        whiteHighlight = cv2.inRange(frame, constants.LOWER_WHITE, constants.UPPER_WHITE)
        whiteHighlight = cv2.GaussianBlur(whiteHighlight, (5, 5), 0)

        pinkHighlight = cv2.inRange(frame, constants.LOWER_PINK, constants.UPPER_PINK)
        redHighlight = cv2.inRange(frame, constants.LOWER_RED, constants.UPPER_RED)

        self.stateTracker.findState(pinkHighlight, redHighlight)

        if(self.stateTracker.getState() == 'ROAD'):


            self.velocityController.lineFollower(whiteHighlight, frame)

            cv2.imshow("soil", soilHighlight)
            cv2.imshow("frame", frame)
            cv2.waitKey(2)

        
        elif(self.stateTracker.getState() == 'PEDESTRIAN'):
            self.velocityController.velocityPublish(0,0)
            
            # I don't like this implementation
            if(self.prevTimeCounter == 0):
                self.previousTime = rospy.get_time()
                self.prevTimeCounter = 1
            
            if(rospy.get_time() > self.previousTime+0.75):
                if(robotFunctions.pedestrianCrossed(frame, self.previousFrame) == True):
                    self.velocityController.velocityPublish(0.5, 0)
                    rospy.sleep(1)
                    self.stateTracker.PedestrainEnd(redHighlight)

                    

                    # while (rospy.get_time() < self.previousTime+4):
                    #     print("go straight!!")
                    
                    # self.stateTracker.setState('ROAD')
        
        elif(self.stateTracker.getStates() == 'ROUNDABOUT'):
            pass
    
        elif(self.stateTracker.getStates() == 'GRASS'):
            pass
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
    # rospy.sleep(1) 

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()