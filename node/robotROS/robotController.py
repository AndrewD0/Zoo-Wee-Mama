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
        self.startTime = 0
        self.GrassTransition = False
        self.respawned = False
    
    def clockCallback(self, data):
        # Start timer
        duration = 240
        msgStart = 'ZoWeMama,lisndrew,0,START'
        msgStop = 'ZoWeMama,lisndrew,-1,STOP'

        if not self.started:
            if (rospy.has_param('launch_called')):
                launch_called = rospy.get_param('launch_called')
                if launch_called:
                    self.scoretracker.publish(msgStart)
                    self.started = True
                    self.startTime = rospy.get_time()
        
        elif (rospy.get_time() == self.startTime + duration):
            self.scoretracker.publish(msgStop)


    def callback(self, data):
        # Obtain a frame
        try:
            frame = self.bridge.imgmsg_to_cv2(data, constants.IMG_STYLE) # Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)

        # We use HSV for some methods
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

        soilHighlight = cv2.inRange(hsvFrame, constants.LOWER_SOIL, constants.UPPER_SOIL)
        #soilHighlight = cv2.GaussianBlur(soilHighlight, (3,3), 0)
     
        ret, whiteHighlight = cv2.threshold(grayFrame, constants.LOWER_WHITE, constants.UPPER_WHITE, cv2.THRESH_BINARY)
        whiteHighlight = cv2.GaussianBlur(whiteHighlight, (9,9),0)

        pinkHighlight = cv2.inRange(frame, constants.LOWER_PINK, constants.UPPER_PINK)
        redHighlight = cv2.inRange(frame, constants.LOWER_RED, constants.UPPER_RED)

        tunnelHighlight = cv2.inRange(hsvFrame, constants.LOWER_TUNNEL, constants.UPPER_TUNNEL)
        
        mask_blue = cv2.inRange(hsvFrame, constants.LOWER_BLUE, constants.UPPER_BLUE)
        blue_region = cv2.bitwise_and(frame, frame, mask=mask_blue)
        sky_mask = cv2.inRange(hsvFrame, constants.LOWER_SKY, constants.UPPER_SKY)
        no_sky = cv2.bitwise_not(sky_mask)
        filtered = cv2.bitwise_and(blue_region, blue_region, mask=no_sky)
        boardHighlight = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow("tunnel", boardHighlight)
        # cv2.imshow("soil", soilHighlight)
        cv2.waitKey(2)

        self.stateTracker.findState(pinkHighlight, redHighlight)

        print(self.stateTracker.getState())

        if(self.stateTracker.getState() == 'ROAD'):
            self.velocityController.lineFollower(whiteHighlight, frame, 'ROAD')

            if(self.stateTracker.getCluesCounter() == 2):
                self.velocityController.setLinearX(0.3)
                # self.velocityController.setBias(20)

            # elif(self.stateTracker.getCluesCounter() == 3):
                # self.velocityController.velocityPublish(-0.1, 0)
            
            elif(self.stateTracker.getCluesCounter() == 3):

                self.stateTracker.setState('ROUNDABOUT')

                self.velocityController.velocityPublish(0,0)
                self.stateTracker.startRoundabout(True)
                self.prevTimeCounter = 0
                

        elif(self.stateTracker.getState() == 'PEDESTRIAN'):
            self.velocityController.velocityPublish(0,0)
            
            # I don't like this implementation
            if(self.prevTimeCounter == 0):
                self.previousTime = rospy.get_time()
                self.prevTimeCounter = 1

            if(rospy.get_time() > self.previousTime+0.75):
                if(robotFunctions.pedestrianCrossed(frame, self.previousFrame) == True):
                    self.stateTracker.setState('ROAD')
        
        elif(self.stateTracker.getState() == 'ROUNDABOUT'):

            if(self.stateTracker.checkRoundabout() == True):
                if(self.prevTimeCounter == 0):
                
                    self.velocityController.setAngularZ(0)
                    self.velocityController.setBias(0)
                    self.velocityController.setError(0)

                    self.velocityController.setLinearX(0.15)
                    self.velocityController.setProportionalConstant(0.03)

                    self.previousTime = rospy.get_time()
                    self.prevTimeCounter = 1

                # Timer
                if(rospy.get_time() < self.previousTime + 1):
                    self.velocityController.velocityPublish(0,-1.05)
                elif(rospy.get_time() < self.previousTime + 2.53):
                        self.velocityController.velocityPublish(0.3, 0)

                else:
                    self.stateTracker.startRoundabout(False)

            else:
                self.velocityController.roundaboutFollower(whiteHighlight, frame)
                if(self.stateTracker.getCluesCounter() == 4):
                    self.stateTracker.setState('ROAD')

                    self.velocityController.setProportionalConstant(0.03)
                    self.velocityController.setLinearX(0.4)

                    self.prevTimeCounter = 0
    
        elif(self.stateTracker.getState() == 'GRASS'):
            # if self.GrassTransition == False:
            #     previous_time = rospy.get_time()
            #     self.GrassTransition = True
            #     while(rospy.get_time() - previous_time < 0.5):
            #         self.velocityController.velocityPublish(0.45, 0)
            self.velocityController.setLinearX(0.35)
            self.velocityController.setProportionalConstant(0.02)
            
            if(self.prevTimeCounter == 0):
                self.previousTime = rospy.get_time()
                self.prevTimeCounter = 1

            if(rospy.get_time() < self.previousTime + 0.60):
                self.velocityController.velocityPublish(0.25,0)
                self.velocityController.setAngularZ(0)
                self.velocityController.setError(0)
            else:

                self.velocityController.soilFollower(soilHighlight, frame)

                
                self.velocityController.setBias(130)
                self.velocityController.setLinearX(0.3)
            
                if(self.stateTracker.getCluesCounter() == 6): # change back to 5
                    self.velocityController.setBias(-40)
                    

                elif(self.stateTracker.getCluesCounter() == 7): # change back to 6
                    # self.stateTracker.setState('YODA')
                    if self.respawned == False:
                        self.spawnPosition([-4.2, -2.3, 0.2, 1])
                        self.respawned = True

                
        elif(self.stateTracker.getState() == 'YODA'):
            self.velocityController.yodaFollower(tunnelHighlight, boardHighlight, pinkHighlight, frame)
            if(self.stateTracker.getCluesCounter() == 8):
                    self.stateTracker.setState('TUNNEL')

        elif(self.stateTracker.getState() == 'TUNNEL'):
            self.velocityController.velocityPublish(0,0)
        
            if(self.stateTracker.getCluesCounter() == 9):
                if self.tunnelCount == True:
                    self.previousTime = rospy.get_time()
                    self.tunnelCount = False
                if rospy.get_time() - self.previousTime < 5:
                    self.velocityController.velocityPublish(0.3, 0)
                else:
                    self.velocityController.mountainClimber(, frame)

        self.previousFrame = frame

    # def Position(self):

    #     # index = msg.name.index(msg.model_name)
    #     pose = self.msg.pose
    #     x_position = pose.position.x
    #     y_position = pose.position.y
    #     w_orientation = pose.orientation.w

    #     print("x-position: ", x_position)
    #     print("y-position: ", y_position)
    #     print("w-position: ", w_orientation)

    #     tolerance = 0.01

    #     if abs(x_position - (-5.7)) > tolerance:
    #         self.velocityController.velocityPublish(0.5, 0)

    #     elif abs(w_orientation - 0.5) > tolerance:
    #         self.velocityController.velocityPublish(0, 0.5)

    #     elif abs(y_position - (-2.3)) > tolerance:
    #         self.velocityController.velocityPublish(0.5, 0)


    def spawnPosition(self, position):
        msg = ModelState()
        msg.model_name = 'R1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        # msg.pose.orientation.x = position[3]
        # msg.pose.orientation.y = position[4]
        msg.pose.orientation.w = position[3]
        # msg.pose.orientation.w = position[6]

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