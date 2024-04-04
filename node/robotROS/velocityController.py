
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class velocityController:
    
    def __init__(self):
        self.control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=3) # Velocity publisher

        # Velocities of the robot
        self.angularZ = 0
        self.linearX = 0.5

        self.error = 0
    
    def velocityPublish(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move
    

    def lineFollower(self, image):
        # Variables
        proportionalConstant = 0.02
        derivativeConstant = 0.1

        cutoffFrame = 0.999999999
        height, width = image.shape

        roiHeight = int(cutoffFrame*height)
        croppedFrame = image[roiHeight:height, :]
        center = croppedFrame.shape[1] // 2

        indicesHigh = np.where(croppedFrame > 0)

        if(indicesHigh[1].size > 0):
            firstX = min(indicesHigh[1])
            lastX = max(indicesHigh[1])
            average = int((firstX+lastX)/2)

            self.error = average - center
            
            self.angularZ = -proportionalConstant*self.error
        
        self.velocityPublish(self.linearX, self.angularZ)
    
