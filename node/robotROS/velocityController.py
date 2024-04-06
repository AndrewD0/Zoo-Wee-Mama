
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

        height, width = image.shape
        road_width = 1500

        for line in range(height-5, height//2, -1):
            croppedFrame = image[line,:]

            indicesHigh = np.where(croppedFrame > 0)[0]
            center = width // 2

            if(indicesHigh.size > 0):
                firstX = min(indicesHigh)
                lastX = max(indicesHigh)

                if firstX < center and lastX > center:
                    average = int((firstX+lastX)/2)
                    self.error = average - center
                    self.angularZ = -proportionalConstant*self.error
                    # print("two lines", average)
                    break
                elif lastX < center:
                    average = int((firstX+road_width)/2)
                    self.error = average - center
                    self.angularZ = -proportionalConstant*self.error
                    # print("left line", average)
                    break
                elif firstX > center:
                    average = int((lastX-(road_width-width))/2)
                    self.error = average - center
                    self.angularZ = -proportionalConstant*self.error
                    # print("right line", average)
                    break
                else:
                    continue
            else:
                continue

        # cv2.imshow("image", image)
        # cv2.waitKey(2)
        
        self.velocityPublish(self.linearX, self.angularZ)

    def RoundAboutFollower(self, image):
        # Variables
        proportionalConstant = 0.012
        derivativeConstant = 0.1

        height, width = image.shape

        for line in range(height-5, height//2, -1):
            croppedFrame = image[line,:]

            indicesHigh = np.where(croppedFrame > 0)[0]
            center = width // 2

            if(indicesHigh.size > 0):
                firstX = min(indicesHigh)
                lastX = max(indicesHigh)

                average = int((firstX+lastX)/2)
                self.error = average - center
                self.angularZ = -proportionalConstant*self.error
            else:
                continue

        # cv2.imshow("image", image)
        # cv2.waitKey(2)