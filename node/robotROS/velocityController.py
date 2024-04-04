
import rospy
import numpy as np
from geometry_msgs.msg import Twist


class velocityController:
    
    def __init__(self):
        self.control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=3) # Velocity publisher

        # Velocities of the robot
        self.angularZ = 0
        self.linearX = 0.2

        self.error = 0
    
    def velocityPublish(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move
    

    def lineFollower(self, image):
        # Variables
        proportionalConstant = 0.02

        cutoffFrame = 0.999999999
        height, width = image.shape

        roiHeight = int(cutoffFrame*height)
        croppedFrame = image[roiHeight:height, :]
        center = croppedFrame.shape[1] // 2

        indices_high = np.where(croppedFrame > 0)

        if(indices_high[1].size > 0):
            first_x = min(indices_high[1])
            last_x = max(indices_high[1])
            average = int((first_x+last_x)/2)

            self.error = average - center
            
            self.angularZ = -proportionalConstant*self.error

        
        self.velocityPublish(self.linearX, self.angularZ)