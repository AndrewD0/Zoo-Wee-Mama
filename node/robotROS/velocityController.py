
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class velocityController:
    
    def __init__(self):
        self.control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=3) # Velocity publisher
        self.msg = rospy.Subscriber('Output_topic', String, queue_size=10)

        # Velocities of the robot
        self.angularZ = 0
        self.linearX = 0.5

        self.averageCentroid = (0,0)
        self.proportionalConstant = 0.035
        self.error = 0
        self.bias = 70
    
    def velocityPublish(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move
    
    def lineFollower(self, image, frame):
        height,width = image.shape[:2]
        centerX = width//2
        centerY = height//2

        # We can make this a function!

        lineContours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        filteredContours = []

        for contour in lineContours:
            area = cv2.contourArea(contour)
            # print(area)

            if area >= 2500:
                filteredContours.append(contour)
        
        filteredContours = sorted(filteredContours, key=cv2.contourArea, reverse=True)

        newContours = []

        for contour in filteredContours:
            for point in contour[:,0]:
                x,y = point
                if x == 0 or x == width - 1 or y == height - 1:
                    newContours.append(contour)
                    break
        
        newContours = sorted(newContours, key = lambda c: cv2.boundingRect(c)[1])
        finalContours = []

        for contour in newContours:
            x,y,_,_ = cv2.boundingRect(contour)
            distance = y
            if distance < 480:
                finalContours.append(contour)
        
        finalContours = sorted(finalContours, key = lambda c: cv2.boundingRect(c)[1])
        cv2.drawContours(frame, finalContours, -1, (0,255,0), 3)

        print(len(finalContours))

        if(len(finalContours) >= 2):

            momentOne = cv2.moments(finalContours[0])
            momentTwo = cv2.moments(finalContours[1])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = int(momentTwo["m10"]/momentTwo["m00"])
            centroidY2 = int(momentTwo["m01"]/momentTwo["m00"])
            
            self.averageCentroid = (int((centroidX1+centroidX2)/2-self.bias),int((centroidY1+centroidY2)/2))
            self.error = centerX - self.averageCentroid[0]
                
        elif(len(finalContours) == 1):
            momentOne = cv2.moments(finalContours[0])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = 0
            centroidY2 = 0

            self.averageCentroid = (centroidX1, centroidY1)
            self.error = self.averageCentroid[0] - centerX

        
        self.angularZ = self.proportionalConstant*self.error

        print(self.angularZ)

        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.imshow("frame", frame)
        cv2.waitKey(2)
        
        self.velocityPublish(self.linearX, self.angularZ)

    def roundaboutFollower(self, image):
        # Variables

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
                self.angularZ = -self.proportionalConstant*self.error
            else:
                continue

        self.velocityPublish(self.linearX, self.angularZ)


        # cv2.imshow("image", image)
        # cv2.waitKey(2)
    
    def setLinearX(self, linearX):
        linearX = self.linearX
    
    def getLinearX(self):
        return self.linearX
    
    def setProportionalConstant(self, proportionalConstant):
        proportionalConstant = self.proportionalConstant
    
    def getProportionalConstant(self):
        return self.proportionalConstant
    
    def setBias(self, bias):
        self.bias = bias
    
    def getBias(self):
        return self.bias
        
     