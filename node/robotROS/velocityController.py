
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

        self.averageCentroid = 0

        self.error = 0

        self.bias = 70

    
    
    def velocityPublish(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move
    

    def lineFollower(self, image, frame):
        # Variables
        proportionalConstant = 0.032
        derivativeConstant = 0.1

        height,width = image.shape[:2]
        centerX = width//2
        centerY = height//2


        lineContours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        filteredContours = []

        for contour in lineContours:
            area = cv2.contourArea(contour)
            # print(area)

            if area >= 2500:
                filteredContours.append(contour)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        
        sortedContours = sorted(filteredContours, key=cv2.contourArea, reverse=True)

        if(len(sortedContours) >= 2):


            momentOne = cv2.moments(sortedContours[0])
            momentTwo = cv2.moments(sortedContours[1])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = int(momentTwo["m10"]/momentTwo["m00"])
            centroidY2 = int(momentTwo["m01"]/momentTwo["m00"])
        else:
            momentOne = cv2.moments(sortedContours[0])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = 0
            centroidY2 = 0

        if(centroidX1 == 0 and centroidY1 == 0):
            self.averageCentroid = (centroidX2, centroidY2)
        elif(centroidX2 == 0 and centroidY2 == 0):
            self.averageCentroid = (centroidX1, centroidY1)
        else:
            self.averageCentroid = (int((centroidX1+centroidX2)/2 - self.bias),int((centroidY1+centroidY2)/2))

        # Error

        if(centroidX1 == 0 and centroidY1 == 0):
            self.error = centroidX2 - centerX
            proportionalConstant = 0.02
        elif(centroidX2 == 0 and centroidY2 == 0):
            self.error = centroidX1-centerX
            proportionalConstant = 0.02
        else:
            self.error = centerX-self.averageCentroid[0]
        
        self.angularZ = proportionalConstant*self.error

        print(self.angularZ)

        #self.velocityPublish(self.linearX,self.angularZ)

        # cv2.imshow("image", image)

        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.waitKey(2)
        
        self.velocityPublish(self.linearX, self.angularZ)

    def roundaboutFollower(self, image):
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
    
    def soilFollower(self, image, frame):
        # Variables
        proportionalConstant = 0.025
        derivativeConstant = 0.1

        height,width = image.shape[:2]
        centerX = width//2
        centerY = height//2


        lineContours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        lineContours = sorted(lineContours,key=cv2.contourArea, reverse=True)

        filteredContours = []

        for contour in lineContours:
            area = cv2.contourArea(contour)

            if area >= 4000:
                filteredContours.append(contour)
        #print("BREAK")
        
        sortedContours = sorted(filteredContours, key=cv2.contourArea, reverse=True)
        #print(len(lineContours))
        #print(len(sortedContours))

        lengthContours = []

        for contour in sortedContours:
            for point in contour[:, 0]:
                x, y= point
                if x == 0 or x == width -1 or y == height - 1:
                    lengthContours.append(contour)
                    break
        
        #print(len(lengthContours)) 
        lengthContours = sorted(lengthContours, key=lambda c: cv2.boundingRect(c)[1])
        # print(len(lengthContours))

        newContours = []    
        for contour in lengthContours:
            x,y,_,_= cv2.boundingRect(contour)
            
            distance = y
            if distance < 480:
                newContours.append(contour)

        
        newContours = sorted(newContours,key=lambda c: cv2.boundingRect(c)[1])
                    

        if(len(newContours) >= 2):


            momentOne = cv2.moments(newContours[0])
            momentTwo = cv2.moments(newContours[1])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = int(momentTwo["m10"]/momentTwo["m00"])
            centroidY2 = int(momentTwo["m01"]/momentTwo["m00"])

            cv2.drawContours(frame, newContours, 0, (0, 255, 0), 2)
            cv2.drawContours(frame, newContours, 1, (0, 255, 0), 2)
        else:
            momentOne = cv2.moments(newContours[0])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = 0
            centroidY2 = 0
            cv2.drawContours(frame, newContours, 0, (0, 255, 0), 2)

        if(centroidX1 == 0 and centroidY1 == 0):
            self.averageCentroid = (centroidX2, centroidY2)
        elif(centroidX2 == 0 and centroidY2 == 0):
            self.averageCentroid = (centroidX1, centroidY1)
        else:
            self.averageCentroid = (int((centroidX1+centroidX2)/2),int((centroidY1+centroidY2)/2))

        # Error

        if(centroidX1 == 0 and centroidY1 == 0):
            self.error = centroidX2 - centerX
            proportionalConstant = 0.02
        elif(centroidX2 == 0 and centroidY2 == 0):
            self.error = centroidX1-centerX
            proportionalConstant = 0.02
        else:
            self.error = centerX-self.averageCentroid[0]
        
        self.angularZ = proportionalConstant*self.error

        #print(self.angularZ)

        self.velocityPublish(self.linearX,self.angularZ)

        cv2.imshow("image", image)

        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.waitKey(2)
        
     