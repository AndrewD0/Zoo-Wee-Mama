
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robotHelpers import robotFunctions

class velocityController:
    
    def __init__(self):
        self.control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1) # Velocity publisher
        self.msg = rospy.Subscriber('Output_topic', String, queue_size=10)

        # Velocities of the robot
        self.angularZ = 0
        self.linearX = 0.4

        self.averageCentroid = (0,0)
        self.proportionalConstant = 0.03
        self.error = 0
        self.bias = 0
        self.roundaboutStart = 0
    
    def velocityPublish(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move
    
    def lineFollower(self, mask, frame, state):

        height,width = mask.shape[:2]
        centerX = width//2
        centerY = height//2

        # We can make this a function!

        finalContours = robotFunctions.findLineContours(mask, state)

        print("Final contours: %d" % len(finalContours))


        if(len(finalContours) >= 2):

            momentOne = cv2.moments(finalContours[0])
            momentTwo = cv2.moments(finalContours[1])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = int(momentTwo["m10"]/momentTwo["m00"])
            centroidY2 = int(momentTwo["m01"]/momentTwo["m00"])
            
            self.averageCentroid = (int((centroidX1+centroidX2)/2-self.bias),int((centroidY1+centroidY2)/2))
            self.error = centerX - self.averageCentroid[0]

            cv2.drawContours(frame, finalContours, 0, (0,255,0), 3)
            cv2.drawContours(frame, finalContours, 1, (0,255,0), 3)
                
        elif(len(finalContours) == 1):
            momentOne = cv2.moments(finalContours[0])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = 0
            centroidY2 = 0

            self.averageCentroid = (centroidX1, centroidY1)
            self.error = self.averageCentroid[0] - centerX

            cv2.drawContours(frame, finalContours, 0, (0,255,0), 3)
        else:
            centroidX1 = 0
            centroidX2 = 0
            centroidY1 = 0
            centroidY2 = 0

        
        self.angularZ = self.proportionalConstant*self.error

        print("AngularZ: %d" % self.angularZ)


        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.circle(frame,(centroidX1, centroidY1), 3, (0,0,255),3)
        cv2.circle(frame, (centroidX2,centroidY2), 3, (0,0,255),3)
        cv2.imshow("frame", frame)
        cv2.waitKey(2)
        
        self.velocityPublish(self.linearX, self.angularZ)

    def roundaboutFollower(self, image, frame):
        # Variables

        # self.linearX = 0.45
        height, width = image.shape
        cutoffFrame = 0.9999999

        roiHeight = int(height * cutoffFrame)

        croppedImage = image[roiHeight:height, :]
        center = croppedImage.shape[1]//2

        indicesHigh = np.where(croppedImage > 0)

        if(indicesHigh[1].size>0):
            print("TRUE")
            firstX = min(indicesHigh[1])
            lastX = max(indicesHigh[1])
            average = int((firstX+lastX)/2)

            print("First: %d Last: %d Average: %d" %(firstX, lastX, average))
    
            self.error = center-average
            self.angularZ = self.proportionalConstant*self.error
            # if(self.roundaboutStart <= 8):
            #     self.angularZ = -self.proportionalConstant*self.error
            #     self.roundaboutStart +=1
        
        print("Error: %d AngularZ: %d" % (self.error, self.angularZ))
        # cv2.imshow("round", image)
        # cv2.waitKey(2)

        self.velocityPublish(self.linearX, self.angularZ)

    def yodaFollower(self, tunnelHighlight, boardHighlight, pinkHighlight, frame):
        height, width = tunnelHighlight.shape[:2]
        centerX = width // 2
        centerY = height // 2

        tunnel_contours, _ = cv2.findContours(tunnelHighlight, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        board_contours, _ = cv2.findContours(boardHighlight, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pink_contours, _ = cv2.findContours(pinkHighlight, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(pink_contours) > 0:
            sorted_pink = sorted(pink_contours, key=cv2.contourArea, reverse=True)
            moment_pink = cv2.moments(sorted_pink[0])
            pink_X = int(moment_pink["m10"]/moment_pink["m00"])
            pink_Y = int(moment_pink["m01"]/moment_pink["m00"])

            self.averageCentroid = (pink_X, pink_Y)

        elif len(board_contours) > 0:
            sorted_board = sorted(board_contours, key=cv2.contourArea, reverse=True)
            moment_board = cv2.moments(sorted_board[0])
            board_X = int(moment_board["m10"]/moment_board["m00"])
            board_Y = int(moment_board["m01"]/moment_board["m00"])

            self.averageCentroid = (board_X, board_Y)

        elif len(tunnel_contours) > 0:
            sorted_tunnel = sorted(tunnel_contours, key=cv2.contourArea, reverse=True)
            moment_tunnel = cv2.moments(sorted_tunnel[0])
            tunnel_X = int(moment_tunnel["m10"]/moment_tunnel["m00"])
            tunnel_Y = int(moment_tunnel["m01"]/moment_tunnel["m00"])
            
            self.averageCentroid = (tunnel_X, tunnel_Y)

        else:
           pass

        self.error = centerX - self.averageCentroid[0]

        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)

        cv2.imshow("yoda", frame)
        cv2.waitKey(2)

        self.angularZ = self.proportionalConstant * self.error

        self.velocityPublish(0.3, self.angularZ)
        
        pass

    def soilFollower(self, mask, frame):

        height,width = frame.shape[:2]
        centerX = width//2
        centerY = height//2

        # We can make this a function!

        finalContours = robotFunctions.findGrassContours(mask)

        print("Final contours: %d" % len(finalContours))


        if(len(finalContours) >= 2):

            momentOne = cv2.moments(finalContours[0])
            momentTwo = cv2.moments(finalContours[1])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = int(momentTwo["m10"]/momentTwo["m00"])
            centroidY2 = int(momentTwo["m01"]/momentTwo["m00"])
            
            self.averageCentroid = (int((centroidX1+centroidX2)/2-self.bias),int((centroidY1+centroidY2)/2))
            self.error = centerX - self.averageCentroid[0]

            cv2.drawContours(frame, finalContours, 0, (0,255,0), 3)
            cv2.drawContours(frame, finalContours, 1, (0,255,0), 3)
                
        elif(len(finalContours) == 1):
            momentOne = cv2.moments(finalContours[0])

            centroidX1 = int(momentOne["m10"]/momentOne["m00"])
            centroidY1 = int(momentOne["m01"]/momentOne["m00"])
            centroidX2 = 0
            centroidY2 = 0

            self.averageCentroid = (centroidX1, centroidY1)
            self.error = self.averageCentroid[0] - centerX

            cv2.drawContours(frame, finalContours, 0, (0,255,0), 3)
        else:
            centroidX1 = 0
            centroidX2 = 0
            centroidY1 = 0
            centroidY2 = 0

        
        self.angularZ = self.proportionalConstant*self.error

        print("AngularZ: %d" % self.angularZ)


        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.circle(frame,(centroidX1, centroidY1), 3, (0,0,255),3)
        cv2.circle(frame, (centroidX2,centroidY2), 3, (0,0,255),3)
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.waitKey(2)
        
        self.velocityPublish(self.linearX, self.angularZ)
    
    def mountainClimber(self, mask, frame):
        height,width = frame.shape[:2]
        centerX = width//2
        centerY = height//2
        threshold = 300

        # We can make this a function!

        finalContours = robotFunctions.findMountainContours(mask)

        print("Final contours: %d" % len(finalContours))


        if(len(finalContours) >= 1):

            moment = cv2.moments(finalContours[0])

            centroidX1 = int(moment["m10"]/moment["m00"])
            centroidY1 = int(moment["m01"]/moment["m00"])
            
            centroidX1 = int(moment["m10"]/moment["m00"])
            centroidY1 = int(moment["m01"]/moment["m00"])
            self.averageCentroid=(centroidX1,centroidY1)
            self.error = (centerX - self.averageCentroid[0])-threshold

            cv2.drawContours(frame, finalContours, 0, (0,255,0), 3)
        else:
            centroidX1 = 0
            centroidX2 = 0

        
        self.angularZ = self.proportionalConstant*self.error

        print("AngularZ: %d Error: %d" % (self.angularZ, self.error))


        cv2.circle(frame, (centerX,centerY), 3, (0,255,0),3)
        cv2.circle(frame, self.averageCentroid, 3, (255,0,0), 3)
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.waitKey(2)
        self.velocityPublish(self.linearX, self.angularZ)


    def setLinearX(self, linearX):
        self.linearX = linearX
    
    def getLinearX(self):
        return self.linearX
    
    def setAngularZ(self, angularZ):
        self.angularZ = angularZ
    
    def getAngularZ(self):
        return self.angularZ
    
    def setProportionalConstant(self, proportionalConstant):
        self.proportionalConstant = proportionalConstant
    
    def getProportionalConstant(self):
        return self.proportionalConstant
    
    def setError(self, error):
        self.error = error
    
    def getError(self):
        return self.error
    
    def setBias(self, bias):
        self.bias = bias
    
    def getBias(self):
        return self.bias
        
     