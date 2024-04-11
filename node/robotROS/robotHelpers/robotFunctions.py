import cv2
import numpy as np


def pedestrianCrossed(frame, previousFrame):
    kernelSize = 3
    kernel = np.ones((kernelSize, kernelSize), np.uint8)

    frameDifference = cv2.absdiff(frame, previousFrame)
    frameGray = cv2.cvtColor(frameDifference, cv2.COLOR_BGR2GRAY)
    _, binaryDifference = cv2.threshold(frameGray, 100, 255, cv2.THRESH_BINARY)
    binaryDifference = cv2.dilate(binaryDifference, kernel, iterations = 3)

    croppedFrame = binaryDifference[350:450, 680:720]

    # cv2.imshow("cropped", croppedFrame)
    # cv2.imshow("diff", frameDifference)
    # cv2.imshow("binary", binaryDifference)
    # cv2.waitKey(2)

    if(np.any(croppedFrame == 255)):
        return True
    else:
        return False

def pedestrianEnd(redImage, pedestrianReached):
        cutoffFrame = 0.9999999
        height, width = redImage.shape
        roiHeight = int(cutoffFrame*height)

        croppedRed = redImage[roiHeight:height, :]
    
        redHigh = np.where(croppedRed > 0)

        if(redHigh[1].size > 0 and pedestrianReached == True):
            return True
        else:
             return False

def findLineContours(mask, state):

    height,width = mask.shape[:2]

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    filteredContours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area >= 4000:
            filteredContours.append(contour)
        
    filteredContours = sorted(filteredContours, key=cv2.contourArea, reverse=True)

    lineContours = []

    for contour in filteredContours:
        for point in contour[:,0]:
            x,y = point
            if x == 0 or x == width - 1 or y == height - 1:
                lineContours.append(contour)
                break
        
    lineContours = sorted(lineContours, key = lambda c: cv2.boundingRect(c)[1])


    grassContours = []

    for contour in lineContours:
        x,y,w,h = cv2.boundingRect(contour)
        distanceTop = y
        area = cv2.contourArea(contour)
        extent = float((area)/(w*h))

        #print("DistanceTop: %d Extent: %4f Base: %d Height: %d" % (distanceTop, extent, w, h))
        if distanceTop < 480:
            grassContours.append(contour)

    finalContours = []

    if(state == 'ROAD'):
        finalContours = lineContours
    elif(state == 'GRASS'):
        finalContours = grassContours
        
    finalContours = sorted(finalContours, key = lambda c: cv2.boundingRect(c)[1])
    return finalContours
     
     
def findGrassContours(mask):

    height,width = mask.shape[:2]

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    filteredContours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area >= 3000:
            filteredContours.append(contour)
        
    filteredContours = sorted(filteredContours, key=cv2.contourArea, reverse=True)

    filteredMask = np.zeros_like(mask)
    cv2.drawContours(filteredMask, filteredContours, -1 ,(255,255,255), thickness= cv2.FILLED)
    #cv2.imshow("filtered", filteredMask)

    lineContours = []

    for contour in filteredContours:
        for point in contour[:,0]:
            x,y = point
            if x == 0 or x == width - 1 or y == height - 1:
                lineContours.append(contour)
                break
        
    lineContours = sorted(lineContours, key = lambda c: cv2.boundingRect(c)[1])

    lineMask = np.zeros_like(mask)
    cv2.drawContours(lineMask, lineContours, -1 ,(255,255,255), thickness= cv2.FILLED)
    #cv2.imshow("LineMask", lineMask)
    cv2.waitKey(2)


    grassContours = []
    print("LineContours: %d" % len(lineContours))

    for contour in lineContours:
        x,y,w,h = cv2.boundingRect(contour)
        distanceTop = y
        area = cv2.contourArea(contour)
        extent = float((area)/(w*h))

        #print("DistanceTop: %d Extent: %4f Base: %d Height: %d" % (distanceTop, extent, w, h))
        if distanceTop < 460:
            grassContours.append(contour)
        
    grassContours = sorted(grassContours, key = lambda c: cv2.boundingRect(c)[1])
    return grassContours

def getValue(frame):
    x,y,w,h= 0, 400, 1280, 100
    sky_blue_low = np.array([70, 50, 50])
    sky_blue_up = np.array([150, 255, 255])

    skyRemoved = cv2.inRange(frame, sky_blue_low, sky_blue_up)

    frameInterest = frame[y:720, x:x+w]

    valueChannels = frameInterest[:,:,2]

    averageValue = np.mean(valueChannels)

    print("Average: %d" % averageValue)

    #cv2.imshow("roi",frameInterest)
    #cv2.imshow("frames", skyRemoved)
    cv2.waitKey(2)

    return averageValue

def brightenFrame(frame):
    groundValue = 170
    comparingValue = getValue(frame)

    factor = float(groundValue/comparingValue)

    frame[:,:,2]= np.clip(frame[:,:,2]*factor,0,255)

    return frame

def findMountainContours(mask):

    height,width = mask.shape[:2]
    
    mask = cv2.GaussianBlur(mask, (1,1), 0)
    mask = cv2.dilate(mask, (5,5), iterations = 3)
    maxHeight = 360
    bottomHeight = 60
    
    roiFrame = mask[height-maxHeight:height-bottomHeight, 0: width//2]

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    filteredContours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area >= 3000:
            filteredContours.append(contour)
        
    finalContours = []

    for contour in filteredContours:
        x,_,_,_ = cv2.boundingRect(contour)
        distance = x
        if (distance < 600):
            finalContours.append(contour)
        
    finalContours = sorted(finalContours, key=cv2.contourArea, reverse=True)
    finalMask = np.zeros_like(mask)
    cv2.drawContours(finalMask, finalContours, -1 ,(255,255,255), thickness= cv2.FILLED)
    cv2.imshow("filtered mask", finalMask)
    cv2.waitKey(2)

    return finalContours

    
