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

    cv2.imshow("cropped", croppedFrame)
    cv2.imshow("diff", frameDifference)
    cv2.imshow("binary", binaryDifference)
    cv2.waitKey(2)

    if(np.any(croppedFrame == 255)):
        return True
    else:
        return False
