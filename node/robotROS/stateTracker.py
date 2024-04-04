import rospy
import numpy as np

class stateTracker:

    def __init__(self):
        self.robotDictionary = {0:'ROAD', 1:'PEDESTRIAN', 2:'ROUNDABOUT', 3:'GRASS', 4:'YODA', 5:'TUNNEL'}
        self.robotState = self.robotDictionary[0]

        self.markersCounter = 0
        self.cluesCounter = 0


    def findState(self, pinkImage, redImage):
        cutoffFrame = 0.9999999
        height, width = redImage.shape
        roiHeight = int(cutoffFrame*height)

        croppedPink = pinkImage[roiHeight:height, :]
        croppedRed = redImage[roiHeight:height, :]

        #pinkHigh = 




    
    def getState(self):
        return self.robotState
    
    def setState(self, state):
        if(state in self.robotDictionary.values()):
            self.robotState = state
        else:
            print("Invalid state!")
    
    def getMarkersCounter(self):
        return self.markersCounter
    
    def getCluesCounter(self):
        return self.cluesCounter
