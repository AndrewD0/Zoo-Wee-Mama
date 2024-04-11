#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String

class stateTracker:

    def __init__(self):
        self.msg = rospy.Subscriber('Output_topic', String, self.msg_callback, queue_size=10)
        self.robotDictionary = {0:'ROAD', 1:'PEDESTRIAN', 2:'ROUNDABOUT', 3:'GRASS', 4:'YODA', 5:'TUNNEL'}
        self.robotState = self.robotDictionary[0]

        self.markersCounter = 0
        self.cluesCounter = 0
        self.pedestrianReached = False
        self.startedRoundabout = False
        self.pinkReached = False

    def msg_callback(self, data):
        self.cluesCounter = int(data.data)
        print("Board Count:" , self.cluesCounter)

    def findState(self, pinkImage, redImage):

        # Check if we have reached the pink.
        if np.any(pinkImage[-1,:]) > 0:
            self.pinkReached = True
        
        # Check if we have crossed the pink line.
        if self.pinkReached == True:
            if np.any(pinkImage[-1,:]) == 0 and self.getState() == 'ROAD':
                self.setState('GRASS')

        if( np.any(redImage[-1,:]) > 0 and self.pedestrianReached == False):
            self.setState('PEDESTRIAN')
            self.pedestrianReached = True
            

    def getState(self):
        return self.robotState
    
    def setState(self, state):
        if(state in self.robotDictionary.values()):
            self.robotState = state
        else:
            print("Invalid state!")
    
    def getMarkersCounter(self):
        return self.markersCounter
    
    def getPedestrianReached(self):
        return self.pedestrianReached
    
    def getCluesCounter(self):
        return self.cluesCounter
    
    def setCluesCounter(self, data):
        self.cluesCounter = data

    def startRoundabout(self, boolean):
        self.startedRoundabout = boolean
    
    def checkRoundabout(self):
        return self.startedRoundabout

    def setPinkReached(self, boolean):
        self.pinkReached = boolean
    
    def getPinkReached(self):
        return self.pinkReached
