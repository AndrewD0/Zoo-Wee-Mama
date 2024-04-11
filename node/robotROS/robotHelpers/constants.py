import cv2
import numpy as np

# Thresholds

# Detecting white lines
LOWER_WHITE = 230
UPPER_WHITE = 255

# Detecting the road in HSV
LOWER_ROAD = np.array([0, 0, 0])
UPPER_ROAD = np.array([128, 128, 128])

# Detecting the soil section
LOWER_SOIL = np.array([10, 35, 178]) #[184, 134, 11] #mean grass: 27, 60, 205
UPPER_SOIL = np.array([80, 255, 255]) #[143, 188, 143] # mean red: 5, 220, 201
    
# Detecting red
LOWER_RED = np.array([0,0,235])
UPPER_RED = np.array([20,20,255])

#Detecting pink
LOWER_PINK = np.array([200, 0, 200])
UPPER_PINK = np.array([255, 30, 255])

LOWER_TUNNEL = np.array([6, 50, 50])
UPPER_TUNNEL = np.array([10, 200, 255])

LOWER_BLUE = np.array([85, 50, 80])
UPPER_BLUE = np.array([130, 255, 255])

LOWER_SKY = np.array([100, 45, 110])
UPPER_SKY = np.array([115, 125, 235])

IMG_STYLE = 'bgr8'
