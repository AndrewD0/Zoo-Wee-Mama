import cv2
import numpy as np

# Thresholds

# Detecting white lines
LOWER_WHITE = np.array([235,235,235])
UPPER_WHITE = np.array([255,255,255])

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

IMG_STYLE = 'bgr8'
