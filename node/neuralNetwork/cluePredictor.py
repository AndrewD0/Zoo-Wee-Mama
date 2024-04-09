#! /usr/bin/env python3

from tensorflow.python.keras.models import load_model
import cv2
import numpy as np
import os
from clueDetector import clue_Detector

class cluePrediction:
    def __init__(self):
        self.img = []
        CNN_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/node/neuralNetwork/C_NN.h5"
        self.CNN = load_model(CNN_path)

    def predict(self, img_array): # one trimmed character at a time

        img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
        img_array = img_array.reshape(img_array.shape[0], img_array.shape[1], 1)
        self.img = np.expand_dims(img_array, axis=0)
        
        NN_prediction = self.CNN.predict(self.img)
        y_predict = (np.argmax(NN_prediction))  # Append the index of the max probability        
        
        labels = [chr(i) for i in range(65, 91)] + [str(i) for i in range(10)]
        predicted_character = labels[y_predict]  # Map the index to the corresponding character
        
        print(f"Predicted Character: {predicted_character}")

def main():
    cnn = cluePrediction()
    char_folder = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/Characters/"
    all_data = []

    for img_file in os.listdir(char_folder):
        img_array = cv2.imread(os.path.join(char_folder, img_file))
        print("Predicted File     : " + img_file[0])
        cnn.predict(img_array)

if __name__ == '__main__':
    main()