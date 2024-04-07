#! /usr/bin/env python3

from tensorflow.python.keras.models import load_model
import cv2
import numpy as np
from clueDetector import clue_Detector

class cluePrediction:
    def __init__(self):
        self.img = []
        CNN_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/node/neuralNetwork/C_NN.h5"
        self.CNN = load_model(CNN_path)
        self.image_read()

    def image_read(self):
        all_data = clue_Detector.getData()

        for img_array in all_data:
            img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
            img_array = img_array.reshape(img_array.shape[0], img_array.shape[1], 1)
            all_data.append([img_array])
        self.img = np.expand_dims([data for data in all_data], axis=0)

    def predict(self):
        y_predict = []

        NN_prediction = self.CNN.predict(self.img[0])
        for i in NN_prediction:
            max = np.max(i.tolist())
            index = np.where(i == max)[0][0]
            y_predict.append(index)
        
        labels = [chr(i) for i in range(65, 91)] + [str(i) for i in range(10)]
        mapped_results = [labels[pred] for pred in y_predict]

        print("Mapped Predictions:")
        for i, label in enumerate( mapped_results):
            print(f"Image {i + 1}: Predicted Label{label}")

def main():
    cnn = cluePrediction()
    cnn.predict()

if __name__ == '__main__':
    main()