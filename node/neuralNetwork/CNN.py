#! /usr/bin/env python3

import tensorflow as tf
# from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model
import cv2
import os
import numpy as np
import pandas as pd

# global sess1
# global graph1

# sess1 = tf.Session()
# graph1 = tf.get_default_graph()
# set_session(sess1)


class Character_NN:
    def __init__(self):
        self.ygt = []
        self.img = []
        CNN_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/node/neuralNetwork/C_NN.h5"
        self.CNN = load_model(CNN_path)
        self.image_read()

    def image_read(self):
        char_folder = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/Character/"
        all_data = []

        for img_file in os.listdir(char_folder):
            img_array = cv2.imread(os.path.join(char_folder, img_file))
            img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
            img_array = cv2.resize(img_array, (120, 100))
            img_array = img_array.reshape(img_array.shape[0], img_array.shape[1], 1)
            # print(img_array.shape)
            label = img_file[0]
            self.ygt.append(label)
            all_data.append([img_array, label])
        self.img = np.expand_dims([data[0] for data in all_data], axis=0)
        # print(self.img.shape)

    def predict(self):
        # with graph1.as_default():
        #     set_session(sess1)
        y_predict = []

        NN_prediction = self.CNN.predict(self.img[0])
        for i in NN_prediction:
            max = np.max(i.tolist())
            index = np.where(i == max)[0][0]
            y_predict.append(index)
        
        # cm = confusion_matrix(self.ygt, y_predict).tolist()

        labels = [chr(i) for i in range(65, 91)] + [str(i) for i in range(10)]
        mapped_results = [labels[pred] for pred in y_predict]

        print("Mapped Predictions:")
        for i, (actual_label, predicted_label) in enumerate(zip(self.ygt, mapped_results)):
            print(f"Image {i + 1}: {actual_label} | {predicted_label}")

def main():
    cnn = Character_NN()
    cnn.predict()

if __name__ == '__main__':
    main()