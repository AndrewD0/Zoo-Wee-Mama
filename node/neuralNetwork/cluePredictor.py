#! /usr/bin/env python3

from tensorflow.python.keras.models import load_model
import cv2
import numpy as np

class cluePrediction:
    def __init__(self):
        self.img = []
        CNN_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/node/neuralNetwork/C_NN.h5"
        self.CNN = load_model(CNN_path)
        print("ready!!!!!")
        self.outputfile = open("/home/fizz/ros_ws/src/Zoo-Wee-Mama/predict.txt", "w")

    def predict(self, char_data): # one trimmed character at a time
        
        for img_array in char_data:

            # img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
            img_array = img_array.reshape(img_array.shape[0], img_array.shape[1], 1)
            self.img = np.expand_dims(img_array, axis=0)
            
            NN_prediction = self.CNN.predict(self.img)
            y_predict = (np.argmax(NN_prediction))  # Append the index of the max probability        
            
            labels = [chr(i) for i in range(65, 91)] + [str(i) for i in range(10)]
            predicted_character = labels[y_predict]  # Map the index to the corresponding character
            
            self.output_file.write(f"Predicted Character: {predicted_character}\n")

def main(good_chars):
    cnn = cluePrediction()

    char_data = good_chars
    for i in char_data:
        cnn.predict(i)

# if __name__ == '__main__':
#     main()