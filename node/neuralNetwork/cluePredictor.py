#! /usr/bin/env python3

from tensorflow.python.keras.models import load_model
from std_msgs.msg import String
import numpy as np
import rospy


class cluePrediction:
    def __init__(self):
        self.img = []
        CNN_path = "/home/fizzer/ros_ws/src/Zoo-Wee-Mama/node/neuralNetwork/C_NN.h5"
        self.CNN = load_model(CNN_path)
        print("ready!!!!!")
        self.output_file = open("/home/fizzer/ros_ws/src/Zoo-Wee-Mama/predict.txt", "w")
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)

    def predict(self, char_data): # one trimmed character at a time
        clue_list = []
        for i in char_data:
            result = []
            
            for img_array in i:

                # img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
                img_array = img_array.reshape(img_array.shape[0], img_array.shape[1], 1)
                self.img = np.expand_dims(img_array, axis=0)
                
                NN_prediction = self.CNN.predict(self.img)
                y_predict = (np.argmax(NN_prediction))  # Append the index of the max probability        
                
                labels = [chr(i) for i in range(65, 91)] + [str(i) for i in range(10)]
                predicted_character = labels[y_predict]  # Map the index to the corresponding character
                
                result.append(predicted_character)
            
            string = ''.join(result)
            clue_list.append(string)
            
                
            self.output_file.write(f"Predicted Character: {string}\n")
        self.msg(clue_list)
        rospy.sleep(0.1)

    def msg(self, clue_list):
        ID = -2
        value = "WRONG!!!"
        clue_dict = {'SIZE': '1', 'VICTIM': '2','CRIME': '3', 'TIME': '4', 'PLACE': '5', 'MOTIVE': '6', 'WEAPON': '7', 'BANDIT': '8'}
        pred_dict = {clue_list[0]:clue_list[1]}

        if bool (pred_dict):
            key = next(iter(pred_dict))
            if key in clue_dict:
                ID = clue_dict[key]
                value = pred_dict[key]

            msg = f'ZoWeMama,lisndrew,{ID},{value}'

            self.scoretracker.publish(msg)
            pred_dict.clear()



def main(good_chars):
    cnn = cluePrediction()

    char_data = good_chars
    for i in char_data:
        cnn.predict(i)

# if __name__ == '__main__':
#     main()