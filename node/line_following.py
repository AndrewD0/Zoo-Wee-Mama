#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
import numpy as np
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class Imag_Convert:

    def __init__(self):
        self.image = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.hsv_callback, queue_size=3) # Image subscriber
        self.clock = rospy.Subscriber('/clock', Clock, self.clock_callback, queue_size=10)
        self.control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=3) # Velocity subscriber
        self.scoretracker = rospy.Publisher('/score_tracker', String, queue_size=10)
        
        self.bridge = CvBridge() # CvBridge initialization

        self.position = 0 # Last position initialization
    
    def clock_callback(self, data):
        # Start timer
        start_time = 0
        duration = 5
        msg_start = 'ZoWeMama,lisndrew,0,START'
        msg_stop = 'ZoWeMama,lisndrew,-1,STOP'

        if (rospy.get_time() == start_time):
            self.scoretracker.publish(msg_start)
        elif (rospy.get_time() == duration):
            self.scoretracker.publish(msg_stop)

    def callback(self, data):

        # variables
        binary_low = (235,235,235) # threshold to be compared with
        binary_high =(255,255,255) #set to if > thresh
        img_style = 'bgr8'
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Convert video to grayscale
        binary = cv2.inRange(frame, binary_low,binary_high) # Convert to binary
        
        cv2.imshow("image",binary)
        linear_vel, angular_vel = self.frame_analysis(binary, frame)  # Calculate linear and angular velocity
        self.vel_control(linear_vel, angular_vel) # Publish velocities

        cv2.waitKey(2)

    def hsv_callback(self, data):
        #variables
        lower_gray = np.array([0, 0, 0])
        upper_gray = np.array([128, 128, 128])

        lower_soil = np.array([184, 134, 11])
        upper_soil = np.array([143, 188, 143])
        img_style = 'bgr8'
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, img_style) #Convert ROS images to OpenCV images
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.inRange(hsv, lower_gray, upper_gray)
        soil = cv2.inRange(hsv, lower_soil, upper_soil)


        # cv2.imshow("image",gray)
        linear_vel, angular_vel = self.frame_analysis(gray, frame)  # Calculate linear and angular velocity
        self.vel_control(linear_vel, angular_vel) # Publish velocities

        # cv2.waitKey(2)


    def frame_analysis(self, binary, frame):

        #variables
        last_line = binary[715] # last line of the frame in pixel
        width= frame.shape[1] 
        length = len(last_line)
        road = 255 # look for white

        angularZ = 0
        linearX = 0.2
        
        height, width, _ = frame.shape
        roi_height = int(height * 0.99999) 

        cropped_mask = binary[roi_height:height, :]

        center = cropped_mask.shape[1] // 2

        indices_high = np.where(cropped_mask > 0)

        if(indices_high[1].size > 0):
            first_x = min(indices_high[1])
            last_x = max(indices_high[1])
            average = int((first_x+last_x)/2)

            error = average - center
            
            angularZ = -0.02*error
        
        return linearX, angularZ
        
    def vel_control(self, linear, angular):
        move = Twist()
        move.linear.x = linear # Forward velocity setup
        move.angular.z = angular # Angular velocity setup
        self.control.publish(move) # Publish move

def spawn_position(self, position):
    msg = ModelState()
    msg.model_name = 'R1'

    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.x = position[3]
    msg.pose.orientation.y = position[4]
    msg.pose.orientation.z = position[5]
    msg.pose.orientation.w = position[6]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( msg )

    except rospy.ServiceException:
        print ("Service call failed")



def main():

    image_processor = Imag_Convert()
    rospy.init_node('image_convert_node', anonymous=True) # Initialize node
    # rospy.sleep(1) 

    position = [5.5, 5, 0.2, 0, 0, 1.57, 0] # "-x 5.5 -y 2.5 -z 0.2 -R 0.0 -P 0.0 -Y -1.57"
    # spawn_position(position)

    try:   
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!!")

if __name__ == '__main__':
    main()