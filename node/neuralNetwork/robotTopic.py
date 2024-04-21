#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from clueDetector import clue_Detector


if __name__ == '__main__':
    rospy.init_node('MsgPublish', anonymous=True)
    pub = rospy.Publisher("clue_topic", String, queue_size=10)
    rate = rospy.Rate(10)
    clueBoard = clue_Detector()
    
    while not rospy.is_shutdown():
        board_count = clueBoard.getBoardCount()
        msg = String(data=str(board_count))
        pub.publish(msg)
        rate.sleep()
