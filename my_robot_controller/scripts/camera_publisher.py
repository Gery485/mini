#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('my_node')

cam = cv2.VideoCapture(0)
print(cam.isOpened())

bridge = CvBridge()
pub = rospy.Publisher('/webcam', Image, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    ret, frame = cam.read()
    if not ret:
        print('Sorry, but there is no camera :(')
        break

    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub.publish(msg)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()