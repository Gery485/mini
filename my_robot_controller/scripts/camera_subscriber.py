#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# Load the pre-trained face detector (Haar cascade)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')

    # Convert the image to grayscale for face detection
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30))

    # Draw rectangles around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 180, 190), 3)
        middle_x = int(w/2) - 30
        cv2.putText(cv_image, 'Face', (x + middle_x, y - 10), cv2.FONT_HERSHEY_COMPLEX, 0.9, (248, 152, 128), 2)

    # Display the image with face detection
    cv2.imshow('/webcam', cv_image)
    cv2.waitKey(1)

rospy.init_node('image_subscriber_node')
rospy.Subscriber('/webcam', Image, image_callback)
rospy.spin()
