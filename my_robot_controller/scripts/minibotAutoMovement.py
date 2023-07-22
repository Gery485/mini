#!/usr/bin/env python3

#imports
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import random

#variables
zero = 0

back = -0.5

slowSpeed = 0.1
normalSpeed = 0.3
fastSpeed = 0.5

turnRight = -2.0
turnLeft = 1.0
turnRightt = 2

listeNegativ = [-1, -0.75, -0.5, -0.25]
listePositiv = [0.25, 0.5, 0.75, 1]
liste = [-100 , 100]

# Definitions for the publisher
rospy.init_node('msgsForMoving')
cmd_vel_pub = rospy.Publisher('/minibot/rvr/cmd_vel', Twist, queue_size = 1)
cmd_vel_msg = Twist()
bridge = CvBridge()
depth_image = None

#converts the image to a CV2 image format and received subscriber node
def depth_image_callback(msg):
    global depth_image
    print('Image received')
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except Exception as e:
        print(e)

#returns the value of the distanca between the camera and the closest obstacle in meters
#returns inf, if there is no received image and filters
def calculate_distance_to_obstacle(depth_image):
    if depth_image is None:
        raise ValueError("No depth image provided")
    
    center_x = depth_image.shape[1] // 2
    depth_values = depth_image[:, center_x]
    valid_depth_values = depth_values[depth_values > 0]

    if len(valid_depth_values) > 0:
        closest_depth = min(valid_depth_values)
        return closest_depth * 0.001  # Convert to meters
    else:
        return None  # Return None when no valid depth values are found


#main-method
def main():
    global depth_image

# Subscriber
    rospy.Subscriber('/minibot/camera/depth/image_rect_raw', Image, depth_image_callback)
    rospy.wait_for_message('/minibot/camera/depth/image_rect_raw', Image)

# Set the rate
    rate = rospy.Rate(10)

#Image and settings for the movement
    while not rospy.is_shutdown():
        #cv2.imshow("Depth", depth_image / 2)
        #value = cv2.waitKey(1)

        distance = calculate_distance_to_obstacle(depth_image)
        if distance is not None:
            print(f"Distance to obstacle: {distance} meters")
        else:
           print("Unable to calculate the distance to the obstacle.")

# Moving of the robot based on the calculated distance
       # if distance < 0.230:
       #     cmd_vel_msg.linear.x = zero
       #     cmd_vel_msg.angular.z = turnLeft
       #     rospy.sleep(0.345)

       # elif distance < 0.190:
       #     cmd_vel_msg.linear.x = back
       #     cmd_vel_msg.angular.z = zero

       # else:
       #     numNum = random.choice(liste)
       #     if numNum < 0:
       #        numNeg = random.choice(listeNegativ)
               
       #        cmd_vel_msg.linear.x = -0.15
       #        cmd_vel_msg.angular.z = numNeg
           # else:
           #     numPos = random.choice(listePositiv)
           #     cmd_vel_msg.linear.x = -0.15
           #     cmd_vel_msg.angular.z = numPos
        
# Publish the message
        #cmd_vel_pub.publish(cmd_vel_msg)
        #rate.sleep()

if __name__ == '__main__':
    main()