#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rtabmap_msgs.msg import OdomInfo
import cv2
import random
import time

# Moving
zero = 0
move = 0.3
turn = 1.5

failure = False

# Waiting
sleep = 0.5
sleep1 = 1

# Distances in cm
minDistance = 120

number = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5]

# Definitions for the publisher
rospy.init_node('msgsForMoving')
cmd_vel_pub = rospy.Publisher('/minibot/rvr/cmd_vel', Twist, queue_size = 1)
cmd_vel_msg = Twist()
bridge = CvBridge()
depth_image = None
camera_info = None

# Converts the image to a CV2 image format and receives subscriber node
def depth_image_callback(msg):
    global depth_image
    #print('Image received')
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except Exception as e:
        print(e)

def camera_info_callback(msg):
    global camera_info
    camera_info = msg

# Returns the value of the distance between the camera and the closest obstacle in meters
# Returns inf, if there is no received image and filters
def calculate_distance_to_obstacle(depth_image, camera_info):
    if depth_image is None or camera_info is None:
        raise ValueError("No depth image or camera info provided")

    # Camera intrinsic parameters
    fx = camera_info.K[0]  # focal length in x-direction
    fy = camera_info.K[4]  # focal length in y-direction
    cx = camera_info.K[2]  # principal point x-coordinate
    cy = camera_info.K[5]  # principal point y-coordinate

    center_x = depth_image.shape[1] // 2
    center_y = depth_image.shape[0] // 2

    # Get the depth value at the center of the image
    depth_value = depth_image[center_y, center_x]

    if depth_value > 0:
        # Convert depth value to meters using the depth image unit (usually meters)
        depth_in_meters = depth_value
        return depth_in_meters
    else:
        return None  # Return None when no valid depth value is found

# Callback function for the /minibot/rtabmap/odom_info topic
def odom_info_callback(msg):
    global failure

    print(msg.lost)

    if msg.lost:
        rospy.loginfo("Received True on /minibot/rtabmap/odom_info topic.")
        failure = True


# Main method
def main():
    global depth_image, camera_info, failure

    failure = False
    # Subscriber for depth image, camera info, and odom_info
    rospy.Subscriber('/minibot/camera/depth/image_rect_raw', Image, depth_image_callback)
    rospy.Subscriber('/minibot/camera/depth/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber("minibot/rtabmap/odom_info", OdomInfo, odom_info_callback, queue_size=10)
    rospy.wait_for_message('/minibot/camera/depth/image_rect_raw', Image)
    rospy.wait_for_message('/minibot/camera/depth/camera_info', CameraInfo)

    # Set the rate
    rate = rospy.Rate(10)

    # Image and settings for the movement
    while not rospy.is_shutdown():
        # Calculate the distance to the obstacle
        distance = calculate_distance_to_obstacle(depth_image, camera_info)
        if distance is not None:
            distance_cm = distance / 10  # Convert distance to centimeters
            print(f"Distance to obstacle: {distance_cm:.2f} cm")


            if failure == True:
                failure = False
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(1.0)

            elif distance_cm <= minDistance:
                choice = random.choice(number)
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(choice)
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(choice)

            else:
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                rospy.sleep(sleep)
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn

                cmd_vel_msg.linear.x = move
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)

                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep1)

                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn * -1
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep1)

                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn * -1
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep1)

                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep1)
        else:
            print("Unable to calculate the distance to the obstacle.")
            cmd_vel_msg.linear.x = zero
            cmd_vel_msg.angular.z = turn
            cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(sleep)
            continue

        rate.sleep()

if __name__ == '__main__':
    main()
