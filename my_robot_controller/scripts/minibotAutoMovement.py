#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import random

# Moving
zero = 0
move = 0.3
turn = 1.5

# Waiting
sleep = 0.5
sleep1 = 1

# Distances in cm
minDistance = 100

number = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5]

# Definitions for the publisher
rospy.init_node('msgsForMoving')
cmd_vel_pub = rospy.Publisher('/minibot/rvr/cmd_vel', Twist, queue_size=1)
cmd_vel_msg = Twist()
bridge = CvBridge()
depth_image = None
camera_info = None

# Converts the image to a CV2 image format and receives subscriber node
def depth_image_callback(msg):
    global depth_image
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

# Check if there is a WARN message containing the specified text in the log messages
def check_warn_messages(log_messages, search_text):
    for log_msg in log_messages:
        if 'WARN' in log_msg and search_text in log_msg:
            return True
    return False

# Turn the robot slowly
def turn_slowly():
    # Set angular velocity for slow turn
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.angular.z = 0.2  # Adjust the angular velocity as needed
    cmd_vel_pub.publish(cmd_vel_msg)
    rospy.sleep(2)  # Adjust the sleep duration as needed
    # Stop turning
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)

def main():
    global depth_image, camera_info

    # Subscriber for depth image and camera info
    rospy.Subscriber('/minibot/camera/depth/image_rect_raw', Image, depth_image_callback)
    rospy.Subscriber('/minibot/camera/depth/camera_info', CameraInfo, camera_info_callback)
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
            if distance_cm <= minDistance:
                choice = random.choice(number)
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(choice)
                cmd_vel_msg.linear.x = zero
                cmd_vel_msg.angular.z = turn
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(choice)
            print(f"Distance to obstacle: {distance_cm:.2f} cm")

            # Check for the specific WARN message "Registration failed"
            log_messages = rospy.get_published_topics('/rosout')
            has_warn_message = check_warn_messages(log_messages, "Registration failed")

            if has_warn_message:
                print("WARN message: Registration failed!")
                # Turn the robot slowly when there is a WARN message
                turn_slowly()

            if distance_cm <= minDistance:
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
