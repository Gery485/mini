#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

zero = 0
normalSpeed = 0.3
turnRight = 2.0
turnLeft = -1.0
back = -0.5

# Definitions (grounding) also known as initialization
rospy.init_node('msgsForMoving')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
cmd_vel_msg = Twist()
x = cmd_vel_msg.linear.x
z = cmd_vel_msg.angular.z

def laser_scan_callback(msg):
    global x, z  # Access the global variables

    # Moving of the robot based on LaserScan data
    distance_to_obstacle = msg.ranges[0]  # Access the first element for simplicity, adjust the index as needed
    #print(type(distance_to_obstacle))

    # Use distance_to_obstacle for your desired calculations or comparisons
    # For example:
    
    #if distance_to_obstacle == inf:
    #    print('the distance is to big!')
    #else:
    print('Distance of the obstacle is: ' + str(distance_to_obstacle))
    rospy.sleep(0.1)

    
    if distance_to_obstacle < 1.9:
        slowSpeed = 0.1
        turnRightt = 2
        x = slowSpeed
        z = turnRightt

    elif distance_to_obstacle < 1.0:
        x = back
        z = turnRight
        rospy.sleep(0.4)

   
    else:
     x = normalSpeed
     z = zero
     rospy.sleep(0.04)

     x = zero
     z = turnLeft
     rospy.sleep(0.1)

     x = normalSpeed
     z = zero
     rospy.sleep(0.01)

     x = zero
     z = turnRight
     rospy.sleep(0.1)

     x = normalSpeed
     z = zero
     rospy.sleep(0.01)

     x = zero
     z = turnLeft
     rospy.sleep(0.1)

def main():
    # Subscriber
    rospy.Subscriber('/scan', LaserScan, laser_scan_callback) #'replace the laser_scan_topic' with the certain topic

    # Set the rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Publish the message
        cmd_vel_msg.linear.x = x
        cmd_vel_msg.angular.z = z
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    main()