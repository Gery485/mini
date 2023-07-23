if distance_cm < 135:
            # Perform turning maneuvers
             cmd_vel_msg.linear.x = point2
             cmd_vel_msg.angular.z = zero
             cmd_vel_pub.publish(cmd_vel_msg)
             print("1. half")
             rospy.sleep(sleep)

#snake:
            if distance_cm > 300:
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = 3
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
            elif distance_cm <= 300:
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = 3
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
            elif distance_cm <= 100:
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = -6
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = 6
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep/2)
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = -3
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
            elif distance_cm <= minDistance:
                 cmd_vel_msg.linear.x = minPoint2
                 cmd_vel_msg.angular.z = point2
                 cmd_vel_pub.publish(cmd_vel_msg)
                
                


                
                if distance_cm > 300:
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
            elif distance_cm <= 300:
                cmd_vel_msg.linear.x = point2
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)
            elif distance_cm <= 100:
                cmd_vel_msg.linear.x = point2/2
                cmd_vel_msg.angular.z = zero
                cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(sleep)

            elif distance_cm < minDistance:
            # Move with minPoint2 linear velocity and point2 angular velocity
             cmd_vel_msg.linear.x = minPoint2
             cmd_vel_msg.angular.z = point2
             cmd_vel_pub.publish(cmd_vel_msg)

            else:
            # Move forward with point2 linear velocity and point2 angular velocity
             cmd_vel_msg.linear.x = point2
             cmd_vel_msg.angular.z = point2
             cmd_vel_pub.publish(cmd_vel_msg)