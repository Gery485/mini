if distance_cm < 135:
            # Perform turning maneuvers
             cmd_vel_msg.linear.x = point2
             cmd_vel_msg.angular.z = zero
             cmd_vel_pub.publish(cmd_vel_msg)
             print("1. half")
             rospy.sleep(sleep)

#snake:
            if distance_cm > 300:
                start1 = time.time()
                if time.time() - start1 < 1:
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




              start = time.time()
                start1 = time.time()
                while time.time() - start < 1.1:
                    cmd_vel_msg.linear.x = zero
                    cmd_vel_msg.angular.z = zero
                    cmd_vel_pub.publish(cmd_vel_msg)
                    rate.sleep()

                while time.time() - start1 < 1.1:
                    cmd_vel_msg.linear.x = zero
                    cmd_vel_msg.angular.z = turn
                    cmd_vel_pub.publish(cmd_vel_msg)
                    rate.sleep()
                continue

            else:
                start1 = time.time()
                start2 = time.time()
                start3 = time.time()
                start4 = time.time()
                start5 = time.time()
                

                while time.time() - start1 < 1.1:
                  cmd_vel_msg.linear.x = move
                  cmd_vel_msg.angular.z = zero
                  cmd_vel_pub.publish(cmd_vel_msg)
                  rate.sleep()
                while time.time() - start2 < 1.1:
                  cmd_vel_msg.linear.x = zero
                  cmd_vel_msg.angular.z = turn
                  cmd_vel_pub.publish(cmd_vel_msg)
                  rate.sleep()
                while time.time() - start3 < 1.1:
                  cmd_vel_msg.linear.x = zero
                  cmd_vel_msg.angular.z = turn * -1
                  cmd_vel_pub.publish(cmd_vel_msg)
                  rate.sleep()
                while time.time() - start4 < 1.1:
                  cmd_vel_msg.linear.x = zero
                  cmd_vel_msg.angular.z = turn * -1
                  cmd_vel_pub.publish(cmd_vel_msg)
                  rate.sleep()
                while time.time() - start5 < 1.1:
                  cmd_vel_msg.linear.x = zero
                  cmd_vel_msg.angular.z = turn
                  cmd_vel_pub.publish(cmd_vel_msg)
                  rate.sleep()
                continue