#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from move_car import MoveCar

class ObstacleStopper(object):
    def __init__(self, rate):
        self.obstacle_move_car = MoveCar()
        self.rate = rate
        #self.obstacle_detection(minimum_distance)
        #rospy.Subscriber("/catvehicle/cmd_vel", Twist, self.cmdvel_callback)
        #self.pub_cmdvel_safe = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=0)
        
    def obstacle_detection(self, minimum_distance, bypass):
        try:
            closest_distance = rospy.wait_for_message('/distanceEstimator/dist', Float32, timeout=1).data
        except:
            rospy.logwarn("Time out /distanceEstimator/dist")
            closest_distance = None

        twist_object = Twist()

        if closest_distance:
            object_detected = closest_distance < minimum_distance
            if object_detected:
                if bypass is True:
                    twist_object.linear.x = 0.0
                    rospy.loginfo("Obstacle Detected")
                self.obstacle_move_car.move_robot(twist_object)
                return True

        else:
            return False
        
        
    def turn_left(self):
        rospy.loginfo("Turning Right")
        for i in range(0,10):
            twist_object = Twist()
            twist_object.linear.x = 2
            twist_object.angular.z = 2
            self.obstacle_move_car.move_robot(twist_object)
            self.rate.sleep()
        
    def go_straight(self):
        rospy.loginfo("Going Straight")
        for i in range(0,20):
            twist_object = Twist()
            twist_object.linear.x = 2
            twist_object.angular.z = .1
            self.obstacle_move_car.move_robot(twist_object)
            self.rate.sleep()

    def turn_right(self):
        rospy.loginfo("Turning Right")
        for i in range(0,10):
            twist_object = Twist()
            twist_object.linear.x = 2
            twist_object.angular.z = -2
            self.obstacle_move_car.move_robot(twist_object)
            self.rate.sleep()
    
    def listener(self):
        rospy.spin()
