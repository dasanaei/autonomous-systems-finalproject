#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_car import MoveCar
from sys import exit

class Self_Drive(object):

    ###Initialize variables##
    def __init__(self, rate):
        #Failsafe values protects against false readings. 
        self.failsafe_grass = 0 
        self.failsafe_cone = 0
        self.failsafe_gas = 0
        #coneNum stores which cone the car is approaching (0 for first cone, 1 for second cone)
        self.coneNum = 0

        #Initialize objects and camera subscriber
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/catvehicle/camera_front/image_raw_front",Image,self.camera_callback)
        self.move_car_object = MoveCar()
        self.rate = rate

    ###Callback function for camera subscriber##
    def camera_callback(self,data):
        # Detect centroid (cx) of the yellow middle lane
        cx , width= self.detectLane(data) 
        # Detect the grass (used for 90 degree turn detection). Returns True when grass is detected
        Grass =  self.detectGreen(data) 
        # Detects the cone. Returns true when cone detected
        obstacle = self.detectCone(data)
        # Detects the gas station. Returns true when station is detected
        gas_station = self.detectStation(data)

        cv2.waitKey(1)
        
        # If grass is detected, follow the yellow middle line
        if Grass:
            self.failsafe_grass = 0

            # Simple Proportional Controller
            error_x = cx - width / 5.2 #Keep lane to the left of the screen
            twist_object = Twist()
            twist_object.linear.x = 2
            twist_object.angular.z = -error_x / 200
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            self.move_car_object.move_car(twist_object) # Send twist message to move car

        # If the first cone is detected
        if obstacle and self.coneNum == 0:
            rospy.loginfo("Obstacle 1 Detected")
            self.failsafe_cone = self.failsafe_cone + 1

            # Require 15 consecutive detections before performing obstacle avoidance (in order to stop false detections)
            if self.failsafe_cone >= 15:

                #Turn right
                rospy.loginfo("Turning Right")
                twist_object = Twist()
                for i in range(0,5):
                    twist_object.linear.x = 2
                    twist_object.angular.z = 2
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()

                #Go straight
                rospy.loginfo("Going Straight")
                for i in range(0,6):
                    twist_object = Twist()
                    twist_object.linear.x = 2
                    twist_object.angular.z = .1
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()

                #Turn left
                rospy.loginfo("Turning Right")
                for i in range(0,7):
                    twist_object = Twist()
                    twist_object.linear.x = 2
                    twist_object.angular.z = -2
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()
                self.failsafe_cone = 0
                self.coneNum = 1

        # If the second cone is detected
        if obstacle and self.coneNum == 1:
            rospy.loginfo("Obstacle 2 Detected")
            self.failsafe_cone = self.failsafe_cone + 1

            # Require 15 consecutive detections before performing obstacle avoidance (in order to stop false detections)
            if self.failsafe_cone >= 15:
                rospy.loginfo("Turning Right")
                twist_object = Twist()
                for i in range(0,5):
                    twist_object.linear.x = 2
                    twist_object.angular.z = 2
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()

                rospy.loginfo("Going Straight")
                for i in range(0,8):
                    twist_object.linear.x = 2
                    twist_object.angular.z = .1
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()

                rospy.loginfo("Turning Right")
                for i in range(0,5):
                    twist_object.linear.x = 2
                    twist_object.angular.z = -2
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()
                self.failsafe_cone = 0
                self.coneNum = 1
        
        #Reset cone detection failsafe variable if cone is not detected
        if obstacle is False:
            self.failsafe_cone = 0

        #Detects the 90 degree turn
        if Grass is not True:
            rospy.loginfo("Turn Detected")
            self.failsafe_grass = self.failsafe_grass + 1

            # Requires 15 consecutive non-grass detections to ensure its not a false alarm
            if self.failsafe_grass >= 15:
                twist_object = Twist()
                rospy.loginfo("Turn Part 1")
                for i in range(0,8):
                    twist_object.linear.x = 2
                    twist_object.angular.z = 0
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()

                rospy.loginfo("Turn Part 2")
                for i in range(0,19):
                    twist_object.linear.x = 1.75
                    twist_object.angular.z = -3
                    self.move_car_object.move_car(twist_object)
                    self.rate.sleep()
                self.failsafe_grass = 0
           
        # Detects the gas station
        if gas_station:
            self.failsafe_gas = self.failsafe_gas + 1

            #Requires 50 consecutive detections in order to stop a false detection and allow the car to move to the required position
            if self.failsafe_gas >= 60:
                twist_object = Twist()
                #Stop car
                twist_object.linear.x = 0
                twist_object.angular.z = 0
                self.move_car_object.move_car(twist_object)
                rospy.loginfo("Gas Station Detected")
                rospy.loginfo("End Drive")
                self.image_sub.unregister()
        
        #Reset gas station detection failsafe
        if gas_station is False:
            self.failsafe_gas = 0


    ###Finds the Cx of the centroid for the yellow middle line###
    def detectLane(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # Import and crop image
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 200
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:400]
        height, width, channels = cv_image.shape
        crop_img = crop_img[1:height, 75:600]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Yellow color range 
        lower_lane = np.array([25, 100, 100])
        upper_lane = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_lane, upper_lane)
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00'] #Obtain centroid if the lane is detected
        except ZeroDivisionError:
            cy, cx = height/2, width/2 #If lane is not detected
        
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        # Display original and masked camera feeds
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK_lane", mask)

        #Return centroid location and image width
        return cx, width
        
    def clean_up(self):
        self.move_car_object.clean_class()
        #cv2.destroyAllWindows()
        ctrl_c = True
    
    ### Detect Green for Grass ###
    def detectGreen(self, data):
        Green = True 
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Import and crop image
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 40
        crop_img = cv_image[2*height/3+20:-1, 1:width]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Green color range
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([60, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            Green = False

        # Display masked camera feeds
        cv2.imshow("MASK_green", mask)

        #Return true if green is detected, False if not detected
        return Green
    

    ### Detect Orange for Cone ###
    def detectCone(self, data):
        Cone = True
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # Import and crop image
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 40
        crop_img = cv_image[500:height-250, width/2 - 50:width/2+50]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Orannge color range
        lower_orange = np.array([5,50,50])
        upper_orange = np.array([15,255,255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            Cone = False #If not detected

        # Display masked camera feeds
        cv2.imshow("MASK_cone", mask)

        #Return true if cone is detected, False if not detected
        return Cone


    def detectStation(self, data):
        red = True
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Import and crop image 
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 40
        crop_img = cv_image[2*height/3+20:-1, 1:width]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Red color range
        mask = cv2.inRange(hsv, (0,50,50), (10,255,255))
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            red = False #If not detected

        # Display masked camera feed
        cv2.imshow("MASK_red", mask)
        
        #Return true if cone is detected, False if not detected
        return red