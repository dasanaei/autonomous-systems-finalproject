#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_car import MoveCar


class LineFollower(object):
    def __init__(self):
        
        #Initialize camera subscriber and move_robot object
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/catvehicle/camera_front/image_raw_front",Image,self.camera_callback)
        self.movekobuki_object = MoveCar()

    #Callback for camera subscriber
    def camera_callback(self,data):
        
        
        cx , width = self.detectWhite(data) # Perform cv on white line
        noGreen = True #Perform cv on green star. Returns false when green is detected
        cx = self.detect_lane(data)

        cv2.waitKey(1)
        
        #If no green is detected, the robot follows the white line
        if noGreen:
            error_x = cx - width / 5
            twist_object = Twist()
            twist_object.linear.x = 1
            twist_object.angular.z = -error_x / 100
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            print(cx)
            self.movekobuki_object.move_robot(twist_object)
        #if green is detected, robot stops and wags tail
        else:
            print("Green Detected")
            self.movekobuki_object.wiggle_tail()
            self.clean_up()
        
    # Finds the Cx of the centroid for the white line
    def detectWhite(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow("combo", cv_image)

        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        #cv_image = cv2.bitwise_not(cv_image) #Invert image to help tracking white line
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # White color range (black since image is inverted)
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([0, 0, 50])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        #cv2.imshow("Original", cv_image)
        #cv2.imshow("Crop", crop_img)
        #cv2.imshow("MASK_white", mask)
        #cv2.imshow("RES", res)
        #cv2.imshow("combo", lane_image)
        return cx, width
        



    def detect_lane(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        lane_image = cv_image
        height, width, channels = cv_image.shape
        gray,blur,canny_image = self.canny(lane_image)
        mask,cropped_image = self.region_of_interest(canny_image)
        #cropped_image = cv_image[200:-1, 1:width]
        cv2.imshow("canny", cropped_image)
        cv2.imshow("canny2343344343", cropped_image)
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]),minLineLength=50,maxLineGap=5)#image,pixel,accuracy,threshold(votes)
        average_lines, target, skip = self.average_slope_intercept(lane_image,lines)  
        if skip is False:
            line_image = self.display_line(lane_image,average_lines)
            combo_image = cv2.addWeighted(lane_image, 1, line_image, 1, 1)
 
            cv2.imshow("combo", combo_image)
            cv2.imshow("canny", cropped_image)
        #cv2.imshow("broke", gray)
        #print(mid_line)
        return target






    def clean_up(self):
        self.movekobuki_object.clean_class()
        #cv2.destroyAllWindows()
        ctrl_c = True
    
    def canny(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # convert to grayscale 
        blur = cv2.GaussianBlur(gray,(5,5),0) # reduce noise
        canny = cv2.Canny(blur,50,150) # Find the edges
        return gray,blur,canny

    def region_of_interest(self, image):
        height,width = image.shape[0],image.shape[1]
        polygons = np.array([[(0,height-200),(width,height-200),(int(width/2),int(0.5*height))]])
        polygons = np.array( [[[100,height-200],[width-100,height-200],[width-100,500],[200,500]]], dtype=np.int32 )
        mask = np.zeros_like(image) # black
        cv2.fillPoly(mask,polygons,255) # white
        maskded_image = cv2.bitwise_and(image,mask)
        return mask,maskded_image

    def display_line(self, image,lines):
        line_image = np.zeros_like(image)
        height, width, channels = image.shape
        if lines is not None:
            print(lines)
            for x1, y1, x2, y2 in lines:
                cv2.line(line_image,(int(x1), int(y1)),(int(x2), int(y2)),(255, 0, 0), 10)#only one line

        #print("width:%s,height:%s,channels:%s" % (width, height, channels))
        #cv2.line(line_image, (int(width/2), int(height)), (int(width/2), int(height*0.6)), (0, 255, 0), 10)  # only one line
        return line_image

    def make_coordinate(self, image,line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1*3/5)
        x1 = int((y1-intercept)/slope)
        x2 = int((y2-intercept)/slope)
        return np.array([x1, y1, x2, y2])

    #-------------------average------------
    def average_slope_intercept(self, image,lines):
        left_fit = []
        right_fit = []
        print(lines)
        if lines is None:
            return np.array([0,0,0]), 100, True 
        else:
            print(lines)
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                parameters = np.polyfit((x1, x2),(y1, y2),1)
                #print(parameters)
                slope = parameters[0]
                intercept = parameters[1]
                if slope<0:
                    left_fit.append((slope,intercept))
                else:
                    right_fit.append((slope,intercept))
            left_fit_average = np.average(left_fit, axis = 0)
            right_fit_average = np.average(right_fit, axis = 0)
            left_line = self.make_coordinate(image,left_fit_average)
            right_line = self.make_coordinate(image,right_fit_average)
            mid_line = (left_line+right_line)/2

            return np.array([left_line,right_line,mid_line]), left_line[0], False


def main():
    rospy.init_node('line_following_node', anonymous=True)

    line_follower_object = LineFollower()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    ctrl_c = False
    main()