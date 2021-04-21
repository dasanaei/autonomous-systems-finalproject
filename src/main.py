#!/usr/bin/env python
import rospy
from perception import Self_Drive

rospy.loginfo("Start Program")
def shutdownhook():
    line_follower_object.clean_up()
    rospy.loginfo("shutdown time!")
    ctrl_c = True

#Initialize node and initial values
rospy.init_node('self_driving_node', anonymous=True)
ctrl_c = False
rate = rospy.Rate(5) # Set rate to 5 hz

# Create self driving object
self_drive_object = Self_Drive(rate)

rospy.on_shutdown(shutdownhook)

# Loop until terminated by user
while not ctrl_c:
    rate.sleep()



