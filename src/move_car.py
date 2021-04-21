import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MoveCar(object):

    def __init__(self):
        # Initialize /cmd_vel publisher and twist object
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_cmdvel_command = Twist()
        self.shutdown_detected = False
       
        
    # Moves the car with a twist object
    def move_car(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
        
    # Stops car 
    def clean_class(self):
        # Stop car
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_detected = True

    # Test function
    def test_move_car(self):
        rospy.init_node('test_node', anonymous=True)
        rate = rospy.Rate(10) # run at 10Hz
        while not rospy.is_shutdown():
            twist_test = Twist()
            twist_test.linear.x = 2
            print("test")
            self.move_robot(twist_test)
            rate.sleep()


#test = MoveCar()
#test.test_move_car()