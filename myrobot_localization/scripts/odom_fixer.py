#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry



class odom_fixer:
    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('odom_fixer', anonymous=True)
        self.odom = Odometry() 

        rospy.Subscriber("steer_bot/ackermann_steering_controller/odom", Odometry, self.callback)
        pub = rospy.Publisher('odom', Odometry, queue_size=10)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pub.publish(self.odom)
            rate.sleep()

    def callback(self, data):
        self.odom = data
        self.odom.twist.twist.linear.x *= 2


    

if __name__ == '__main__':
    odom_fixer()
