#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
        rospy.init_node("pub_cmd_node")
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.05
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
                pub.publish(twist)
                r.sleep()
