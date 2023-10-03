#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

x_slow = 0.5
x_med = 1.0
x_fast = 1.5
yaw_rate = 1.0

def callback(msg):
    global pub
    command = msg.data.lower()

    if "stop" in command:
        pub.publish(Twist())
    else:
        if "fast" in command:
            x = x_fast
        if "medium" in command:
            x = x_med
        if "slow" in command:
            x = x_slow
        if "right" in command:
            yaw = -yaw_rate
        if "left" in command:
            yaw = yaw_rate

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.angular.z = yaw
        pub.publish(twist_msg)

def main():
    global pub
    rospy.init_node("riva_command_handler", anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("commands", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
