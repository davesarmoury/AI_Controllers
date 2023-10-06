#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

x_slow = 0.5
x_med = 1.0
x_fast = 1.5
yaw_rate = 1.0

hz = 10

x = 0.0
yaw = 0.0

def callback(msg):
    global pub, x, yaw

    command = msg.data.lower()

    if "stop" in command:
        x = 0.0
        z = 0.0
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
        if "straight" in command:
            yaw = 0

def main():
    global pub, x, yaw

    rospy.init_node("riva_command_handler", anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("commands", String, callback)

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        twist_msg = Twist()

        twist_msg.linear.x = x
        twist_msg.angular.z = yaw

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
