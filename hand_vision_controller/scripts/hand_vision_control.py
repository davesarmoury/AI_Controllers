#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2

device = "cuda:0"
threshold = 0.5

colours = {"fast": (255, 0, 0), "medium": (255, 255, 0), "slow": (0, 255, 0), "left": (165, 255, 0), "right": (255, 165, 0)}
x_slow = 0.4
x_med = 1.0
x_fast = 1.5
yaw = 1.0

def image_cb(msg):
    global cv_bridge, yolo, img_pub, cmd_pub

    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#    cv_image = cv2.flip(cv_image, 1)

    results = yolo.predict(
        source = cv_image,
        verbose = False,
        stream = False,
        conf = threshold,
        device = device
    )

    results = results[0].cpu()

    actions = []
    if results.boxes:
        for box in results.boxes:
            rect = box.xyxy[0]
            class_id = int(box.cls)
            class_name = yolo.names[class_id]
            actions.append(class_name)
            score = float(box.conf)
            cv2.rectangle(cv_image, (int(rect[0]), int(rect[1])), (int(rect[2]), int(rect[3])), colours[class_name], 4)
            cv2.putText(cv_image, class_name,  (int(rect[0]), int(rect[1])), cv2.FONT_HERSHEY_SIMPLEX, 2, colours[class_name], 2)

    cmd_msg = Twist()
    if "fast" in actions:
        cmd_msg.linear.x = x_fast
    elif "medium" in actions:
        cmd_msg.linear.x = x_med
    elif "slow" in actions:
        cmd_msg.linear.x = x_slow
    else:
        cmd_msg.linear.x = 0.0

    if "left" in actions:
        cmd_msg.angular.z = -yaw
    elif "right" in actions:
        cmd_msg.angular.z = yaw
    else:
        cmd_msg.angular.z = 0.0

    cmd_pub.publish(cmd_msg)
    img_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, encoding="rgb8"))

def main():
    global cv_bridge, yolo, img_pub, cmd_pub

    rospy.init_node('yolo_runner', anonymous=True)

    model = rospy.get_param('~model')

    rospy.loginfo("Starting...")
    cv_bridge = CvBridge()
    rospy.loginfo("Loading Model...")
    yolo = YOLO(model)
    rospy.loginfo("Fusing...")
    yolo.fuse()

    rospy.loginfo("GO!")

    rospy.Subscriber("input", Image, image_cb)
    img_pub = rospy.Publisher("output", Image, queue_size=10)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.spin()

main()
