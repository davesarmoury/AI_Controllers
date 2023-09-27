#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2

device = "cuda:0"
threshold = 0.5

colours = {"fast": (255, 0, 0), "medium": (255, 255, 0), "slow": (0, 255, 0), "left": (165, 255, 0), "right": (255, 165, 0)}

def image_cb(msg):
    global cv_bridge, yolo

    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    cv_image = cv2.flip(cv_image, 1)

    results = yolo.predict(
        source = cv_image,
        verbose = False,
        stream = False,
        conf = threshold,
        device = device
    )

    results = results[0].cpu()

    if results.boxes:
        for box in results.boxes:
            rect = box.xyxy[0]
            class_id = int(box.cls)
            class_name = yolo.names[class_id]
            score = float(box.conf)
            cv2.rectangle(cv_image, (int(rect[0]), int(rect[1])), (int(rect[2]), int(rect[3])), colours[class_name], 4)
            cv2.putText(cv_image, class_name,  (int(rect[0]), int(rect[1])), cv2.FONT_HERSHEY_SIMPLEX, 2, colours[class_name], 2)

    pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, encoding="rgb8"))

def main():
    global cv_bridge, yolo, pub

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
    pub = rospy.Publisher("output", Image, queue_size=10)

    rospy.spin()

main()
