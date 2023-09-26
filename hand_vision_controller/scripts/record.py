import numpy as np
import cv2 as cv2
import math
from yaml import safe_load
import os

def main():
    if not os.path.exists("images"):
        print("Creating images dir")
        os.mkdir("images")

    cam = cv2.VideoCapture(0, cv2.CAP_V4L2) # Need CAP_V4L2 for properties to work

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cam.set(cv2.CAP_PROP_FOCUS, 0)

    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    count = 0
    while True:
        ret, frame = cam.read()
        cv2.imwrite("images/" + str(count) + ".jpg", frame)
        count = count + 1
        cv2.imshow("camera", frame)
        key = cv2.waitKey(10)
        if key & 0xFF == 27:
             break

main()
cv2.destroyAllWindows()
