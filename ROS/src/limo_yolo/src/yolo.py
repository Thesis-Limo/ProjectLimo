#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import queue
from threading import Thread

cupID = 39 # this is the id from coco libary
processing = False
model = YOLO("yolov8n.pt")
q1 = queue.Queue(10)

def ProcessImage(image):
    global processing
    processing = True
    print("processed")
    results  = model.predict(image)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            if(box.cls == cupID):
                print(box.xyxy[0])
    processing = False
    return


def callback(data):
    if(q1.full()):
        q1.get()
    image_a = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height,data.width,-1)
    image = cv2.normalize(image_a, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    q1.put(image)
    

if __name__ == '__main__':
    rospy.init_node('ImageConverter', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback   )
    rate = rospy.Rate(10)
    img = q1.get()
    thread = None
    while not rospy.is_shutdown():
        # do stuff
        if len(img) <= 0:
            print("gotImage")
            img = q1.get()
        elif processing == False:
            print("start thread")
            thread = Thread(target=ProcessImage, args=(img,))
            thread.start()
        elif processing == True:
            print("stop thread")
            thread.join()
            img = q1.get()
        #print("tt")
        rate.sleep()
