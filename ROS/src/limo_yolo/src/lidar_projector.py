#!/usr/bin/env python
import rospy
from limo_yolo.msg import transformedLidarPoints
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
#   from limo_behaviour_tree import TypeObjectTracking 
from sensor_msgs.msg import  LaserScan,CameraInfo,Image
from geometry_msgs.msg import PointStamped, Point
import time
from darknet_ros_msgs.msg import BoundingBoxes
import tf2_ros,  tf2_geometry_msgs

import Queue

class LIDARCameraOverlay:
    def __init__(self):
        rospy.init_node('lidar_camera_overlay')

        self.bridge = CvBridge()
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback_image_and_lidar)
        
        # Set up message filters for approximate time synchronization
        self.pub = rospy.Publisher("/points", transformedLidarPoints, queue_size=10)
        self.yolopub = rospy.Publisher("/camera/yolo_input", Image, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, self.callback)
        self.id = 0
        self.pushedFrames = Queue.Queue()
        self.camera_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo)
        self.K = np.array(self.camera_info.K).reshape((3, 3))
        self.tfBuffer = tf2_ros.Buffer()           # Creates a frame buffer
        tf2_ros.TransformListener(self.tfBuffer)


    def callback_image_and_lidar(self, image_msg, lidar_msg):
        p = (self.id, lidar_msg, image_msg)
        self.yolopub.publish(image_msg)
        self.pushedFrames.put(p)
        self.id+=1

    def callback(self, darknet): 
        if(self.pushedFrames.qsize() <= 0): return
        img = self.pushedFrames.queue[0]
        while img[0] < darknet.image_header.seq:
            img = self.pushedFrames.get()
        while img[0] > darknet.image_header.seq:
            img = self.pushedFrames.get()
        print(img[0])
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img[2], "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        image = cv_image.copy()
        min_dist = img[1].range_min
        max_dist = img[1].range_max

        for i, distance in enumerate(img[1].ranges):
            if min_dist < distance < max_dist:
                angle = img[1].angle_min + i * img[1].angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                lidar_point = PointStamped()
                lidar_point.header.frame_id = "laser_link"
                lidar_point.point.x = x - 0.020
                lidar_point.point.y = y - 0.045
                lidar_point.point.z = 0

                try:
                #transformed_point = self.tl.transformPoint("camera_rgb_frame", lidar_point)
                    transformed_point = self.tfBuffer.transform(lidar_point, "camera_rgb_optical_frame", rospy.Duration(1.0))
                    pt = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
                    pixel = np.dot(self.K, pt)
                    pixel /= pixel[2]
                
                    color = self.distance_to_color(distance, min_dist, max_dist)

                    if 0 <= pixel[0] < image.shape[1] and 0 <= pixel[1] < image.shape[0]:
                        cv2.circle(image, (int(pixel[0]), int(pixel[1])), 5, color, -1)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    rospy.logwarn("Could not get transform: %s" % ex)

        cv2.imshow("LIDAR Camera Overlay", image)
        cv2.waitKey(1)
    def distance_to_color(self, distance, min_dist, max_dist):
        normalized = (distance - min_dist) / (max_dist - min_dist)
        color_hsv = np.array([[[(1-normalized) * 120, 255, (1-normalized) * 120]]], dtype=np.uint8)
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)
        color = tuple(map(int, color_bgr[0][0]))
        return color
    
if __name__ == '__main__':
    LIDARCameraOverlay()
    rospy.spin()