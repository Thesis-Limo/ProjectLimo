#!/usr/bin/env python3.6
import rospy
# import cv2
import numpy as np
import message_filters
#   from limo_behaviour_tree import TypeObjectTracking 
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, Point


class LIDARCameraOverlay:
    def __init__(self):
        rospy.init_node('lidar_camera_overlay')

        #self.bridge = CvBridge()
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Set up message filters for approximate time synchronization
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback_image_and_lidar)
        print("haha")
        # self.yoloImagePub = rospy.Publisher("/camera/yolo_input", Image, queue_size=10)
        # self.yoloInfoBack = False
        # self.PosObject = rospy.Publisher("/ObjectPos", Point, queue_size=10)
        # self.target = rospy.Service("/TrackID", TypeObjectTracking, self.switch_target)


        # self.camera_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo)
        # self.K = np.array(self.camera_info.K).reshape((3, 3))
        # self.objectID = 0

    def callback_image_and_lidar(self, image_msg, lidar_msg):
        
        """Callback for simultanious lidar and image information

        Args:
            image_msg (sensor_msgs/Image): current image
            lidar_msg (sensor_msgs/LaserScan): currentlaserscan
        """
        print("works")
        if(self.yoloInfoBack):
            print("start yolo with image")
            self.currentImage = image_msg.data
            self.currentLidarScan = lidar_msg
            self.yoloInfoBack = False
            self.yoloImagePub.publish(image_msg)

    def callback_yolo_result(self, boundingboxes):
        items=[]
        for box in boundingboxes.bounding_boxes:
            if(box.id != self.objectID): continue
            items.append(box.Class)
        print("finished yolo with ", len(items), " items found")
        targetPos = Point
        targetPos.x = 0
        targetPos.y = 0
        targetPos.z = 0
        self.PosObject.publish(targetPos) 
        self.yoloInfoBack = True
    
    def switch_target(self, object):
        """update the tracking id and reset looking

        Args:
            object (TypeObjectTracking): trackingobjectservice
        """
        self.objectID = object.objectID
        self.currentImage = None




    #def callback(self, image_msg, lidar_msg):   
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #         return

    #     image = cv_image.copy()

    #     min_dist = lidar_msg.range_min
    #     max_dist = lidar_msg.range_max

    #     for i, distance in enumerate(lidar_msg.ranges):
    #         if min_dist < distance < max_dist:
    #             angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
    #             x = distance * np.cos(angle)
    #             y = distance * np.sin(angle)
    #             lidar_point = PointStamped()
    #             lidar_point.header.frame_id = "laser_link"
    #             lidar_point.point.x = x - 0.020
    #             lidar_point.point.y = y - 0.045
    #             lidar_point.point.z = 0

    #             try:
    #                 transformed_point = self.tf_buffer.transform(lidar_point, "camera_rgb_optical_frame", rospy.Duration(1.0))
    #                 pt = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
    #                 pixel = np.dot(self.K, pt)
    #                 pixel /= pixel[2]

    #                 color = self.distance_to_color(distance, min_dist, max_dist)

    #                 if 0 <= pixel[0] < image.shape[1] and 0 <= pixel[1] < image.shape[0]:
    #                     cv2.circle(image, (int(pixel[0]), int(pixel[1])), 5, color, -1)
    #             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
    #                 rospy.logwarn("Could not get transform: %s" % ex)

    #     cv2.imshow("LIDAR Camera Overlay", image)
    #     cv2.waitKey(1)
    # def distance_to_color(self, distance, min_dist, max_dist):
    #     normalized = (distance - min_dist) / (max_dist - min_dist)
    #     color_hsv = np.array([[[(1-normalized) * 120, 255, (1-normalized) * 120]]], dtype=np.uint8)
    #     color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)
    #     color = tuple(map(int, color_bgr[0][0]))
    #     return color
    
if __name__ == '__main__':
    LIDARCameraOverlay()
    rospy.spin()