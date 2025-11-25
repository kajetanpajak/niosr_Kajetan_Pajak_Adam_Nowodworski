#!/usr/bin/env python3

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_talker")

        self.cv_bridge = CvBridge()
        self.img_window = cv2.namedWindow("camera image")

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.camera_subscription = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)


    def camera_callback(self, img_msg: Image):
        img = self.cv_bridge.imgmsg_to_cv2(img_msg,
                                           desired_encoding='bgr8')
        
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_parameters)

        print(ids)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

        cv2.imshow('camera image', img)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraSubscriber()

    rclpy.spin(camera_node)

    camera_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()