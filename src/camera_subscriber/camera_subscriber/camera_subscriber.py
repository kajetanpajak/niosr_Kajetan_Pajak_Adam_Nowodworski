#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2 
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_talker")

        self.cv_bridge = CvBridge()
        self.img_window = cv2.namedWindow("camera_image")

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.camera_subscription = self.create_subscription(msg_type=Image,
                                                            topic='image_raw',
                                                            callback=self.camera_callback,
                                                            qos_profile=10)

        self.aruco_pos_publisher = self.create_publisher(msg_type=Bool,
                                                        topic='aruco_above_center',
                                                        qos_profile=10)
        
        self.prev_above_center = None


    def camera_callback(self, img_msg: Image):
        img = self.cv_bridge.imgmsg_to_cv2(img_msg,
                                           desired_encoding='bgr8')
        
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_parameters)


        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

            window_h = img.shape[0]
            center_y = window_h // 2

            corner_pts = corners[0][0]
            aruco_center_y = np.mean(corner_pts[:, 1])

            if aruco_center_y <= center_y:
                aruco_above_center = True
            else:
                aruco_above_center = False

            bool_msg = Bool()

            if aruco_above_center != self.prev_above_center or self.prev_above_center is None:
                bool_msg.data = aruco_above_center
                self.aruco_pos_publisher.publish(bool_msg)
                self.prev_above_center = aruco_above_center

        cv2.imshow('camera_image', img)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraSubscriber()

    rclpy.spin(camera_node)

    camera_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()