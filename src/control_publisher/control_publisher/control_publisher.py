import rclpy 
from rclpy.node import Node

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class ControlPublisher(Node):
    def __init__(self):
        super().__init__("control_publisher")

        self.bool_subscription = self.create_subscription(msg_type=Bool,
                                                          topic="aruco_above_center",
                                                          callback=self.bool_msg_callback,
                                                          qos_profile=10)
        
        self.trajectory_publisher = self.create_publisher(msg_type=JointTrajectory,
                                                          topic="scaled_joint_trajectory_controller/joint_trajectory",
                                                          qos_profile=10)
        
        self.down_pos = [np.deg2rad(-91),
                        np.deg2rad(-108),
                        np.deg2rad(-130),
                        np.deg2rad(-33),
                        np.deg2rad(90),
                        np.deg2rad(0)]
        
        self.up_pos = [np.deg2rad(-91),
                        np.deg2rad(-84),
                        np.deg2rad(-105),
                        np.deg2rad(-82),
                        np.deg2rad(90),
                        np.deg2rad(0)]
        
        self.joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'elbow_joint',
                            'wrist_1_joint',
                            'wrist_2_joint',
                            'wrist_3_joint']
        

    def bool_msg_callback(self, msg: Bool):
        value = msg.data

        if value is True:
            self.move_arm(self.up_pos)
        elif value is False:
            self.move_arm(self.down_pos)
    

    def move_arm(self, target_pos):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        point.positions = target_pos
        point.time_from_start.sec = 4

        trajectory_msg.points.append(point)

        self.trajectory_publisher.publish(trajectory_msg)

        
def main():
    rclpy.init()

    control_node = ControlPublisher()

    rclpy.spin(control_node)

    control_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
