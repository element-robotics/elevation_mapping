#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

class TfToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')
        
        # Declare and get parameters
        self.declare_parameter('from_frame', '')
        self.declare_parameter('to_frame', '')
        self.from_frame = self.get_parameter('from_frame').get_parameter_value().string_value
        self.to_frame = self.get_parameter('to_frame').get_parameter_value().string_value
        
        pose_name = f'{self.to_frame}_pose'
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, pose_name, 10)

        # Create timer for the callback (50ms)
        self.timer = self.create_timer(0.05, self.callback)

    def callback(self):

        if not self.tf_buffer.can_transform(self.from_frame, self.to_frame, rclpy.time.Time()):
            self.get_logger().warn(f'Cannot transform from {self.from_frame} to {self.to_frame}')
            return
        # Lookup the transform between the frames
        trans = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame, rclpy.time.Time())
    
        # Create and fill pose message for publishing
        pose = PoseWithCovarianceStamped()
        pose.header = trans.header
        pose.pose.pose.position = trans.transform.translation
        pose.pose.pose.orientation = trans.transform.rotation

        # Fill covariance with zeros
        pose.pose.covariance = [0] * 36

        self.publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    
    node = TfToPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()