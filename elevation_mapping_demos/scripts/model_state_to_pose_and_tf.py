#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, TransformStamped
import tf2_ros

class PoseNode:
    def __init__(self):
        # Node initialization
        rospy.init_node('robot_pose_node')

        # Subscribe to /gazebo/model_states
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        
        # Publisher for robot_pose
        self.pose_pub = rospy.Publisher('/leo_pose', PoseWithCovarianceStamped, queue_size=10)

        # Transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.last_timestamp = rospy.Time(0)

        # Store initial pose (origin)
        self.initial_pose = None

    def model_callback(self, data):
        try:
            robot_index = data.name.index('leo')  # Replace with your robot's model name
            current_pose = data.pose[robot_index]

            if self.initial_pose is None:
                self.initial_pose = current_pose
                return

            offset_pose = self.compute_offset_pose(current_pose)

            # Publish PoseWithCovariance
            pose_with_cov = PoseWithCovarianceStamped()
            pose_with_cov.pose.pose = offset_pose
            pose_with_cov.pose.covariance = [0.0] * 36
            pose_with_cov.header.stamp = rospy.Time.now()
            pose_with_cov.header.frame_id = "odom"
            self.pose_pub.publish(pose_with_cov)

            # Broadcast transform with the current timestamp
            timestamp = rospy.Time.now()  # Or use a timestamp from the message if available
            if timestamp != self.last_timestamp:
                self.broadcast_transform(offset_pose, timestamp)
            self.last_timestamp = timestamp

        except ValueError:
            rospy.logwarn("Robot model not found in /gazebo/model_states")

    def compute_offset_pose(self, current_pose):
        # Offset current_pose by initial_pose to make initial_pose the origin
        offset_pose = Pose()
        offset_pose.position.x = current_pose.position.x - self.initial_pose.position.x
        offset_pose.position.y = current_pose.position.y - self.initial_pose.position.y
        offset_pose.position.z = current_pose.position.z - self.initial_pose.position.z

        offset_pose.orientation = current_pose.orientation

        return offset_pose

    def broadcast_transform(self, pose, timestamp):
        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"

        # Set translation
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        # Set rotation
        transform.transform.rotation = pose.orientation

        # Broadcast the transform
        self.br.sendTransform(transform)

if __name__ == '__main__':
    try:
        node = PoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass