import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray

class TagListener(Node):

    def __init__(self):
        super().__init__('tag_listener')

        # Subscribe to the AprilTag detections topic
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',   # default topic from apriltag_ros
            self.listener_callback,
            10
        )

        self.get_logger().info("TagListener node started. Waiting for detections...")

    def listener_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            self.get_logger().info("No tags detected.")
            return

        # Use first detected tag
        det = msg.detections[0]
        pos = det.pose.pose.pose.position

        # Print values
        self.get_logger().info(
            f"Tag ID {det.id[0]} | Position -> x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TagListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
