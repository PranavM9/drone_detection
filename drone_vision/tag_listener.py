import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray


class TagListener(Node):
    """
    A ROS2 node that listens for AprilTag detections produced by apriltag_ros.

    It subscribes to the topic:
        /tag_detections

    apriltag_ros (not this node) handles:
        - Reading images from RealSense D435
        - Detecting AprilTags
        - Publishing 6-DoF poses for each detected tag

    This node simply extracts and prints the pose information.
    """

    def __init__(self):
        super().__init__('tag_listener')

        # --- Subscriber Setup ---
        # Queue size = 10 (standard for low-latency sensor updates)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10
        )

        self.get_logger().info("TagListener started. Waiting for AprilTag detections...")

    # ---------------------------------------------------------------
    # Callback triggered whenever apriltag_ros publishes detections
    # ---------------------------------------------------------------
    def listener_callback(self, msg: AprilTagDetectionArray):

        # No detections in this frame
        if not msg.detections:
            self.get_logger().warn("No AprilTags detected in this frame.")
            return

        # For now, use only the FIRST tag (we can expand later)
        detection = msg.detections[0]

        # Tag ID
        tag_id = detection.id[0] if detection.id else -1

        # Extract 3D pose (in camera frame)
        position = detection.pose.pose.pose.position
        orientation = detection.pose.pose.pose.orientation  # not used yet

        # Log pose
        self.get_logger().info(
            f"[Tag {tag_id}] Position -> "
            f"x: {position.x:.3f} m, "
            f"y: {position.y:.3f} m, "
            f"z: {position.z:.3f} m"
        )

        # Later we'll:
        # - Compute centering error
        # - Publish control commands to flight controller
        # - Align drone for landing


def main(args=None):
    rclpy.init(args=args)
    node = TagListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
