#!/usr/bin/env python3

# BSD 3-Clause License
# Copyright (c) ...
# (Keeping your original license text)
# -------------------------------------------------------------

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml

from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory


class AprilTagNode(Node):
    """
    A ROS2 node that:
      - Subscribes to a camera image topic
      - Detects AprilTags using pupil_apriltags
      - Computes pose (x,y,z + quaternion)
      - Publishes pose to /apriltag_pose
      - Displays debugging visualization
    """

    def __init__(self):
        super().__init__('apriltag_node')

        self.bridge = CvBridge()

        # Load camera parameters
        calib_file = self.declare_parameter(
            'camera_info_file', ''
        ).get_parameter_value().string_value

        calib_file = os.path.expanduser(calib_file)

        # If user provided calibration file is missing, fallback to package default
        if calib_file and os.path.isfile(calib_file):
            camera_info_path = calib_file
        else:
            pkg_share = get_package_share_directory('lab11_apriltag')
            camera_info_path = os.path.join(pkg_share, 'config', 'default_cam.yaml')

            if not os.path.isfile(camera_info_path):
                self.get_logger().error(f"Calibration file not found: {camera_info_path}")
                raise FileNotFoundError(camera_info_path)

        self.load_camera_info(camera_info_path)

        # AprilTag parameters
        self.tag_size = 0.166  # meters
        self.detector = Detector(families="tag36h11")

        # ✨ Camera topic subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Jetson RealSense topic (change if using webcam)
            self.image_callback,
            10
        )

        # ✨ Pose publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/apriltag_pose',
            10
        )

        self.get_logger().info("AprilTagNode initialized. Waiting for camera frames...")

    # -------------------------------------------------------
    # Load calibration file
    # -------------------------------------------------------
    def load_camera_info(self, filepath):
        with open(filepath, 'r') as f:
            calib = yaml.safe_load(f)

        self.K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(calib['distortion_coefficients']['data'])

        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

        self.get_logger().info(f"Loaded calibration: {filepath}")

    # -------------------------------------------------------
    # Image processing + AprilTag detection
    # -------------------------------------------------------
    def image_callback(self, image_msg):
        try:
            # Convert ROS image → CV2
            frame = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rectify
        h, w = gray.shape
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1)
        gray = cv2.undistort(gray, self.K, self.D, None, new_K)

        # Camera params for detector
        camera_params = [self.fx, self.fy, self.cx, self.cy]

        # Detect tags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size
        )

        if not tags:
            self.get_logger().info("No tags detected.")
            return

        # For now: use only the first tag
        tag = tags[0]

        # ------------------------------
        # Extract position + orientation
        # ------------------------------
        t = tag.pose_t.flatten()
        R = tag.pose_R

        quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]

        # ------------------------------
        # Build PoseStamped message
        # ------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link'

        pose_msg.pose.position.x = float(t[0])
        pose_msg.pose.position.y = float(t[1])
        pose_msg.pose.position.z = float(t[2])

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # Publish pose
        self.pose_pub.publish(pose_msg)

        # Print for debugging
        self.get_logger().info(
            f"[Tag {tag.tag_id}] xyz=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})"
        )

        # ------------------------------
        # Debug Visualization
        # ------------------------------
        for tag in tags:
            corners = np.int32(tag.corners)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

            c = tuple(np.int32(tag.center))
            cv2.putText(frame, f"ID:{tag.tag_id}", c,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow("AprilTag Detection", frame)
        cv2.waitKey(1)


# -------------------------------------------------------
# Main
# -------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
