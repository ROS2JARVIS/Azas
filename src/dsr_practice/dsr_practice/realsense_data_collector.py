import csv
import json
import os
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RealSenseDataCollector(Node):
    def __init__(self):
        super().__init__("realsense_data_collector")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter(
            "output_dir", "/home/ssu/ros2_ws/realsense_dataset/raw"
        )
        self.declare_parameter("save_interval_sec", 1.0)
        self.declare_parameter("max_frames", 0)
        self.declare_parameter("save_depth", True)
        self.declare_parameter("jpeg_quality", 95)

        self.color_topic = (
            self.get_parameter("color_topic").get_parameter_value().string_value
        )
        self.depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self.save_interval_sec = (
            self.get_parameter("save_interval_sec").get_parameter_value().double_value
        )
        self.max_frames = (
            self.get_parameter("max_frames").get_parameter_value().integer_value
        )
        self.save_depth = (
            self.get_parameter("save_depth").get_parameter_value().bool_value
        )
        self.jpeg_quality = (
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )

        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_stamp = None
        self.latest_depth_stamp = None
        self.camera_info_saved = False
        self.frame_index = 0

        self.color_dir = self.output_dir / "color"
        self.depth_dir = self.output_dir / "depth"
        self.meta_dir = self.output_dir / "meta"
        self.color_dir.mkdir(parents=True, exist_ok=True)
        if self.save_depth:
            self.depth_dir.mkdir(parents=True, exist_ok=True)
        self.meta_dir.mkdir(parents=True, exist_ok=True)

        self.metadata_path = self.meta_dir / "frames.csv"
        self._init_metadata_file()

        self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        if self.save_depth:
            self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.create_timer(self.save_interval_sec, self.save_latest_frames)

        self.get_logger().info(f"Saving RealSense data to {self.output_dir}")
        self.get_logger().info(f"Color topic: {self.color_topic}")
        if self.save_depth:
            self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info("Press Ctrl+C to stop collecting data.")

    def _init_metadata_file(self):
        if self.metadata_path.exists():
            return

        with self.metadata_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "frame_index",
                    "color_file",
                    "depth_file",
                    "color_stamp_sec",
                    "color_stamp_nanosec",
                    "depth_stamp_sec",
                    "depth_stamp_nanosec",
                ]
            )

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_color_stamp = msg.header.stamp

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.latest_depth_stamp = msg.header.stamp

    def camera_info_callback(self, msg):
        if self.camera_info_saved:
            return

        camera_info = {
            "width": msg.width,
            "height": msg.height,
            "distortion_model": msg.distortion_model,
            "d": list(msg.d),
            "k": list(msg.k),
            "r": list(msg.r),
            "p": list(msg.p),
            "intrinsics": {
                "fx": msg.k[0],
                "fy": msg.k[4],
                "cx": msg.k[2],
                "cy": msg.k[5],
            },
        }
        camera_info_path = self.meta_dir / "camera_info.json"
        with camera_info_path.open("w") as json_file:
            json.dump(camera_info, json_file, indent=2)

        self.camera_info_saved = True
        self.get_logger().info(f"Saved camera info to {camera_info_path}")

    def save_latest_frames(self):
        if self.latest_color is None:
            self.get_logger().warn("Waiting for color image...")
            return

        if self.save_depth and self.latest_depth is None:
            self.get_logger().warn("Waiting for aligned depth image...")
            return

        self.frame_index += 1
        base_name = f"frame_{self.frame_index:06d}"
        color_file = f"{base_name}.jpg"
        depth_file = f"{base_name}.png" if self.save_depth else ""

        color_path = self.color_dir / color_file
        cv2.imwrite(
            str(color_path),
            self.latest_color,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if self.save_depth:
            depth_path = self.depth_dir / depth_file
            cv2.imwrite(str(depth_path), self.latest_depth)

        self._append_metadata(color_file, depth_file)
        self.get_logger().info(f"Saved {base_name}")

        if self.max_frames > 0 and self.frame_index >= self.max_frames:
            self.get_logger().info(f"Reached max_frames={self.max_frames}. Stopping.")
            rclpy.shutdown()

    def _append_metadata(self, color_file, depth_file):
        color_stamp_sec = self.latest_color_stamp.sec if self.latest_color_stamp else ""
        color_stamp_nanosec = (
            self.latest_color_stamp.nanosec if self.latest_color_stamp else ""
        )
        depth_stamp_sec = self.latest_depth_stamp.sec if self.latest_depth_stamp else ""
        depth_stamp_nanosec = (
            self.latest_depth_stamp.nanosec if self.latest_depth_stamp else ""
        )

        with self.metadata_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    self.frame_index,
                    color_file,
                    depth_file,
                    color_stamp_sec,
                    color_stamp_nanosec,
                    depth_stamp_sec,
                    depth_stamp_nanosec,
                ]
            )


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
