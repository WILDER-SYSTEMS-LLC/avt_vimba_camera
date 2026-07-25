#!/usr/bin/env python3
"""Sim publisher for the AVT mono camera.

Loops through a directory of pre-captured sample images and publishes each
in turn as sensor_msgs/Image on the `image` topic, along with a matching
sensor_msgs/CameraInfo on `camera_info`. Intended to stand in for the real
avt_vimba_camera mono_camera_node when running scissor_tail in sim mode.

Topics (remap in the launch file to hit the actual downstream names, e.g.
/sci_viz_image_raw and /sci_viz_cam_info):
    image        : sensor_msgs/Image        (bgr8)
    camera_info  : sensor_msgs/CameraInfo   (identity K/D, matching dims)

Parameters:
    image_dir  (string): directory containing sample images. Defaults to
                         share/avt_vimba_camera/sample_images.
    rate_hz    (double): publish rate in Hz. Default 0.5 (one image every
                         2 seconds).
    frame_id   (string): frame_id stamped on published messages.
                         Default "ai_camera_optical".
    loop       (bool)  : loop back to the first image after the last.
                         Default True.
"""

from __future__ import annotations

from pathlib import Path
from typing import List

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")


class MonoCameraSimNode(Node):
    def __init__(self) -> None:
        super().__init__("mono_camera_sim_node")

        default_dir = str(
            Path(get_package_share_directory("avt_vimba_camera")) / "sample_images"
        )
        self.declare_parameter("image_dir", default_dir)
        self.declare_parameter("rate_hz", 0.5)
        self.declare_parameter("frame_id", "ai_camera_optical")
        self.declare_parameter("loop", True)

        image_dir = Path(self.get_parameter("image_dir").get_parameter_value().string_value)
        rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._loop = self.get_parameter("loop").get_parameter_value().bool_value

        if rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz}")

        self._images: List[Path] = self._collect_images(image_dir)
        if not self._images:
            raise FileNotFoundError(
                f"No sample images found in {image_dir} "
                f"(extensions searched: {IMAGE_EXTENSIONS})"
            )
        self.get_logger().info(
            f"mono_camera_sim: publishing {len(self._images)} image(s) "
            f"from {image_dir} at {rate_hz} Hz (loop={self._loop})"
        )

        self._bridge = CvBridge()
        self._index = 0

        self._image_pub = self.create_publisher(Image, "image", 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, "camera_info", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_next)

    @staticmethod
    def _collect_images(image_dir: Path) -> List[Path]:
        if not image_dir.is_dir():
            return []
        return sorted(
            p for p in image_dir.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
        )

    def _publish_next(self) -> None:
        if self._index >= len(self._images):
            if not self._loop:
                self.get_logger().info("Reached end of sample images, loop=false, shutting down timer.")
                self._timer.cancel()
                return
            self._index = 0

        image_path = self._images[self._index]
        self._index += 1

        cv_image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warning(f"Failed to read {image_path}, skipping.")
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = self._bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self._frame_id

        info_msg = self._identity_camera_info(cv_image.shape[1], cv_image.shape[0])
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self._frame_id

        self._image_pub.publish(image_msg)
        self._camera_info_pub.publish(info_msg)
        self.get_logger().info(f"Published {image_path.name} ({cv_image.shape[1]}x{cv_image.shape[0]})")

    @staticmethod
    def _identity_camera_info(width: int, height: int) -> CameraInfo:
        """Camera info with a passthrough (identity-ish) intrinsic.

        Focal length is set to the image width — a sensible default that
        keeps image_proc::RectifyNode happy when it re-projects; distortion
        is zero, so rectify becomes a passthrough for our pre-processed
        sample images.
        """
        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(width)
        fy = float(width)
        cx = width / 2.0
        cy = height / 2.0
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info


def main() -> None:
    rclpy.init()
    node = MonoCameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
