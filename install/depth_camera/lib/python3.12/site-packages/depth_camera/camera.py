#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


class D455Publisher(Node):
    def __init__(self):
        super().__init__('d455_publisher_node')

        # ---- Parameters ----
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('mount_frame', 'camera_mount_link')
        self.declare_parameter('upside_down', True)  # flip via param if you ever remount it
        self.declare_parameter('depth_colormap', 'jet')     # jet, turbo, viridis, bone, hot, inferno
        self.declare_parameter('depth_max_meters', 4.0)     # anything farther clips to the top color

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.mount_frame = self.get_parameter('mount_frame').value
        self.upside_down = self.get_parameter('upside_down').value
        self.depth_max_meters = self.get_parameter('depth_max_meters').value

        colormap_name = self.get_parameter('depth_colormap').value.lower()
        colormap_options = {
            'jet': cv2.COLORMAP_JET,
            'turbo': cv2.COLORMAP_TURBO,
            'viridis': cv2.COLORMAP_VIRIDIS,
            'bone': cv2.COLORMAP_BONE,
            'hot': cv2.COLORMAP_HOT,
            'inferno': cv2.COLORMAP_INFERNO,
        }
        if colormap_name not in colormap_options:
            self.get_logger().warn(
                f"Unknown depth_colormap '{colormap_name}', falling back to 'jet'. "
                f"Options: {list(colormap_options.keys())}"
            )
            colormap_name = 'jet'
        self.depth_colormap = colormap_options[colormap_name]

        self.bridge = CvBridge()

        # ---- RealSense pipeline setup ----
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        try:
            profile = self.pipeline.start(config)
        except RuntimeError as e:
            self.get_logger().error(f'Failed to start RealSense pipeline: {e}')
            raise

        # Align depth to color viewpoint so pixel (u,v) matches in both streams
        self.align = rs.align(rs.stream.color)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.color_intrinsics = color_stream.get_intrinsics()
        self.depth_intrinsics = depth_stream.get_intrinsics()

        # Depth scale (to know units if you need meters later)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f'Depth scale: {self.depth_scale} m/unit')

        # ---- QoS: sensor data, best-effort, keep last ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- Publishers ----
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos)
        self.depth_color_pub = self.create_publisher(Image, '/camera/depth/image_colorized', qos)

        # ---- Static TF: mount_frame -> camera_optical_link (180 deg about Z) ----
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_mount_tf()

        # Pre-build CameraInfo messages (they don't change frame to frame)
        self.color_info_msg = self._build_camera_info(self.color_intrinsics, self.camera_frame)
        self.depth_info_msg = self._build_camera_info(self.depth_intrinsics, self.camera_frame)

        # ---- Timer loop ----
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info('D455 publisher started (upside_down=%s)' % self.upside_down)

    def _broadcast_mount_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.mount_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        if self.upside_down:
            # 180 deg rotation about Z (roll axis of the optical frame when
            # the camera is flipped top-to-bottom on its mount)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 1.0  # sin(pi/2) for a 180 deg turn -> qz=1, qw=0
            t.transform.rotation.w = 0.0
        else:
            t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def _build_camera_info(self, intr, frame_id):
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = intr.width
        msg.height = intr.height

        fx, fy = intr.fx, intr.fy
        cx, cy = intr.ppx, intr.ppy

        if self.upside_down:
            # A 180-degree rotation of the image maps (u,v) -> (W-1-u, H-1-v),
            # so the principal point moves accordingly. fx, fy, distortion
            # are unchanged since the rotation is about the optical axis.
            cx = intr.width - 1 - cx
            cy = intr.height - 1 - cy

        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.d = list(intr.coeffs)
        msg.distortion_model = 'plumb_bob'
        return msg

    def _colorize_depth(self, depth_image_mm):
        """Map a 16-bit mm depth image to an 8-bit BGR image for visualization."""
        max_mm = self.depth_max_meters * 1000.0
        # Clip so anything beyond depth_max_meters saturates at the top color
        # instead of stretching the colormap across noise/out-of-range values.
        clipped = np.clip(depth_image_mm, 0, max_mm)
        scaled = (clipped / max_mm * 255.0).astype(np.uint8)
        colorized = cv2.applyColorMap(scaled, self.depth_colormap)
        # Zero-depth pixels (no return) are usually noise, not "very close" —
        # force them to black so they don't show as the colormap's low-end color.
        colorized[depth_image_mm == 0] = 0
        return colorized

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            self.get_logger().warn(f'Frame wait timed out: {e}')
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())  # 16-bit, mm

        if self.upside_down:
            color_image = cv2.rotate(color_image, cv2.ROTATE_180)
            depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)

        now = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = now
        color_msg.header.frame_id = self.camera_frame

        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = self.camera_frame

        depth_color_image = self._colorize_depth(depth_image)
        depth_color_msg = self.bridge.cv2_to_imgmsg(depth_color_image, encoding='bgr8')
        depth_color_msg.header.stamp = now
        depth_color_msg.header.frame_id = self.camera_frame

        self.color_info_msg.header.stamp = now
        self.depth_info_msg.header.stamp = now

        self.color_pub.publish(color_msg)
        self.color_info_pub.publish(self.color_info_msg)
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self.depth_info_msg)
        self.depth_color_pub.publish(depth_color_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D455Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()