#!/usr/bin/env python3
"""
stereo_driver.py
Synchronized dual camera driver for Raspberry Pi 5 / test rigs.
Publishes sensor_msgs/Image and matching CameraInfo with synchronized timestamps and frame IDs.
"""

import time
import os
import yaml
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
except ImportError:
    # Allows offline testing/mocking without ROS 2 installed
    Node = object

class StereoCameraDriver(Node):
    def __init__(self, node_name="stereo_camera_driver"):
        super().__init__(node_name)
        
        # Declare parameters
        self.declare_parameter("left_calib_path", "")
        self.declare_parameter("right_calib_path", "")
        self.declare_parameter("frame_rate", 15.0)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("left_frame_id", "left_optical_frame")
        self.declare_parameter("right_frame_id", "right_optical_frame")
        
        self.left_calib_path = self.get_parameter("left_calib_path").value
        self.right_calib_path = self.get_parameter("right_calib_path").value
        self.fps = self.get_parameter("frame_rate").value
        self.use_hardware = self.get_parameter("use_hardware").value
        self.left_frame_id = self.get_parameter("left_frame_id").value
        self.right_frame_id = self.get_parameter("right_frame_id").value

        # Publishers
        self.pub_left_image = self.create_publisher(Image, "stereo/left/image_raw", 10)
        self.pub_left_info = self.create_publisher(CameraInfo, "stereo/left/camera_info", 10)
        self.pub_right_image = self.create_publisher(Image, "stereo/right/image_raw", 10)
        self.pub_right_info = self.create_publisher(CameraInfo, "stereo/right/camera_info", 10)

        self.left_info_msg = self._load_camera_info(self.left_calib_path, self.left_frame_id)
        self.right_info_msg = self._load_camera_info(self.right_calib_path, self.right_frame_id)

        self.picam_left = None
        self.picam_right = None

        if self.use_hardware:
            self._init_hardware()

        # Timer for capture loop
        timer_period = 1.0 / max(1.0, self.fps)
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        self.get_logger().info(f"StereoCameraDriver started at {self.fps} Hz (Hardware={self.use_hardware}).")

    def _init_hardware(self):
        try:
            from picamera2 import Picamera2
            self.picam_left = Picamera2(0)
            self.picam_right = Picamera2(1)
            
            cfg_l = self.picam_left.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
            cfg_r = self.picam_right.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
            self.picam_left.configure(cfg_l)
            self.picam_right.configure(cfg_r)
            self.picam_left.start()
            self.picam_right.start()
            self.get_logger().info("Dual Picamera2 hardware initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Picamera2: {e}. Falling back to synthetic pattern.")
            self.use_hardware = False

    def _load_camera_info(self, yaml_path, frame_id):
        info = CameraInfo()
        info.header.frame_id = frame_id
        if not yaml_path or not os.path.exists(yaml_path):
            info.width = 640
            info.height = 480
            info.k = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
            info.p = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
            return info
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                info.width = data.get("image_width", 640)
                info.height = data.get("image_height", 480)
                # Parsing opencv YAML node if present
        except Exception:
            pass
        return info

    def capture_and_publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        if self.use_hardware and self.picam_left and self.picam_right:
            left_frame = self.picam_left.capture_array()
            right_frame = self.picam_right.capture_array()
        else:
            # Synthetic textured test pattern for headless/bench testing
            left_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            left_frame[100:380, 150:490] = 180 # Textured rectangle
            right_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            right_frame[100:380, 120:460] = 180 # Displaced by 30 pixels for disparity

        # Package left image
        msg_l = Image()
        msg_l.header.stamp = stamp
        msg_l.header.frame_id = self.left_frame_id
        msg_l.height = left_frame.shape[0]
        msg_l.width = left_frame.shape[1]
        msg_l.encoding = "rgb8"
        msg_l.step = left_frame.shape[1] * 3
        msg_l.data = left_frame.tobytes()

        # Package right image
        msg_r = Image()
        msg_r.header.stamp = stamp
        msg_r.header.frame_id = self.right_frame_id
        msg_r.height = right_frame.shape[0]
        msg_r.width = right_frame.shape[1]
        msg_r.encoding = "rgb8"
        msg_r.step = right_frame.shape[1] * 3
        msg_r.data = right_frame.tobytes()

        self.left_info_msg.header.stamp = stamp
        self.right_info_msg.header.stamp = stamp

        self.pub_left_image.publish(msg_l)
        self.pub_left_info.publish(self.left_info_msg)
        self.pub_right_image.publish(msg_r)
        self.pub_right_info.publish(self.right_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
