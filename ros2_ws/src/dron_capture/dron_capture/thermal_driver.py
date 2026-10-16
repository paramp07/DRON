#!/usr/bin/env python3
"""
thermal_driver.py
Meridian Innovation MI48 radiometric thermal camera driver.
Publishes calibrated radiometric temperature matrix as sensor_msgs/Image (32FC1 in Celsius)
along with matching sensor_msgs/CameraInfo and timestamp metadata.
"""

import os
import sys
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
except ImportError:
    Node = object

class ThermalCameraDriver(Node):
    def __init__(self, node_name="thermal_camera_driver"):
        super().__init__(node_name)

        self.declare_parameter("device_port", "/dev/ttyACM0")
        self.declare_parameter("frame_rate", 9.0)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("frame_id", "thermal_optical_frame")
        self.declare_parameter("calib_path", "")

        self.port = self.get_parameter("device_port").value
        self.fps = self.get_parameter("frame_rate").value
        self.use_hardware = self.get_parameter("use_hardware").value
        self.frame_id = self.get_parameter("frame_id").value

        # Publishers
        self.pub_thermal_image = self.create_publisher(Image, "thermal/image_temperature", 10)
        self.pub_thermal_info = self.create_publisher(CameraInfo, "thermal/camera_info", 10)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = self.frame_id
        self.camera_info_msg.width = 80
        self.camera_info_msg.height = 62
        self.camera_info_msg.k = [96.5, 0.0, 39.5, 0.0, 96.5, 30.5, 0.0, 0.0, 1.0]

        self.mi48 = None
        if self.use_hardware:
            self._init_hardware()

        timer_period = 1.0 / max(1.0, self.fps)
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        self.get_logger().info(f"ThermalCameraDriver started at {self.fps} Hz (Hardware={self.use_hardware}).")

    def _init_hardware(self):
        try:
            from senxor.mi48 import MI48
            from senxor.interfaces import get_serial, USB_Interface
            try:
                ser = get_serial()
            except Exception:
                import serial
                ser = serial.Serial(self.port)
            usb = USB_Interface(ser)
            self.mi48 = MI48([usb, usb])
            self.mi48.set_fps(int(self.fps))
            self.mi48.start(stream=True, with_header=True)
            self.get_logger().info("MI48 thermal sensor initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MI48: {e}. Falling back to synthetic pattern.")
            self.use_hardware = False

    def capture_and_publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        if self.use_hardware and self.mi48:
            try:
                data, header = self.mi48.read()
                if data is not None:
                    from senxor.utils import data_to_frame
                    temp_matrix = data_to_frame(data, self.mi48.fpa_shape).astype(np.float32)
                else:
                    return
            except Exception as e:
                self.get_logger().warn(f"Error reading MI48: {e}")
                return
        else:
            # Synthetic radiometric heat pattern (ambient 22°C with a 75°C hotspot)
            temp_matrix = np.full((62, 80), 22.5, dtype=np.float32)
            # Add warm circular hotspot at (x=45, y=30)
            yy, xx = np.ogrid[:62, :80]
            dist_sq = (xx - 45)**2 + (yy - 30)**2
            hotspot = np.exp(-dist_sq / 50.0) * 55.0 # Up to 77.5°C
            temp_matrix += hotspot.astype(np.float32)

        # Build sensor_msgs/Image (32FC1: 32-bit float in Celsius)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = temp_matrix.shape[0]
        msg.width = temp_matrix.shape[1]
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = temp_matrix.shape[1] * 4 # 4 bytes per float32
        msg.data = temp_matrix.tobytes()

        self.camera_info_msg.header.stamp = stamp

        self.pub_thermal_image.publish(msg)
        self.pub_thermal_info.publish(self.camera_info_msg)

    def destroy_node(self):
        if self.mi48:
            try:
                self.mi48.stop()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
