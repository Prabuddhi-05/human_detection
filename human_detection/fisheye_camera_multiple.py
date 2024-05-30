#!/usr/bin/env python

import subprocess #
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyudev
import time

class FisheyeCameraNode(Node):
    def __init__(self):
        super().__init__('fisheye_camera_node')
        self.serial_no = self.declare_parameter('serial_no').value
        self.video_device = None
        self.find_video_number(self.serial_no)
        if self.video_device is None:
            self.get_logger().error(f"Error: H264 format not found on any device with serial number {self.serial_no}.")
            return

        self.get_logger().info(f"Found camera device at /dev/video{self.video_device} with serial number {self.serial_no}")

        self.topic_name = f"/fisheye_image_{self.serial_no}"
        self.publisher = self.create_publisher(Image, self.topic_name, 10)

        command = f"ffmpeg -framerate 60 -i /dev/video{self.video_device} -pix_fmt bgr24 -f rawvideo -"
        self.ffmpeg_process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01, self.publish_video)

    def find_video_number(self, serial_number):
        max_video_number = 50  # Increase the range of video device numbers to search
        for video_number in range(max_video_number):
            try:
                if self.getSerialNumber(video_number) == serial_number:
                    command = f"v4l2-ctl --list-formats-ext -d {video_number}"
                    result = subprocess.run(command, shell=True, capture_output=True, text=True)
                    if "H264" in result.stdout:
                        self.video_device = video_number
                        return
            except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError):
                continue
            except Exception as e:
                self.get_logger().error(f"Error finding video device: {e}")

        self.get_logger().warning(f"Video device not found for serial number {serial_number} after checking up to /dev/video{max_video_number - 1}")

    def getSerialNumber(self, device=0):
        context = pyudev.Context()
        device_file = "/dev/video{}".format(device)
        try:
            device = pyudev.Devices.from_device_file(context, device_file)
            info = {item[0]: item[1] for item in device.items()}
            return info.get("ID_SERIAL_SHORT")
        except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError):
            return None

    def publish_video(self):
        if self.video_device is None:
            return

        raw_frame = self.ffmpeg_process.stdout.read(1920 * 1080 * 3)
        if len(raw_frame) != 1920 * 1080 * 3:
            return

        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((1080, 1920, 3))
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

