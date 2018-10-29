import cv2
import os
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import Image

from convertions.convertions import numpy_to_image

DIR = os.path.dirname(os.path.realpath(__file__))


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam2ros')
        self.video_capture = cv2.VideoCapture(0)

        qos_profile = qos_profile_default
        qos_profile.depth = 1

        self.publisher = self.create_publisher(Image, 'image', qos_profile = qos_profile)
        freq = 10
        timer_period = 1 / freq
        self.get_logger().info('NODE STARTED')
        self.timer = self.create_timer(timer_period, self.grab_and_convert_frame)

    def grab_and_convert_frame(self):
        # Grab a single frame of video
        ret, frame = self.video_capture.read()
        msg = numpy_to_image(frame, 'rgb8')
        self.publisher.publish(msg)
        return msg

    def stop(self):
        self.video_capture.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    webcam_node = WebcamNode()

    while rclpy.ok():
        webcam_node.grab_and_convert_frame()
        sleep(0.1)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webcam_node.get_logger().info('NODE TERMINATED')
    webcam_node.stop()
    webcam_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

