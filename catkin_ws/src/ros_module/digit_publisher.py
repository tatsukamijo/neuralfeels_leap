#!/usr/bin/env python3
"""
Publishes RGB images from a DIGIT tactile sensor as sensor_msgs/Image on a ROS topic.
"""
import rospy
from sensor_msgs.msg import Image
from digit_interface import Digit
import cv2
from cv_bridge import CvBridge


def main():
    rospy.init_node('digit_publisher')
    serial_number = rospy.get_param('~serial_number', 'D20262')  # デフォルト値は適宜変更
    topic = rospy.get_param('~topic', '/digit/image_raw')
    frame_rate = rospy.get_param('~frame_rate', 30)

    digit = Digit(serial_number)
    digit.connect()
    bridge = CvBridge()
    pub = rospy.Publisher(topic, Image, queue_size=1)
    rate = rospy.Rate(frame_rate)

    rospy.loginfo(f"Publishing DIGIT images on {topic} at {frame_rate} Hz (serial: {serial_number})")
    try:
        while not rospy.is_shutdown():
            frame = digit.get_frame()
            # frame: np.ndarray, shape (H, W, 3), dtype=uint8
            msg = bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        digit.disconnect()

if __name__ == '__main__':
    main() 