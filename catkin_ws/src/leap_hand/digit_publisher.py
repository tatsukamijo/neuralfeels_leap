#!/usr/bin/env python3
"""
Publishes RGB images from all connected DIGIT tactile sensors as sensor_msgs/Image on separate ROS topics.
Each device is published to /digit/<index>/image_raw (index is detection order).
If no device can be published, the node shuts down. Devices that fail to open are skipped with a warning.
Warn if any of the expected 4 DIGITs are missing.
"""
import rospy
from sensor_msgs.msg import Image
from digit_interface import Digit, DigitHandler
from cv_bridge import CvBridge
import threading
import time

FRAME_RATE = 30
ENCODING = 'rgb8'
EXPECTED_SERIALS = ['D20262', 'D20219', 'D20236', 'D20221']


def digit_publish_worker(idx, serial, dev_name, stop_event):
    try:
        digit = Digit(serial)
        digit.connect()
        bridge = CvBridge()
        topic = f"/digit/{idx}/image_raw"
        pub = rospy.Publisher(topic, Image, queue_size=1)
        rate = rospy.Rate(FRAME_RATE)
        rospy.loginfo(f"Publishing DIGIT {serial} ({dev_name}) on {topic} at {FRAME_RATE} Hz")
        while not rospy.is_shutdown() and not stop_event.is_set():
            frame = digit.get_frame()
            msg = bridge.cv2_to_imgmsg(frame, encoding=ENCODING)
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
    except Exception as e:
        rospy.logwarn(f"Skipping DIGIT idx={idx}, serial={serial}, dev={dev_name}: {e}")
        stop_event.set()  # Signal to main thread that this device failed
    finally:
        try:
            digit.disconnect()
        except Exception:
            pass


def main():
    rospy.init_node('digit_publisher')
    digits = DigitHandler.list_digits()
    if not digits:
        rospy.logerr("No DIGIT devices found.")
        return
    # Use only the first device for each serial number
    serial_to_dev = {}
    for d in digits:
        serial = d['serial']
        if serial not in serial_to_dev:
            serial_to_dev[serial] = d
    # Assign index in detection order
    selected_digits = list(serial_to_dev.values())
    # Warn if any expected DIGIT serials are missing
    found_serials = set(serial_to_dev.keys())
    for s in EXPECTED_SERIALS:
        if s not in found_serials:
            rospy.logwarn(f"Expected DIGIT serial {s} not found.")
    if not selected_digits:
        rospy.logerr("No usable DIGIT devices found.")
        return
    stop_event = threading.Event()
    threads = []
    started = 0
    for idx, d in enumerate(selected_digits):
        serial = d['serial']
        dev_name = d.get('dev_name', 'unknown')
        t = threading.Thread(target=digit_publish_worker, args=(idx, serial, dev_name, stop_event), daemon=True)
        threads.append(t)
        t.start()
        started += 1
    if started == 0:
        rospy.logerr("No DIGIT publishers started.")
        return
    try:
        while not rospy.is_shutdown():
            if stop_event.is_set():
                rospy.logwarn("At least one DIGIT publisher failed. Others continue.")
                stop_event.clear()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for t in threads:
            t.join()
        rospy.loginfo("digit_publisher node stopped.")

if __name__ == '__main__':
    main() 