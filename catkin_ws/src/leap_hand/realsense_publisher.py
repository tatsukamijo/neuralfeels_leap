#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

def get_camera_info(color_intrinsics):
    """Create CameraInfo message from RealSense intrinsics."""
    camera_info = CameraInfo()
    camera_info.header.frame_id = "camera_color_optical_frame"
    camera_info.width = color_intrinsics.width
    camera_info.height = color_intrinsics.height
    camera_info.distortion_model = "plumb_bob"
    
    # Intrinsic camera matrix
    camera_info.K = [
        color_intrinsics.fx, 0, color_intrinsics.ppx,
      0, color_intrinsics.fy, color_intrinsics.ppy,
    0, 0, 1 ]
    
    # Distortion coefficients
    camera_info.D = list(color_intrinsics.coeffs)
    
    # Rectification matrix (identity for no rectification)
    camera_info.R = [1, 0, 0, 0, 0, 1]
    
    # Projection matrix
    camera_info.P = [
        color_intrinsics.fx, 0, color_intrinsics.ppx, 0,
      0, color_intrinsics.fy, color_intrinsics.ppy, 0,
       0, 1, 0
    ]
    
    return camera_info

def main():
    rospy.init_node("realsense_publisher")
    bridge = CvBridge()
    
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = pipeline.start(config)
        
        # Get color stream profile for camera info
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        
        # Create camera info message
        camera_info = get_camera_info(color_intrinsics)
        
        align = rs.align(rs.stream.color)

        pub_color = rospy.Publisher("/realsense/color/image_raw", Image, queue_size=1)
        pub_depth = rospy.Publisher("/realsense/aligned_depth_to_color/image_raw", Image, queue_size=1)
        pub_camera_info = rospy.Publisher("/realsense/color/camera_info", CameraInfo, queue_size=1)

        rospy.loginfo("Started RealSense publisher node")
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames()
                aligned = align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                    
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                
                # Convert BGR to RGB for color image
                color_img_rgb = color_img[:, :, ::-1]
                
                color_msg = bridge.cv2_to_imgmsg(color_img_rgb, encoding="rgb8")
                depth_msg = bridge.cv2_to_imgmsg(depth_img, encoding="16UC1")
                
                current_time = rospy.Time.now()
                color_msg.header.stamp = current_time
                depth_msg.header.stamp = current_time
                camera_info.header.stamp = current_time
                
                # Set frame IDs
                color_msg.header.frame_id = "camera_color_optical_frame"
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                camera_info.header.frame_id = "camera_color_optical_frame"
                
                pub_color.publish(color_msg)
                pub_depth.publish(depth_msg)
                pub_camera_info.publish(camera_info)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logwarn(f"Error processing frames: {e}")
                continue
                
    except Exception as e:
        rospy.logerr(f"Failed to initialize RealSense: {e}")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
