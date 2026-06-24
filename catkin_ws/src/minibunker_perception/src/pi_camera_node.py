#!/usr/bin/env python3
# ============================================================================
#  pi_camera_node.py — REAL-robot camera source -> /camera/image_raw.
#
#  Same output topic the Gazebo camera plugin publishes in sim, so the detector
#  and behaviour nodes are identical across sim and real (plan.md §3.1, §8.3).
#
#  Tries picamera2 (libcamera) first; if unavailable (e.g. running in a
#  container without the libcamera stack, or a USB webcam is used instead),
#  falls back to a plain V4L2 cv2.VideoCapture. See docs/HARDWARE_SETUP.md
#  §Camera for the Docker passthrough options.
# ============================================================================
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def try_picamera2(w, h):
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (w, h), "format": "RGB888"})
        cam.configure(cfg)
        cam.start()
        rospy.loginfo("[pi_camera] using picamera2 (libcamera)")
        return cam
    except Exception as exc:  # noqa: BLE001
        rospy.logwarn("[pi_camera] picamera2 unavailable (%s); trying V4L2", exc)
        return None


def main():
    rospy.init_node("minibunker_pi_camera")
    bridge = CvBridge()
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

    w = int(rospy.get_param("camera/width", 640))
    h = int(rospy.get_param("camera/height", 480))
    fps = float(rospy.get_param("camera/fps", 30))
    flip = bool(rospy.get_param("camera/flip_horizontal", False))
    dev = rospy.get_param("camera/v4l2_device", "/dev/video0")

    cam = try_picamera2(w, h)
    cap = None
    if cam is None:
        cap = cv2.VideoCapture(dev)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            rospy.logerr("[pi_camera] no camera (picamera2 nor %s). Exiting.", dev)
            return
        rospy.loginfo("[pi_camera] using V4L2 %s", dev)

    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        if cam is not None:
            rgb = cam.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            ok, frame = cap.read()
            if not ok:
                rospy.logwarn_throttle(5.0, "[pi_camera] frame grab failed")
                rate.sleep()
                continue
        if flip:
            frame = cv2.flip(frame, 1)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_link"
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
