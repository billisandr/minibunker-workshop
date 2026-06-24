#!/usr/bin/env python3
# ============================================================================
#  image_pub_node.py — hardware-free image source for /camera/image_raw.
#
#  Lets the whole detector+behaviour stack run with NO Gazebo and NO camera.
#  Source is chosen by the camera/source param (plan.md §5, §9.3):
#     webcam            -> default webcam (cv2.VideoCapture(0))
#     video:/path.mp4   -> loop a bundled demo clip
#     synthetic         -> procedurally draw a moving green ball + orange cone
#                          (zero assets — useful for CI / a laptop with nothing)
#
#  In SIM the Gazebo camera plugin already publishes /camera/image_raw, so this
#  node is NOT launched there. It is for the hardware-free dev / demo path.
# ============================================================================
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class SyntheticScene:
    """Draws a green ball that drifts and an orange cone, on a dark 'arena'."""

    def __init__(self, w, h):
        self.w, self.h, self.t = w, h, 0.0

    def frame(self):
        self.t += 0.03
        img = np.full((self.h, self.w, 3), 40, dtype=np.uint8)
        cv2.rectangle(img, (0, int(self.h * 0.7)), (self.w, self.h), (60, 60, 60), -1)
        # green ball — sweeps left/right, grows as if approaching
        bx = int(self.w * (0.5 + 0.3 * np.sin(self.t)))
        by = int(self.h * 0.55)
        br = int(28 + 14 * (1 + np.sin(self.t * 0.5)))
        cv2.circle(img, (bx, by), br, (40, 210, 40), -1)
        # orange cone — fixed, lower-right
        cx, cy = int(self.w * 0.72), int(self.h * 0.72)
        pts = np.array([[cx, cy - 60], [cx - 35, cy + 30], [cx + 35, cy + 30]])
        cv2.drawContours(img, [pts], 0, (40, 130, 240), -1)
        return img


def open_capture(source):
    if source.startswith("video:"):
        path = source.split("video:", 1)[1]
        cap = cv2.VideoCapture(path)
        return cap, ("video", path)
    if source == "webcam" or source.isdigit():
        idx = int(source) if source.isdigit() else 0
        return cv2.VideoCapture(idx), ("webcam", idx)
    return None, ("synthetic", None)


def main():
    rospy.init_node("minibunker_image_pub")
    bridge = CvBridge()
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

    source = rospy.get_param("camera/source", "synthetic")
    w = int(rospy.get_param("camera/width", 640))
    h = int(rospy.get_param("camera/height", 480))
    fps = float(rospy.get_param("camera/fps", 30))

    cap, (kind, ref) = open_capture(source)
    synth = SyntheticScene(w, h) if cap is None else None
    if cap is not None and not cap.isOpened():
        rospy.logwarn("[image_pub] could not open %s '%s' — using synthetic scene",
                      kind, ref)
        cap, synth = None, SyntheticScene(w, h)
    rospy.loginfo("[image_pub] source=%s (%s)", source, kind)

    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        if synth is not None:
            frame = synth.frame()
        else:
            ok, frame = cap.read()
            if not ok:  # loop video files
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            frame = cv2.resize(frame, (w, h))
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_link"
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
