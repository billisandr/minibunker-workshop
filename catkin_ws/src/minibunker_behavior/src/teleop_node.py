#!/usr/bin/env python3
# ============================================================================
#  teleop_node.py — WASD keyboard teleop for the MiniBunker rover (plan2.md §4).
#
#  Publishes : /minibunker/teleop_cmd  (geometry_msgs/Twist)  — WASD *intent*
#
#  This node publishes intent ONLY. It is NOT a second /cmd_vel publisher:
#  behavior_node owns /cmd_vel and only honours this intent when
#  mission/follow_item == none (TELEOP), always through the ARM gate + clamps
#  (plan2.md §4.2, §6). So a key press here can never bypass DISARM.
#
#  Keys:  W = forward   S = back   A = turn left   D = turn right
#         space / K = stop      Q / Ctrl-C = quit
#
#  Speeds come from behavior/teleop/{linear_speed,angular_speed} and are re-read
#  every loop, so the UI sliders tune them live. The node publishes continuously
#  at publish_hz so behaviour's teleop watchdog stays fed while moving; releasing
#  to "stop" publishes a zero Twist, and quitting stops publishing so the
#  watchdog zeroes the rover after behavior/teleop/timeout_ms.
#
#  This needs an interactive TTY for raw keyboard capture, so it is run by hand,
#  NOT from roslaunch (which has no stdin):
#     docker exec -it <container> rosrun minibunker_behavior teleop_node.py
#  The Streamlit UI WASD pad is the primary, no-terminal path; this is for power
#  users. See docs/TELEOP.md.
# ============================================================================
import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

# key -> (linear sign, angular sign).  +angular = turn left (CCW), ROS convention.
MOVE_BINDINGS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    " ": (0.0, 0.0),
    "k": (0.0, 0.0),
}

BANNER = """\
MiniBunker WASD teleop  ->  /minibunker/teleop_cmd
  W forward   S back   A turn-left   D turn-right
  space / K stop       Q / Ctrl-C quit
NOTE: the rover only moves when ARMED *and* mission/follow_item == none.
"""


def get_key(timeout):
    """Return a single keypress within `timeout` seconds, or '' if none."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


def main():
    rospy.init_node("minibunker_teleop")
    pub = rospy.Publisher("/minibunker/teleop_cmd", Twist, queue_size=1)
    hz = float(rospy.get_param("behavior/teleop/publish_hz", 20.0))
    period = 1.0 / hz

    settings = termios.tcgetattr(sys.stdin)
    lin_sign, ang_sign = 0.0, 0.0
    rospy.loginfo("[teleop] up; publishing /minibunker/teleop_cmd at %.0f Hz", hz)
    print(BANNER)
    try:
        tty.setraw(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = get_key(period)
            if key:
                k = key.lower()
                if k == "q" or key == "\x03":   # q or Ctrl-C
                    break
                if k in MOVE_BINDINGS:
                    lin_sign, ang_sign = MOVE_BINDINGS[k]
            # re-read speeds each loop so UI sliders tune teleop live
            lin = float(rospy.get_param("behavior/teleop/linear_speed", 0.25))
            ang = float(rospy.get_param("behavior/teleop/angular_speed", 0.8))
            twist = Twist()
            twist.linear.x = lin_sign * lin
            twist.angular.z = ang_sign * ang
            pub.publish(twist)
    finally:
        # restore the terminal and command a final stop (defensive; the watchdog
        # would zero anyway once we stop publishing).
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        pub.publish(Twist())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
