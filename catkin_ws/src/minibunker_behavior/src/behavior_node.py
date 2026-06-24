#!/usr/bin/env python3
# ============================================================================
#  behavior_node.py — reactive space-mining state machine (plan.md §4.4).
#
#  Subscribes : /minibunker/perception_state (std_msgs/Float32MultiArray)
#               /minibunker/arm              (std_msgs/Bool)  ARM/DISARM gate
#  Publishes  : /cmd_vel                     (geometry_msgs/Twist)
#               /minibunker/state            (std_msgs/String) current FSM state
#
#  States:  SEARCH -> APPROACH (green ball) -> AVOID (cone) -> STOP/COLLECT
#  Distance to the ball is proxied by its bbox height fraction (no depth).
#
#  SAFETY: boots DISARMED. While disarmed it publishes a zero Twist every tick
#  so the robot never lurches and any latched command is overridden. All speeds
#  are clamped to behavior/limits. The ARM gate mirrors the "start frozen"
#  posture of the z1 stations.
#
#  perception_state layout (must match detector_node.py):
#     [0] target_seen  [1] target_cx_norm  [2] target_cy_norm
#     [3] target_h_frac  [4] cone_seen  [5] cone_danger  [6] cone_cx_norm
# ============================================================================
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String

SEARCH, APPROACH, AVOID, COLLECT, STOP = "SEARCH", "APPROACH", "AVOID", "COLLECT", "STOP"


class BehaviorNode:
    def __init__(self):
        self.armed = bool(rospy.get_param("behavior/arm_on_start", False))
        self.state = SEARCH
        self.last_seen_ticks = 0
        self.collect_hold = 0
        self.ps = [0.0] * 7          # latest perception_state
        self.have_ps = False

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/minibunker/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/minibunker/perception_state", Float32MultiArray, self.on_state, queue_size=1)
        rospy.Subscriber("/minibunker/arm", Bool, self.on_arm, queue_size=1)

        hz = float(rospy.get_param("behavior/rate_hz", 15.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self.tick)
        rospy.loginfo("[behavior] up; armed=%s (publish /minibunker/arm to toggle)", self.armed)

    # -- callbacks -----------------------------------------------------------
    def on_state(self, msg):
        if len(msg.data) >= 7:
            self.ps = list(msg.data)
            self.have_ps = True

    def on_arm(self, msg):
        self.armed = bool(msg.data)
        rospy.loginfo("[behavior] %s", "ARMED" if self.armed else "DISARMED")
        if not self.armed:
            self.state = STOP

    # -- helpers -------------------------------------------------------------
    def _params(self):
        g = rospy.get_param
        return dict(
            scan_w=float(g("behavior/search/scan_angular_speed", 0.5)),
            steer_gain=float(g("behavior/approach/steer_gain", 0.8)),
            fwd=float(g("behavior/approach/forward_speed", 0.25)),
            collect_frac=float(g("behavior/approach/collect_bbox_frac", 0.45)),
            backoff=float(g("behavior/avoid/backoff_speed", -0.15)),
            turn=float(g("behavior/avoid/turn_speed", 0.6)),
            max_lin=float(g("behavior/limits/max_linear", 0.4)),
            max_ang=float(g("behavior/limits/max_angular", 1.0)),
            lost=int(g("behavior/limits/lost_frames", 15)),
        )

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # -- main loop -----------------------------------------------------------
    def tick(self, _evt):
        twist = Twist()

        if not self.armed or not self.have_ps:
            self._emit(STOP, twist)
            return

        p = self._params()
        target_seen = self.ps[0] > 0.5
        cx = self.ps[1]
        h_frac = self.ps[3]
        cone_danger = self.ps[5] > 0.5
        cone_cx = self.ps[6]

        if target_seen:
            self.last_seen_ticks = 0
        else:
            self.last_seen_ticks += 1

        # ---- transition + control ----
        if cone_danger:
            # AVOID has priority: back off and turn away from the cone side.
            self.state = AVOID
            twist.linear.x = p["backoff"]
            twist.angular.z = -p["turn"] if cone_cx >= 0 else p["turn"]

        elif self.collect_hold > 0:
            self.state = COLLECT
            self.collect_hold -= 1
            # stay stopped a moment at the "ore", then resume searching
            if self.collect_hold == 0:
                self.state = SEARCH

        elif target_seen and h_frac >= p["collect_frac"]:
            # close enough -> collect (stop and hold)
            self.state = COLLECT
            self.collect_hold = int(rospy.get_param("behavior/approach/collect_hold_ticks", 20))

        elif target_seen:
            self.state = APPROACH
            twist.angular.z = -p["steer_gain"] * cx     # P-control on image-x error
            twist.linear.x = p["fwd"] * (1.0 - min(0.8, h_frac))  # ease off as it nears

        elif self.last_seen_ticks <= p["lost"]:
            # briefly keep the last intent before giving up
            self.state = SEARCH
            twist.angular.z = p["scan_w"]

        else:
            self.state = SEARCH
            twist.angular.z = p["scan_w"]

        twist.linear.x = self._clamp(twist.linear.x, -p["max_lin"], p["max_lin"])
        twist.angular.z = self._clamp(twist.angular.z, -p["max_ang"], p["max_ang"])
        self._emit(self.state, twist)

    def _emit(self, state, twist):
        self.state = state
        self.cmd_pub.publish(twist)
        self.state_pub.publish(String(data=state))


def main():
    rospy.init_node("minibunker_behavior")
    BehaviorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
