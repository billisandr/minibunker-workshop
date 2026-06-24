#!/usr/bin/env python3
# ============================================================================
#  behavior_node.py — reactive space-mining state machine (plan.md §4.4).
#
#  Subscribes : /minibunker/perception_state (std_msgs/Float32MultiArray)
#               /minibunker/arm              (std_msgs/Bool)  ARM/DISARM gate
#               /minibunker/teleop_cmd       (geometry_msgs/Twist) WASD intent (TELEOP)
#  Publishes  : /cmd_vel                     (geometry_msgs/Twist)
#               /minibunker/state            (std_msgs/String) current FSM state
#
#  Mission (mission/follow_item, re-read every tick so the UI flips it live):
#     ball | cone -> autonomous follow: SEARCH -> APPROACH -> AVOID -> COLLECT/STOP
#     none        -> TELEOP: yield /cmd_vel to WASD intent on /minibunker/teleop_cmd
#  The followed class is whatever the detector packs into the target_* slots, so
#  the same FSM follows either a ball or a cone (plan2.md §3).
#  Distance to the target is proxied by its bbox height fraction (no depth).
#
#  SAFETY: behaviour is the SINGLE owner of /cmd_vel. Autonomous follow, teleop
#  pass-through and the DISARM zero-Twist all flow through the ONE ARM gate +
#  behavior/limits clamp, so DISARM is always authoritative and there is never a
#  second publisher racing on /cmd_vel (plan2.md §6). Boots DISARMED. Teleop is
#  watchdogged: if no teleop_cmd arrives within behavior/teleop/timeout_ms it
#  commands zero rather than latching the last motion.
#
#  perception_state layout (ROLE-BASED, must match detector_node.py):
#     [0] target_seen  [1] target_cx_norm  [2] target_cy_norm
#     [3] target_h_frac  [4] hazard_seen  [5] hazard_danger  [6] hazard_cx_norm
# ============================================================================
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String

SEARCH, APPROACH, AVOID, COLLECT, STOP, TELEOP = (
    "SEARCH", "APPROACH", "AVOID", "COLLECT", "STOP", "TELEOP")


class BehaviorNode:
    def __init__(self):
        self.armed = bool(rospy.get_param("behavior/arm_on_start", False))
        self.state = SEARCH
        self.last_seen_ticks = 0
        self.collect_hold = 0
        self.ps = [0.0] * 7          # latest perception_state
        self.have_ps = False
        self.teleop = Twist()        # latest WASD intent (used only in TELEOP)
        self.teleop_stamp = rospy.Time(0)   # when it arrived (for the watchdog)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/minibunker/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/minibunker/perception_state", Float32MultiArray, self.on_state, queue_size=1)
        rospy.Subscriber("/minibunker/arm", Bool, self.on_arm, queue_size=1)
        rospy.Subscriber("/minibunker/teleop_cmd", Twist, self.on_teleop, queue_size=1)

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

    def on_teleop(self, msg):
        # Store the WASD intent; tick() decides whether to honour it (TELEOP +
        # ARMED + fresh) and always passes it through the same clamp + ARM gate.
        self.teleop = msg
        self.teleop_stamp = rospy.Time.now()

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

        if not self.armed:
            self._emit(STOP, twist)
            return

        # mission/follow_item is re-read each tick so the UI can switch live.
        follow = str(rospy.get_param("mission/follow_item", "none")).lower()
        if follow == "none":
            self._tick_teleop()       # yield /cmd_vel to WASD intent
            return

        if not self.have_ps:
            self._emit(STOP, twist)
            return

        p = self._params()
        target_seen = self.ps[0] > 0.5
        cx = self.ps[1]
        h_frac = self.ps[3]
        hazard_danger = self.ps[5] > 0.5
        hazard_cx = self.ps[6]

        if target_seen:
            self.last_seen_ticks = 0
        else:
            self.last_seen_ticks += 1

        # ---- transition + control ----
        if hazard_danger:
            # AVOID has priority: back off and turn away from the hazard side.
            self.state = AVOID
            twist.linear.x = p["backoff"]
            twist.angular.z = -p["turn"] if hazard_cx >= 0 else p["turn"]

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

    # -- teleop pass-through (mission/follow_item == none) -------------------
    def _tick_teleop(self):
        """Republish the latest WASD intent through the SAME ARM gate + clamp.

        Caller has already verified we are ARMED. A watchdog drops to zero if no
        teleop_cmd has arrived within behavior/teleop/timeout_ms, so motion is
        never latched when the operator releases the keys or the link drops.
        """
        g = rospy.get_param
        twist = Twist()
        timeout = float(g("behavior/teleop/timeout_ms", 400)) / 1000.0
        fresh = (rospy.Time.now() - self.teleop_stamp).to_sec() <= timeout
        if fresh:
            twist.linear.x = self.teleop.linear.x
            twist.angular.z = self.teleop.angular.z
        # else: stale/no input -> zero Twist (watchdog)
        max_lin = float(g("behavior/limits/max_linear", 0.4))
        max_ang = float(g("behavior/limits/max_angular", 1.0))
        twist.linear.x = self._clamp(twist.linear.x, -max_lin, max_lin)
        twist.angular.z = self._clamp(twist.angular.z, -max_ang, max_ang)
        self._emit(TELEOP, twist)

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
