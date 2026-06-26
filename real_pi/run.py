#!/usr/bin/env python3
# ============================================================================
#  run.py — the native MiniBunker station: ONE loop, no ROS, no Docker.
#
#      camera -> detector -> perception_state -> FSM -> clamp -> CAN(0x111)
#
#  This is the no-ROS path of plan.md §9.4. It is the real-robot equivalent of
#  minibunker_real.launch (pi_camera + detector + behavior + bunker_base), run
#  as a single Python process on the Pi.
#
#  SAFETY (mirrors the ROS station, non-negotiable):
#    * Boots DISARMED: no motion frame is sent until you ARM (key 'a' / --arm).
#    * Hard clamp = min(behavior/limits, can/hw_max) before the wire.
#    * Watchdog: a slow/failed frame still ticks the loop and sends zero.
#    * Ctrl-C / any exit -> stop() (zero Twist) + set_standby() (release CAN).
#  Keep the hardware e-stop in hand; run inside the fenced arena only.
#
#  Usage:
#    python3 run.py                      # DISARMED; type 'a'<Enter> to ARM
#    python3 run.py --can vcan0          # dry-run on a virtual CAN bus
#    python3 run.py --no-can             # perception-only (no bus at all)
#    python3 run.py --headless           # no debug window (saves CPU on the Pi)
#    python3 run.py --save-frames out/   # dump annotated frames instead of a window
# ============================================================================
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import cv2

from minibunker_real.config import Config
from minibunker_real.camera import Camera
from minibunker_real.detector import make_detector
from minibunker_real import perception_state as psmod
from minibunker_real.fsm import BehaviorFSM


def parse_args():
    ap = argparse.ArgumentParser(description="Native MiniBunker station")
    ap.add_argument("--config", default=None, help="path to config.yaml")
    ap.add_argument("--can", default=None, help="override can/channel (e.g. vcan0)")
    ap.add_argument("--no-can", action="store_true",
                    help="perception/FSM only; never open a CAN bus")
    ap.add_argument("--arm", action="store_true",
                    help="UNSAFE: boot ARMED (skips the DISARMED default)")
    ap.add_argument("--headless", action="store_true", help="no debug window")
    ap.add_argument("--save-frames", default=None,
                    help="dir to write annotated frames into (implies headless)")
    return ap.parse_args()


class Keyboard:
    """Non-blocking stdin reader (daemon thread) for ARM/DISARM, WASD teleop, quit.

      a = ARM    d = DISARM    q = quit
      w/s = fwd/back   j/l = turn left/right   x = stop   (when mission == none)

    Line-buffered (each key + Enter), so it works over SSH. There is no key-release
    event, so a WASD press sets a *fresh* teleop intent that AUTO-EXPIRES via the
    FSM watchdog (behavior/teleop/timeout_ms): the rover drives for that window
    then stops unless the key is pressed again. 'x' is an explicit zero. This is
    the single motion owner — teleop is in-process, never a second sender on CAN.
    """
    def __init__(self, lin_speed=0.25, ang_speed=0.8):
        self.armed = False
        self.quit = False
        self.lin_speed = lin_speed
        self.ang_speed = ang_speed
        self._intent = (0.0, 0.0, 0.0)     # (linear, angular, stamp)
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def teleop(self):
        return self._intent

    def _set_intent(self, lin, ang):
        self._intent = (lin, ang, time.monotonic())

    def _loop(self):
        for line in sys.stdin:
            c = line.strip().lower()[:1]
            if c == "a":
                self.armed = True
                print("[run] >>> ARMED")
            elif c == "d":
                self.armed = False
                print("[run] >>> DISARMED")
            elif c == "q":
                self.quit = True
                return
            elif c == "w":
                self._set_intent(self.lin_speed, 0.0)
            elif c == "s":
                self._set_intent(-self.lin_speed, 0.0)
            elif c == "j":
                self._set_intent(0.0, self.ang_speed)
            elif c == "l":
                self._set_intent(0.0, -self.ang_speed)
            elif c == "x":
                self._set_intent(0.0, 0.0)


def main():
    args = parse_args()
    cfg = Config(args.config)

    # --- perception ---
    cam = Camera(cfg)
    backend_name, detector = make_detector(cfg.block("detector"))
    fsm = BehaviorFSM(cfg.block("behavior"))
    cone_danger = float(cfg.get("behavior/avoid/cone_danger_frac", 0.35))

    # --- CAN (optional / overridable) ---
    bunker = None
    if not args.no_can:
        from minibunker_real.bunker_can import BunkerCAN
        channel = args.can or cfg.get("can/channel", "can0")
        bunker = BunkerCAN(
            channel=channel,
            interface=cfg.get("can/interface", "socketcan"),
            hw_max_linear=float(cfg.get("can/hw_max_linear", 1.5)),
            hw_max_angular=float(cfg.get("can/hw_max_angular", 0.7853)))
        print(f"[run] CAN up on {channel}")
    else:
        print("[run] --no-can: perception/FSM only (no motion)")

    kb = Keyboard(
        lin_speed=float(cfg.get("behavior/teleop/linear_speed", 0.25)),
        ang_speed=float(cfg.get("behavior/teleop/angular_speed", 0.8)))
    kb.armed = bool(args.arm or cfg.get("behavior/arm_on_start", False))
    if kb.armed:
        print("[run] WARNING: booting ARMED")

    rate = float(cfg.get("behavior/rate_hz", 20.0))
    period = 1.0 / rate
    enable_can_mode = bool(cfg.get("can/enable_can_mode", True))
    was_armed = False

    save_dir = args.save_frames
    headless = args.headless or bool(save_dir)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
    frame_i = 0

    print("[run] DISARMED. Keys (+Enter): a=ARM d=DISARM q=quit | "
          "teleop (mission=none): w/s=fwd/back j/l=turn x=stop")
    try:
        while not kb.quit:
            t0 = time.monotonic()

            ok, frame = cam.read()
            if not ok:
                print("[run] camera read failed; sending zero")
                if bunker:
                    bunker.stop()
                time.sleep(period)
                continue
            h, w = frame.shape[:2]

            # mission re-read each tick (edit config.yaml live? no — read once at
            # load; live-switch is a UI feature we don't carry to the native path)
            target_cls, hazard_classes = psmod.resolve_mission(cfg.block("mission"))
            follow = cfg.get("mission/follow_item", "none")

            dets, _mask = detector.detect(frame)
            ps = psmod.pack(dets, w, h, target_cls, hazard_classes, cone_danger)

            # ARM edge handling: on ARM, (re)enable CAN mode so the base listens.
            if kb.armed and not was_armed and bunker and enable_can_mode:
                bunker.enable_can_mode()
                print("[run] sent CONTROL_MODE_CAN")
            if was_armed and not kb.armed:
                fsm.on_disarm()
            was_armed = kb.armed

            lin, ang, state = fsm.step(ps, kb.armed, follow, kb.teleop(), t0)

            if bunker:
                if kb.armed:
                    bunker.send_motion(lin, ang)   # clamps to HW ceiling inside
                else:
                    bunker.stop()
                bunker.poll()                       # drain state frames

            # --- debug viz ---
            if not headless:
                annotated = psmod.annotate(frame, dets, ps, backend_name,
                                           target_cls, state)
                _draw_status(annotated, kb.armed, lin, ang,
                             bunker.state if bunker else None)
                cv2.imshow("minibunker (real)", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            elif save_dir and frame_i % 5 == 0:
                annotated = psmod.annotate(frame, dets, ps, backend_name,
                                           target_cls, state)
                cv2.imwrite(os.path.join(save_dir, f"f{frame_i:05d}.jpg"), annotated)
            frame_i += 1

            # keep the loop at rate_hz (also paces the CAN motion frames)
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[run] Ctrl-C")
    finally:
        print("[run] shutting down -> zero motion + STANDBY")
        if bunker:
            bunker.shutdown()
        cam.close()
        if not headless:
            cv2.destroyAllWindows()


def _draw_status(img, armed, lin, ang, state):
    txt = "%s  v=%.2f w=%.2f" % ("ARMED" if armed else "DISARMED", lin, ang)
    colour = (0, 0, 255) if armed else (0, 200, 0)
    cv2.putText(img, txt, (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, colour, 2)
    if state is not None and state.control_mode >= 0:
        estop = " ESTOP" if state.estop_engaged else ""
        cv2.putText(img, "batt=%.1fV mode=%d err=%d%s" % (
            state.battery_voltage, state.control_mode, state.error_code, estop),
            (8, img.shape[0] - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 165, 255) if estop else (255, 255, 0), 1)


if __name__ == "__main__":
    main()
