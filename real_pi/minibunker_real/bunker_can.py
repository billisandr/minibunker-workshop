#!/usr/bin/env python3
# ============================================================================
#  bunker_can.py — native AgileX protocol-v2 CAN driver for the Bunker Mini 2.0.
#
#  Replaces the C++ `bunker_base` ROS node on the no-ROS Pi path (plan.md §9.4).
#  Sends the same frames the SDK's SetMotionCommand / EnableCommandedMode emit,
#  decoded straight from ugv_sdk/src/protocol_v2 (agilex_msg_parser_v2.c):
#
#  TX  CTRL_MODE_CONFIG  id=0x421  dlc=8  byte0 = control_mode (0x01 = CONTROL_MODE_CAN)
#                                          (sent on enable so the base accepts CAN cmds)
#  TX  MOTION_COMMAND    id=0x111  dlc=8  int16 BE *1000 each:
#        [0:2] linear_velocity  (m/s)   [2:4] angular_velocity (rad/s)
#        [4:6] lateral_velocity (0)     [6:8] steering_angle   (0)
#      (the Bunker is differential/tracked: lateral & steering stay 0; the base
#       firmware does the track mixing from linear+angular, so no v0 track-width
#       mixing is needed here.)
#  RX  SYSTEM_STATE      id=0x211  byte0 = control_mode, [2:4] battery*0.1 (BE),
#                                          [4:6] error_code (BE)
#  RX  MOTION_STATE      id=0x221  [0:2] actual linear*1000, [2:4] actual angular*1000
#
#  SAFETY: nothing moves unless send_motion() is called. stop() zeroes motion and
#  set_standby() hands control back to the base (CONTROL_MODE_STANDBY); the caller
#  (run.py) calls both on shutdown / e-stop.
# ============================================================================
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

try:
    import can  # python-can
except ImportError:  # pragma: no cover - import-guarded so tests can stub it
    can = None

# --- protocol-v2 constants (see ugv_sdk/src/protocol_v2/agilex_protocol_v2.h) ---
MOTION_COMMAND_ID = 0x111
CTRL_MODE_CONFIG_ID = 0x421
SYSTEM_STATE_ID = 0x211
MOTION_STATE_ID = 0x221

CONTROL_MODE_STANDBY = 0x00
CONTROL_MODE_CAN = 0x01

# Bunker Mini hardware ceilings (ugv_sdk/.../bunker_params.hpp). Hard safety clamp.
HW_MAX_LINEAR = 1.5      # m/s
HW_MAX_ANGULAR = 0.7853  # rad/s


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def encode_motion(linear: float, angular: float) -> bytes:
    """Build the 8-byte 0x111 payload. int16 big-endian, value * 1000.

    Pure function (no bus) so it is unit-testable without hardware.
    """
    lin = int(round(_clamp(linear, -HW_MAX_LINEAR, HW_MAX_LINEAR) * 1000))
    ang = int(round(_clamp(angular, -HW_MAX_ANGULAR, HW_MAX_ANGULAR) * 1000))
    # >h = signed 16-bit big-endian. lateral + steering are 0 for the tracked base.
    return struct.pack(">hhhh", lin, ang, 0, 0)


def decode_motion(data: bytes) -> tuple[float, float]:
    """Inverse of encode_motion: 0x111 payload -> (linear, angular) in SI units."""
    lin, ang, _lat, _steer = struct.unpack(">hhhh", bytes(data[:8]))
    return lin / 1000.0, ang / 1000.0


def encode_ctrl_mode(mode: int) -> bytes:
    """Build the 8-byte 0x421 payload: byte0 = control mode, rest reserved (0)."""
    return bytes([mode & 0xFF, 0, 0, 0, 0, 0, 0, 0])


@dataclass
class BunkerState:
    control_mode: int = -1      # 0 standby, 1 CAN, ... ; -1 = not yet heard
    battery_voltage: float = 0.0
    error_code: int = 0
    actual_linear: float = 0.0
    actual_angular: float = 0.0
    last_rx: float = 0.0        # monotonic timestamp of the last decoded frame


class BunkerCAN:
    """python-can wrapper around the Bunker protocol-v2 control + state frames."""

    def __init__(self, channel="can0", interface="socketcan",
                 hw_max_linear=HW_MAX_LINEAR, hw_max_angular=HW_MAX_ANGULAR,
                 bus=None):
        self.hw_max_linear = float(hw_max_linear)
        self.hw_max_angular = float(hw_max_angular)
        self.state = BunkerState()
        if bus is not None:
            self.bus = bus          # injected (tests / custom transport)
        else:
            if can is None:
                raise RuntimeError(
                    "python-can is not installed; `pip install python-can`")
            self.bus = can.interface.Bus(channel=channel, interface=interface)

    # -- control ------------------------------------------------------------
    def enable_can_mode(self):
        """Tell the base to accept CAN motion commands (CONTROL_MODE_CAN)."""
        self._send(CTRL_MODE_CONFIG_ID, encode_ctrl_mode(CONTROL_MODE_CAN))

    def set_standby(self):
        """Hand control back to the base (e-stop / clean shutdown)."""
        self._send(CTRL_MODE_CONFIG_ID, encode_ctrl_mode(CONTROL_MODE_STANDBY))

    def send_motion(self, linear: float, angular: float):
        """Send one 0x111 motion frame. Values are clamped to the HW ceiling."""
        lin = _clamp(linear, -self.hw_max_linear, self.hw_max_linear)
        ang = _clamp(angular, -self.hw_max_angular, self.hw_max_angular)
        self._send(MOTION_COMMAND_ID, encode_motion(lin, ang))

    def stop(self):
        """Zero-velocity motion frame (does not change control mode)."""
        self._send(MOTION_COMMAND_ID, encode_motion(0.0, 0.0))

    # -- state --------------------------------------------------------------
    def poll(self, timeout=0.0):
        """Drain pending RX frames into self.state. Returns the updated state.

        Non-blocking by default (timeout=0). Call once per loop tick.
        """
        while True:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                break
            self._decode(msg)
            timeout = 0.0   # after the first, only drain what is already queued
        return self.state

    # -- internals ----------------------------------------------------------
    def _send(self, can_id: int, data: bytes):
        msg = can.Message(arbitration_id=can_id, data=data,
                          is_extended_id=False)
        self.bus.send(msg)

    def _decode(self, msg):
        data = bytes(msg.data)
        if msg.arbitration_id == SYSTEM_STATE_ID and len(data) >= 6:
            self.state.control_mode = data[0]
            self.state.battery_voltage = struct.unpack(">H", data[2:4])[0] * 0.1
            self.state.error_code = struct.unpack(">H", data[4:6])[0]
            self.state.last_rx = time.monotonic()
        elif msg.arbitration_id == MOTION_STATE_ID and len(data) >= 4:
            lin, ang = struct.unpack(">hh", data[0:4])
            self.state.actual_linear = lin / 1000.0
            self.state.actual_angular = ang / 1000.0
            self.state.last_rx = time.monotonic()

    def shutdown(self):
        """Best-effort safe teardown: stop, standby, close the bus."""
        try:
            self.stop()
            self.set_standby()
        finally:
            try:
                self.bus.shutdown()
            except Exception:  # noqa: BLE001
                pass
