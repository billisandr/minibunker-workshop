#!/usr/bin/env python3
# ============================================================================
#  test_bunker_can.py — validate the AgileX protocol-v2 frame encoding.
#
#  Two layers:
#   1. Pure encode/decode round-trips (no hardware, no python-can) — always run.
#   2. A real socketcan LOOPBACK over vcan0 if it exists (created on the Pi with
#      `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan &&
#       sudo ip link set up vcan0`). Confirms BunkerCAN actually puts the right
#       bytes on a bus and reads SYSTEM_STATE/MOTION_STATE back. Skipped if vcan0
#       is absent or python-can is missing — so it never blocks the unit layer.
#
#  Run:  python3 -m pytest real_pi/tests/test_bunker_can.py -v
#    or: python3 real_pi/tests/test_bunker_can.py   (no pytest needed)
# ============================================================================
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from minibunker_real.bunker_can import (  # noqa: E402
    BunkerCAN, CONTROL_MODE_CAN, CONTROL_MODE_STANDBY, CTRL_MODE_CONFIG_ID,
    HW_MAX_ANGULAR, HW_MAX_LINEAR, MOTION_COMMAND_ID, decode_motion,
    encode_ctrl_mode, encode_motion,
)


# ---- layer 1: pure encode/decode (no deps) --------------------------------
def test_motion_roundtrip():
    for lin, ang in [(0.0, 0.0), (0.25, -0.5), (-0.4, 0.7), (1.0, 0.3)]:
        d = encode_motion(lin, ang)
        assert len(d) == 8
        rl, ra = decode_motion(d)
        assert abs(rl - lin) < 1e-3 and abs(ra - ang) < 1e-3


def test_motion_is_big_endian_x1000():
    # 0.25 m/s -> 250 -> 0x00FA big-endian; angular field is bytes [2:4].
    d = encode_motion(0.25, 0.0)
    assert d[0:2] == struct.pack(">h", 250)
    assert d[2:4] == struct.pack(">h", 0)
    assert d[4:8] == b"\x00\x00\x00\x00"   # lateral + steering must be 0


def test_motion_clamps_to_hw_ceiling():
    d = encode_motion(99.0, 99.0)   # absurd request
    rl, ra = decode_motion(d)
    assert abs(rl - HW_MAX_LINEAR) < 1e-3
    assert abs(ra - HW_MAX_ANGULAR) < 1e-3
    d = encode_motion(-99.0, -99.0)
    rl, ra = decode_motion(d)
    assert abs(rl + HW_MAX_LINEAR) < 1e-3
    assert abs(ra + HW_MAX_ANGULAR) < 1e-3


def test_ctrl_mode_frame():
    assert encode_ctrl_mode(CONTROL_MODE_CAN) == bytes([0x01, 0, 0, 0, 0, 0, 0, 0])
    assert encode_ctrl_mode(CONTROL_MODE_STANDBY) == bytes(8)


# ---- layer 2: real vcan0 loopback (skipped if unavailable) ----------------
def _vcan_available():
    if not os.path.exists("/sys/class/net/vcan0"):
        return False
    try:
        import can  # noqa: F401
        return True
    except ImportError:
        return False


def test_vcan_loopback():
    if not _vcan_available():
        print("[skip] vcan0 / python-can not available")
        return
    import can
    listener = can.interface.Bus(channel="vcan0", interface="socketcan")
    bunker = BunkerCAN(channel="vcan0", interface="socketcan")
    try:
        bunker.send_motion(0.25, -0.5)
        msg = listener.recv(timeout=1.0)
        assert msg is not None and msg.arbitration_id == MOTION_COMMAND_ID
        rl, ra = decode_motion(msg.data)
        assert abs(rl - 0.25) < 1e-3 and abs(ra + 0.5) < 1e-3

        bunker.enable_can_mode()
        msg = listener.recv(timeout=1.0)
        assert msg is not None and msg.arbitration_id == CTRL_MODE_CONFIG_ID
        assert msg.data[0] == CONTROL_MODE_CAN

        # inject a SYSTEM_STATE frame and confirm BunkerCAN decodes it
        from minibunker_real.bunker_can import SYSTEM_STATE_ID
        batt = struct.pack(">H", 254)        # 25.4 V
        payload = bytes([CONTROL_MODE_CAN, 0]) + batt + b"\x00\x00\x00\x00"
        listener.send(can.Message(arbitration_id=SYSTEM_STATE_ID,
                                  data=payload[:8], is_extended_id=False))
        st = None
        for _ in range(20):
            st = bunker.poll(timeout=0.2)
            if st.control_mode == CONTROL_MODE_CAN:
                break
        assert st.control_mode == CONTROL_MODE_CAN
        assert abs(st.battery_voltage - 25.4) < 0.05
        print("[ok] vcan0 loopback: motion, ctrl-mode, state decode")
    finally:
        bunker.bus.shutdown()
        listener.shutdown()


if __name__ == "__main__":
    # run without pytest
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
