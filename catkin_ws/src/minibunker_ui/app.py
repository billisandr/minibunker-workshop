#!/usr/bin/env python3
"""
MiniBunker — Streamlit control panel (plan.md §6).

A non-coder front door at http://localhost:8501. Talks to the ROS graph over
rosbridge (ws://HOST:9090) using roslibpy, so it runs in an ordinary host
Python venv — fully decoupled from ROS Noetic's Python 3.8 inside the container.

Panels:
  1. Live view    — the annotated /minibunker/debug_image (+ HSV mask in hsv mode)
  2. Telemetry    — FSM state, /odom speed, detection flags
  3. Knob panel   — detector backend, CNN conf, HSV ranges, gains, speed caps
  4. Controls     — ARM / DISARM (safety gate), backend + platform switch

Knobs are set live via the rosapi set_param service; the nodes re-read the soft
knobs each loop, so sliders take effect immediately. Backend/platform/camera
changes that are read at node init are flagged as needing a relaunch.

Run:  bash catkin_ws/src/minibunker_ui/run_ui.sh   (or: streamlit run app.py)
"""
import base64
import json
import threading

import numpy as np
import streamlit as st

import roslibpy

# --------------------------------------------------------------------------- ROS bridge
DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9090

# latest messages, updated by roslibpy subscriber threads
_latest = {"debug_image": None, "hsv_mask": None, "state": None,
           "odom": None, "perception": None}
_lock = threading.Lock()


@st.cache_resource
def connect(host, port):
    client = roslibpy.Ros(host=host, port=port)
    client.run()

    def _img_cb(key):
        def cb(msg):
            with _lock:
                _latest[key] = msg
        return cb

    def _sub(name, typ, key, throttle=0):
        t = roslibpy.Topic(client, name, typ, throttle_rate=throttle,
                           queue_length=1, queue_size=1)
        t.subscribe(_img_cb(key))
        return t

    # keep references alive on the client object. The main view uses the
    # COMPRESSED debug image (jpeg, ~40KB/frame) instead of the raw Image
    # (~1.2MB/frame base64) — a big bandwidth win over rosbridge, esp. on the Pi.
    client._subs = [
        _sub("/minibunker/debug_image/compressed", "sensor_msgs/CompressedImage",
             "debug_image", 100),
        _sub("/minibunker/hsv_mask", "sensor_msgs/Image", "hsv_mask", 200),
        _sub("/minibunker/state", "std_msgs/String", "state"),
        _sub("/odom", "nav_msgs/Odometry", "odom", 200),
        _sub("/minibunker/perception_state", "std_msgs/Float32MultiArray",
             "perception", 200),
    ]
    return client


def ros_image_to_np(msg):
    """rosbridge sensor_msgs/Image -> HxWx3 RGB uint8 (handles base64 or list)."""
    if msg is None:
        return None
    data = msg["data"]
    raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)
    h, w = msg["height"], msg["width"]
    enc = msg.get("encoding", "bgr8")
    if enc == "mono8":
        return np.frombuffer(raw, np.uint8).reshape(h, w)
    arr = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
    return arr[:, :, ::-1] if enc == "bgr8" else arr  # BGR->RGB


def compressed_to_np(msg):
    """rosbridge sensor_msgs/CompressedImage (jpeg) -> HxWx3 RGB uint8."""
    if msg is None:
        return None
    data = msg["data"]
    raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)
    try:
        import cv2
        bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        return None if bgr is None else bgr[:, :, ::-1]
    except Exception:  # noqa: BLE001 — pillow fallback if cv2 isn't present
        from io import BytesIO

        from PIL import Image as PILImage
        return np.array(PILImage.open(BytesIO(raw)).convert("RGB"))


def get_param(client, name, default=None):
    try:
        svc = roslibpy.Service(client, "/rosapi/get_param", "rosapi/GetParam")
        res = svc.call(roslibpy.ServiceRequest({"name": name}), timeout=3)
        return json.loads(res["value"])
    except Exception:
        return default


def set_param(client, name, value):
    try:
        svc = roslibpy.Service(client, "/rosapi/set_param", "rosapi/SetParam")
        svc.call(roslibpy.ServiceRequest(
            {"name": name, "value": json.dumps(value)}), timeout=3)
    except Exception as exc:  # noqa: BLE001
        st.warning("set_param %s failed: %s" % (name, exc))


def publish_arm(client, armed):
    t = roslibpy.Topic(client, "/minibunker/arm", "std_msgs/Bool")
    t.publish(roslibpy.Message({"data": bool(armed)}))


# --------------------------------------------------------------------------- UI
st.set_page_config(page_title="MiniBunker", layout="wide")
st.title("🛰️ MiniBunker — Control Panel")

with st.sidebar:
    st.subheader("Connection")
    host = st.text_input("rosbridge host", DEFAULT_HOST)
    port = st.number_input("port", 1, 65535, DEFAULT_PORT)
    live = st.checkbox("🔴 Live (auto-refresh)", value=True)
    refresh_hz = st.slider("refresh Hz", 1, 15, 8)

try:
    client = connect(host, int(port))
    connected = client.is_connected
except Exception as exc:  # noqa: BLE001
    connected = False
    st.error("Could not reach rosbridge at ws://%s:%s — start the station first "
             "(start_sim.sh / start_real.sh). %s" % (host, port, exc))
    st.stop()

if not connected:
    st.warning("rosbridge not connected yet — reload once the station is up.")
    st.stop()

with _lock:
    snap = dict(_latest)

# ---- top row: live view + telemetry ----
col_view, col_tel = st.columns([3, 1])
with col_view:
    img = compressed_to_np(snap["debug_image"])
    if img is not None:
        st.image(img, caption="/minibunker/debug_image", use_container_width=True)
    else:
        st.info("Waiting for the annotated camera feed…")
    mask = ros_image_to_np(snap["hsv_mask"])
    if mask is not None and get_param(client, "/detector/backend", "hsv") == "hsv":
        st.image(mask, caption="what the rover sees (HSV mask)", clamp=True,
                 use_container_width=True)

with col_tel:
    state = snap["state"]["data"] if snap["state"] else "—"
    st.metric("State", state)
    spd = "—"
    if snap["odom"]:
        v = snap["odom"]["twist"]["twist"]
        spd = "%.2f m/s" % v["linear"]["x"]
    st.metric("Speed", spd)
    ps = snap["perception"]["data"] if snap["perception"] else [0] * 7
    if len(ps) >= 7:
        st.metric("Green ball", "seen ✅" if ps[0] > 0.5 else "—")
        st.metric("Cone", "DANGER ⚠️" if ps[5] > 0.5 else (
            "seen" if ps[4] > 0.5 else "—"))

st.divider()

# ---- controls + knobs ----
tab_ctrl, tab_detect, tab_behave = st.tabs(["Controls", "Detector", "Behaviour"])

with tab_ctrl:
    c1, c2, c3 = st.columns(3)
    if c1.button("🟢 ARM", use_container_width=True):
        publish_arm(client, True)
        st.success("ARMED — rover will move")
    if c2.button("🛑 DISARM", use_container_width=True):
        publish_arm(client, False)
        st.info("DISARMED — rover frozen")
    backend = get_param(client, "/detector/backend", "hsv")
    new_backend = c3.selectbox("Detector backend", ["hsv", "cnn"],
                               index=0 if backend == "hsv" else 1)
    if new_backend != backend:
        set_param(client, "/detector/backend", new_backend)
        st.toast("backend -> %s (live)" % new_backend)
    st.caption("SAFETY: the rover boots DISARMED and publishes zero velocity "
               "until you press ARM. Keep the e-stop in hand on the real robot.")

with tab_detect:
    st.caption("CNN confidence applies in cnn mode; HSV ranges apply in hsv mode.")
    a, b = st.columns(2)
    with a:
        set_param(client, "/detector/cnn/conf_threshold",
                  st.slider("CNN confidence", 0.05, 0.95,
                            float(get_param(client, "/detector/cnn/conf_threshold", 0.45)), 0.05))
    with b:
        st.markdown("**Green ball HSV**")
        gl = get_param(client, "/detector/hsv/green_ball/lower", [40, 70, 40])
        gu = get_param(client, "/detector/hsv/green_ball/upper", [85, 255, 255])
        gh = st.slider("Green Hue", 0, 179, (int(gl[0]), int(gu[0])))
        set_param(client, "/detector/hsv/green_ball/lower", [gh[0], int(gl[1]), int(gl[2])])
        set_param(client, "/detector/hsv/green_ball/upper", [gh[1], int(gu[1]), int(gu[2])])
        st.markdown("**Cone HSV (orange)**")
        cl = get_param(client, "/detector/hsv/cone/lower", [5, 120, 90])
        cu = get_param(client, "/detector/hsv/cone/upper", [25, 255, 255])
        ch = st.slider("Cone Hue", 0, 179, (int(cl[0]), int(cu[0])))
        set_param(client, "/detector/hsv/cone/lower", [ch[0], int(cl[1]), int(cl[2])])
        set_param(client, "/detector/hsv/cone/upper", [ch[1], int(cu[1]), int(cu[2])])

with tab_behave:
    a, b = st.columns(2)
    with a:
        set_param(client, "/behavior/approach/forward_speed",
                  st.slider("Forward speed (m/s)", 0.0, 0.5,
                            float(get_param(client, "/behavior/approach/forward_speed", 0.25)), 0.01))
        set_param(client, "/behavior/approach/steer_gain",
                  st.slider("Steer gain", 0.0, 2.0,
                            float(get_param(client, "/behavior/approach/steer_gain", 0.8)), 0.05))
        set_param(client, "/behavior/approach/collect_bbox_frac",
                  st.slider("Stop distance (ball bbox frac)", 0.1, 0.9,
                            float(get_param(client, "/behavior/approach/collect_bbox_frac", 0.45)), 0.05))
    with b:
        set_param(client, "/behavior/avoid/cone_danger_frac",
                  st.slider("Cone danger (bbox frac)", 0.1, 0.9,
                            float(get_param(client, "/behavior/avoid/cone_danger_frac", 0.35)), 0.05))
        set_param(client, "/behavior/limits/max_linear",
                  st.slider("Max linear (m/s)", 0.1, 0.6,
                            float(get_param(client, "/behavior/limits/max_linear", 0.4)), 0.05))
        set_param(client, "/behavior/limits/max_angular",
                  st.slider("Max angular (rad/s)", 0.2, 2.0,
                            float(get_param(client, "/behavior/limits/max_angular", 1.0)), 0.1))

st.caption("Knobs set ROS params live over rosbridge; nodes re-read soft knobs "
           "each loop. Backend/platform/camera changes need a relaunch.")

# ---- live auto-refresh loop ----
if live:
    import time
    time.sleep(1.0 / float(refresh_hz))
    st.rerun()
