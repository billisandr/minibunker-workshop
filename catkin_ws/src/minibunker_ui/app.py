#!/usr/bin/env python3
"""
MiniBunker — Streamlit control panel (plan.md §6).

A non-coder front door at http://localhost:8501. Talks to the ROS graph over
rosbridge (ws://HOST:9090) using roslibpy, so it runs in an ordinary host
Python venv — fully decoupled from ROS Noetic's Python 3.8 inside the container.

Panels:
  1. Live view    — the annotated /minibunker/debug_image (+ HSV mask in hsv mode)
  2. Telemetry    — FSM state, /odom speed, mission + target/hazard flags
  3. Knob panel   — detector backend, CNN conf, HSV ranges, gains, speed caps
  4. Controls     — ARM / DISARM (safety gate), backend, Mission selector, WASD pad

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


@st.cache_resource
def connect(host, port):
    client = roslibpy.Ros(host=host, port=port)
    client.run()

    # The message buffer lives on the CACHED client so it persists across
    # Streamlit reruns. A module-level dict gets re-initialised to None on every
    # rerun — before the async roslibpy callbacks can repopulate it — so the
    # live view and telemetry read empty almost every frame.
    latest = {"debug_image": None, "hsv_mask": None, "state": None,
              "odom": None, "perception": None}
    lock = threading.Lock()

    def _img_cb(key):
        def cb(msg):
            with lock:
                latest[key] = msg
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
    # Persistent, advertised publisher for WASD teleop. Like the subscriptions
    # above, this Topic MUST be kept alive on the cached client: a Topic created
    # per-publish (and then GC'd) churns its rosbridge advertise so behaviour's
    # subscriber never stays connected, and the continuous teleop stream is
    # silently dropped — which looks like "WASD does nothing in Gazebo".
    teleop_pub = roslibpy.Topic(
        client, "/minibunker/teleop_cmd", "geometry_msgs/Twist")
    teleop_pub.advertise()
    client._teleop_pub = teleop_pub

    client._latest = latest
    client._lock = lock
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
        val = json.loads(res["value"])
        # A missing param comes back as JSON "null" -> None (the service call
        # itself succeeds, so this never reaches the except). Fall back to the
        # caller's default so float()/index() sites don't get a None.
        return default if val is None else val
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


def publish_teleop(client, lin, ang):
    """Publish WASD *intent* on /minibunker/teleop_cmd via the persistent
    publisher set up in connect(). behavior_node honours it only in TELEOP
    (mission/follow_item == none) and always through the ARM gate + clamps, so
    this can never bypass DISARM."""
    pub = getattr(client, "_teleop_pub", None)
    if pub is None:
        return
    pub.publish(roslibpy.Message({
        "linear": {"x": float(lin), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(ang)},
    }))


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

with client._lock:
    snap = dict(client._latest)

# Current mission drives the telemetry labels, the selector default, and whether
# the WASD pad is live. Fetched once per rerun and reused below.
mission = get_param(client, "/mission/follow_item", "none")

# ---- top row: live view + telemetry ----
col_view, col_tel = st.columns([3, 1])
with col_view:
    # Annotated feed + HSV mask side by side (each half-width) rather than two
    # full-width monitors stacked vertically.
    img = compressed_to_np(snap["debug_image"])
    mask = ros_image_to_np(snap["hsv_mask"])
    show_mask = mask is not None and \
        get_param(client, "/detector/backend", "hsv") == "hsv"
    view_l, view_r = st.columns(2)
    with view_l:
        if img is not None:
            st.image(img, caption="/minibunker/debug_image",
                     width="stretch")
        else:
            st.info("Waiting for the annotated camera feed…")
    with view_r:
        if show_mask:
            st.image(mask, caption="what the rover sees (HSV mask)", clamp=True,
                     width="stretch")
        else:
            st.caption("HSV mask shows here in hsv mode.")

with col_tel:
    state = snap["state"]["data"] if snap["state"] else "—"
    st.metric("State", state)
    spd = "—"
    if snap["odom"]:
        v = snap["odom"]["twist"]["twist"]
        spd = "%.2f m/s" % v["linear"]["x"]
    st.metric("Speed", spd)
    st.metric("Mission", "teleop" if mission == "none" else "follow %s" % mission)
    ps = snap["perception"]["data"] if snap["perception"] else [0] * 7
    if len(ps) >= 7:
        tgt = "— (teleop)" if mission == "none" else (
            "seen ✅" if ps[0] > 0.5 else "—")
        st.metric("Target (%s)" % mission, tgt)
        st.metric("Hazard", "DANGER ⚠️" if ps[5] > 0.5 else (
            "seen" if ps[4] > 0.5 else "—"))

st.divider()

# ---- controls + knobs ----
tab_ctrl, tab_detect, tab_behave = st.tabs(["Controls", "Detector", "Behaviour"])

with tab_ctrl:
    c1, c2, c3 = st.columns(3)
    if c1.button("🟢 ARM", width="stretch"):
        publish_arm(client, True)
        st.success("ARMED — rover will move")
    if c2.button("🛑 DISARM", width="stretch"):
        publish_arm(client, False)
        st.info("DISARMED — rover frozen")
    backend = get_param(client, "/detector/backend", "hsv")
    new_backend = c3.selectbox("Detector backend", ["hsv", "cnn"],
                               index=0 if backend == "hsv" else 1)
    if new_backend != backend:
        set_param(client, "/detector/backend", new_backend)
        st.toast("backend -> %s (live)" % new_backend)

    # -- Mission: what the rover hunts (live). 'none' = manual WASD teleop. --
    options = ["none", "ball", "cone"]
    new_mission = st.selectbox(
        "Mission — follow item", options,
        index=options.index(mission) if mission in options else 0,
        help="ball/cone = autonomous follow; none = manual WASD drive below.")
    if new_mission != mission:
        set_param(client, "/mission/follow_item", new_mission)
        st.toast("mission -> %s (live)" % new_mission)

    st.caption("SAFETY: the rover boots DISARMED and publishes zero velocity "
               "until you press ARM. Keep the e-stop in hand on the real robot.")

    # -- WASD drive pad (only when mission == none) --
    st.markdown("**🎮 WASD drive**")
    if mission != "none":
        st.caption("Set Mission to **none** to enable manual driving.")
    else:
        if "teleop_intent" not in st.session_state:
            st.session_state.teleop_intent = (0.0, 0.0)
        tl = float(get_param(client, "/behavior/teleop/linear_speed", 0.25))
        ta = float(get_param(client, "/behavior/teleop/angular_speed", 0.5))
        pad = st.columns(3)
        if pad[1].button("⬆️ W", width="stretch"):
            st.session_state.teleop_intent = (tl, 0.0)
        mid = st.columns(3)
        if mid[0].button("⬅️ A", width="stretch"):
            st.session_state.teleop_intent = (0.0, ta)
        if mid[1].button("⏹️ STOP (X)", width="stretch"):
            st.session_state.teleop_intent = (0.0, 0.0)
        if mid[2].button("➡️ D", width="stretch"):
            st.session_state.teleop_intent = (0.0, -ta)
        bot = st.columns(3)
        if bot[1].button("⬇️ S", width="stretch"):
            st.session_state.teleop_intent = (-tl, 0.0)

        # Keyboard control: a tiny JS listener on the parent page maps the
        # physical W/A/S/D keys (and X = stop) to clicking the pad buttons above,
        # so you can drive from the keyboard. It binds once per session and
        # ignores keys while a text field is focused (so typing host/port etc.
        # never drives the rover). Tap to set intent; X or DISARM to stop.
        # st.iframe renders this HTML in a (same-origin) iframe, like the old
        # components.html, so window.parent reaches the app's buttons.
        st.iframe(
            """
            <script>
            (function () {
              const pwin = window.parent, pdoc = pwin.document;
              if (pwin.__wasdBound) return;            // bind only once
              pwin.__wasdBound = true;
              const MAP = {KeyW:'W', KeyS:'S', KeyA:'A', KeyD:'D', KeyX:'STOP'};
              function matches(label, want) {
                const t = (label || '').trim();
                if (want === 'STOP') return t.indexOf('STOP') !== -1;
                return t.endsWith(' ' + want) || t === want;
              }
              pdoc.addEventListener('keydown', function (e) {
                if (e.repeat) return;
                const want = MAP[e.code];
                if (!want) return;
                const ae = pdoc.activeElement, tag = ae ? ae.tagName : '';
                if (tag === 'INPUT' || tag === 'TEXTAREA' ||
                    (ae && ae.isContentEditable)) return;   // don't hijack typing
                const btns = Array.prototype.slice.call(
                    pdoc.querySelectorAll('button'));
                const hit = btns.find(function (b) {
                    return matches(b.innerText, want); });
                if (hit) { hit.click(); e.preventDefault(); }
              });
            })();
            </script>
            """,
            height=0,
        )
        st.caption("Drive with the **W A S D** keys (or click); **X** stops. "
                   "Keys set an intent that's re-published each refresh and "
                   "persists until X/DISARM; if the link drops, the watchdog "
                   "stops the rover. Needs ARM. (Click the page once so it has "
                   "keyboard focus.)")

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

# ---- teleop intent republish (keeps behaviour's watchdog fed while driving) ----
# Re-published every rerun so the rover keeps moving until STOP/DISARM; when the
# tab/browser closes the reruns stop, teleop_cmd stops, and the watchdog halts it.
if mission == "none":
    lin, ang = st.session_state.get("teleop_intent", (0.0, 0.0))
    publish_teleop(client, lin, ang)

# ---- live auto-refresh loop ----
if live:
    import time
    time.sleep(1.0 / float(refresh_hz))
    st.rerun()
