# HARDWARE_SETUP — Raspberry Pi 5 + Camera + CAN → Bunker Mini 2.0

Everything needed to run the **real** station. The sim needs none of this.

> The open hardware decisions from plan.md §15 (Q1, Q3, Q4) are resolved here as
> **defaults**; swap in your actual parts where noted.

---

## 1. Pi 5 base image

- **OS:** Raspberry Pi OS 64-bit (Bookworm) or Ubuntu 24.04 arm64.
- ROS Noetic is **not** native on Bookworm — we run it in the **arm64 Docker
  image** (Q1 default = ROS-in-Docker for sim↔real parity; a no-ROS native
  Python fallback is sketched in plan.md §9.4 if the container proves too heavy).

Build the image for arm64 **on the Pi** (or via buildx):

```bash
docker build -f docker/Dockerfile \
  --build-arg ROS_BASE_IMAGE=arm64v8/ros:noetic-ros-base \
  -t minibunker-spacemine .
```

Optional CNN runtimes (default image is lean = onnxruntime only):

```bash
# fastest CPU on ARM — adds ncnn:
docker build -f docker/Dockerfile \
  --build-arg ROS_BASE_IMAGE=arm64v8/ros:noetic-ros-base \
  --build-arg INSTALL_NCNN=true -t minibunker-spacemine .
```

**Q3 (inference HW):** default is **CPU** (onnxruntime/ncnn). If a **Hailo-8L AI
HAT** is fitted, export to `.hef` and run the Hailo runtime — out of scope for
the default image; add the HailoRT deps and a `runtime: hailo` branch.

---

## 2. CAN bus → Bunker Mini

The Bunker speaks **CAN** (default `can0`, 500 kbps). **Q4 (adapter):** either a
**USB-CAN adapter** (gs_usb/slcan) or a **CAN HAT** (e.g. Waveshare 2-CH CAN HAT,
MCP2515/2518) works — pick one and bring `can0` up on the **host**:

```bash
# USB-CAN (gs_usb, appears as a native CAN netdev):
sudo ip link set can0 up type can bitrate 500000

# MCP2515 CAN HAT: enable the overlay in /boot/firmware/config.txt first, e.g.
#   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
# then the same `ip link set can0 up ...`.

candump can0     # sanity check — you should see Bunker frames
```

`start_real.sh` brings `can0` up for you if it's down, then passes it into the
container with `--network host --cap-add NET_ADMIN` (CAN is a netdev, so the
container shares the host interface). `bunker_base` runs against `can0` exactly
as AgileX documents.

---

## 3. Camera

**Raspberry Pi Camera Module 3** (or v2) via libcamera/picamera2, **or** any USB
webcam (V4L2). `pi_camera_node` tries `picamera2` first and falls back to a V4L2
`cv2.VideoCapture` on `camera/v4l2_device` (default `/dev/video0`).

Getting the camera into the container (plan.md §9.3) — pick whichever is reliable
on your Pi:

- **(a) V4L2 passthrough:** `start_real.sh` already mounts `/dev/video0`,
  `/dev/video1` and `/run/udev` when present. Set `camera/source: webcam` (or
  leave `picamera` and let it fall back to V4L2).
- **(b) Host bridge:** run `pi_camera_node` (or a tiny picamera2 publisher) on the
  **host** Python and publish into the container's ROS master via
  `ROS_MASTER_URI`/`ROS_IP`. Use this if libcamera-in-container is painful.

Hardware-free bring-up: set `camera/source: video:/media/demo.mp4` (a bundled
clip) or `synthetic` — the stack runs with no camera at all.

---

## 4. Launch + safety

```bash
bash start_real.sh
# panel:  bash catkin_ws/src/minibunker_ui/run_ui.sh
# ARM:    docker exec -it minibunker-real bash -ic mb_arm
```

SAFETY (non-negotiable):
- Rover boots **DISARMED**; it publishes zero `Twist` until you ARM it.
- Hard speed caps live in `config/minibunker.yaml -> behavior/limits`.
- Keep a **hardware e-stop** in hand. Run inside a **fenced arena**, instructor-only.

---

## 5. Windows dev note (sim only)

The sim runs on the x86 laptop and hits the same Windows/Docker gotchas as the
sibling `ros_z1_*` stations: detached-container + foreground viz to avoid `-it`
contention, `MSYS_NO_PATHCONV=1` for device/path args, CRLF normalisation on
vendored `CMakeLists.txt` (the Dockerfile does this), full rebuild after edits.
There is no real Bunker on Windows — the real path is Pi-only.
