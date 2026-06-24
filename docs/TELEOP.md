# TELEOP — manual WASD driving

When `mission/follow_item: none` (the default — see [MISSIONS.md](MISSIONS.md)),
the rover does not drive itself. You drive it with **WASD**.

| Key | Action |
| --- | --- |
| **W** | forward |
| **S** | back |
| **A** | turn left |
| **D** | turn right |
| **space / K** | stop |
| **Q / Ctrl-C** | quit (terminal node only) |

## Safety model (read this)

`behavior_node` is the **single owner of `/cmd_vel`**. Teleop only publishes
**intent** on `/minibunker/teleop_cmd`; behaviour republishes it to `/cmd_vel`
through the **same ARM gate + `behavior/limits` clamp** as autonomous mode. So:

- Teleop moves the rover **only when ARMED**. DISARM zeroes it instantly.
- Teleop speeds are clamped to `behavior/limits/max_linear|max_angular`.
- A **watchdog** (`behavior/teleop/timeout_ms`, default 400 ms) zeroes the rover
  if no fresh `teleop_cmd` arrives — so a dropped link or closed browser stops it
  rather than latching the last motion.
- On the real robot this is a backstop, **not** a substitute for the hardware
  e-stop. Keep it in hand.

```yaml
behavior:
  teleop:
    timeout_ms: 400      # watchdog: stop if no teleop_cmd within this window
    publish_hz: 20.0     # terminal teleop_node republish rate (feeds the watchdog)
    linear_speed: 0.25   # m/s per fwd/back key (then clamped to limits)
    angular_speed: 0.8   # rad/s per turn key (then clamped to limits)
```

## Two ways to drive

### 1. Streamlit WASD pad (primary, no terminal)

Controls tab → **🎮 WASD drive** pad (only shown when Mission = `none`). It
publishes `teleop_cmd` straight over rosbridge — no extra node needed.

It is **click-to-set, not key-hold**: a click sets the intent (e.g. forward), and
the UI re-publishes that intent every refresh so the rover keeps moving until you
press **STOP** or **DISARM**. If the tab/browser/link closes, publishing stops
and the watchdog halts the rover. Raise the sidebar **refresh Hz** for snappier
control. ARM first.

### 2. Terminal teleop node (power users)

A `teleop_twist_keyboard`-style node for real key presses. It needs an
interactive TTY, so it is **run by hand, not from roslaunch**:

```bash
# from the host, into the running station container:
docker exec -it minibunker rosrun minibunker_behavior teleop_node.py
```

On Windows run the host side via Git Bash (workspace convention):

```powershell
& "C:\Program Files\Git\bin\bash.exe" -c 'docker exec -it minibunker rosrun minibunker_behavior teleop_node.py'
```

It publishes the current intent continuously at `publish_hz` (keeping the
watchdog fed while moving) and a final zero on quit.

## Quick test (sim)

1. Launch the sim station; open the Streamlit UI.
2. Leave Mission = `none`. Press **ARM**.
3. Pad: **W** → rover drives forward; **STOP** → halts; **DISARM** → frozen even
   if an intent is set.
4. Close the browser tab while moving → the rover stops within `timeout_ms`.
