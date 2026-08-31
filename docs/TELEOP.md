# TELEOP — manual WASD driving

When `mission/follow_item: none` (the default, see [MISSIONS.md](MISSIONS.md)),
the rover doesn't drive itself. You drive it with WASD.

| Key | Action |
| --- | --- |
| W | forward |
| S | back |
| A | turn left |
| D | turn right |
| X (UI) / space or K (terminal) | stop |
| Q / Ctrl-C | quit (terminal node only) |

## Safety model (read this)

`behavior_node` is the single owner of `/cmd_vel`. Teleop only publishes
intent on `/minibunker/teleop_cmd`; behaviour republishes it to `/cmd_vel`
through the same ARM gate and `behavior/limits` clamp as autonomous mode. So:

- Teleop moves the rover only when armed. DISARM zeroes it instantly.
- Teleop speeds are clamped to `behavior/limits/max_linear|max_angular`.
- A watchdog (`behavior/teleop/timeout_ms`, default 400 ms) zeroes the rover if
  no fresh `teleop_cmd` arrives, so a dropped link or a closed browser stops it
  rather than latching the last motion.
- On the real robot this is a backstop, not a substitute for the hardware
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

Controls tab, then the WASD drive pad (only shown when Mission is `none`). It
publishes `teleop_cmd` straight over rosbridge, no extra node needed.

Drive with the physical W A S D keys (or click the buttons); X stops. A small
JS listener in the UI maps those keys to the pad buttons, and it ignores keys
while a text field is focused, so typing the host or port never drives the
rover. Click anywhere on the page once so it has keyboard focus.

It's set-and-persist, not key-hold: a key press or click sets the intent, say
forward, and the UI re-publishes that intent every refresh so the rover keeps
moving until you press X or DISARM. If the tab, browser, or link closes,
publishing stops and the watchdog halts the rover. Raise the sidebar refresh
rate for snappier control. Arm first.

### 2. Terminal teleop in the sim (`teleop.sh`)

A `teleop_twist_keyboard`-style node for real key presses, run inside the
running sim container so it shares the sim's ROS graph. It needs an
interactive TTY, so it's run by hand rather than from roslaunch. The
`teleop.sh` helper does the `docker exec` for you, and uses `winpty` on Git
Bash so the TTY works:

```bash
# in a SECOND terminal, after `bash start_sim.sh`:
bash teleop.sh
```

On Windows via Git Bash (workspace convention):

```powershell
& "C:\Program Files\Git\bin\bash.exe" ./teleop.sh
```

Then set Mission to none and press ARM in the UI, focus the teleop terminal,
and drive: W/A/S/D, X/space/K to stop, Q to quit. It publishes the current
intent continuously at `publish_hz` (keeping the watchdog fed while moving)
and sends a final zero on quit.

Under the hood, `teleop.sh` just runs this against the `minibunker-sim`
container (override with `MB_CONTAINER`):

```bash
docker exec -it minibunker-sim bash -ic "rosrun minibunker_behavior teleop_node.py"
```

## Quick test (sim)

1. Launch the sim station and open the Streamlit UI.
2. Leave Mission at `none`. Press ARM.
3. On the pad: W drives the rover forward; STOP halts it; DISARM freezes it
   even if an intent is still set.
4. Close the browser tab while moving, and the rover stops within
   `timeout_ms`.
