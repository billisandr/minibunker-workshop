# MISSIONS — pick what the rover hunts (`follow_item`)

One knob decides what the rover does: `mission/follow_item` in
`config/minibunker.yaml`. It is read **live**, so you can switch it from the
Streamlit **Controls** tab mid-run — no relaunch.

| `follow_item` | Behaviour |
| --- | --- |
| `none` *(default)* | No autonomous driving. You drive with **WASD** teleop (see [TELEOP.md](TELEOP.md)). |
| `ball` | Autonomous find-and-follow of the green "ore" ball: `SEARCH → APPROACH → COLLECT → RETREAT → SEARCH`, `AVOID`ing hazards. |
| `cone` | Same FSM, but the **cone** is the thing it approaches. |

On reaching the target (within the `behavior/approach/collect_bbox_frac` stop
distance — keep a minimum gap, ~0.5 m, calibrated per [ARENA.md](ARENA.md)) the
rover **collects**: it stops, holds for `behavior/collect/pause_sec` (5 s), then
turns a **random** way and drives off (`RETREAT`) so it doesn't immediately
re-collect the same item, and resumes `SEARCH`. Tune the cycle under
`behavior/collect` in `config/minibunker.yaml`.

```yaml
mission:
  follow_item: none           # none | ball | cone
  hazard_items: [cone]        # classes treated as obstacles to AVOID; may be empty
```

## Target vs hazard (the role-based contract)

The detector publishes one `/minibunker/perception_state` array. It is **role
based**: whatever `follow_item` names becomes the **target** (slots 0–3) and the
nearest of `hazard_items` becomes the **hazard** (slots 4–6). The behaviour FSM
never needs to know which physical class it is chasing — so the identical
`SEARCH/APPROACH/AVOID/COLLECT` logic follows a ball or a cone.

| idx | role | meaning |
| --- | --- | --- |
| 0 | `target_seen` | followed class present (0/1) |
| 1 | `target_cx_norm` | target centre x, −1..1 (+ = right) |
| 2 | `target_cy_norm` | target centre y, −1..1 (+ = down) |
| 3 | `target_h_frac` | target bbox height / image height (**distance proxy**) |
| 4 | `hazard_seen` | nearest hazard present (0/1) |
| 5 | `hazard_danger` | hazard is big + low-centre (in the danger zone) |
| 6 | `hazard_cx_norm` | nearest hazard centre x |

**The followed class is auto-excluded from its own hazard set.** If you
`follow_item: cone` with `hazard_items: [cone]`, the cone you are chasing is the
target and is *not* simultaneously treated as a hazard. With only `{ball, cone}`
in play this means following the cone currently leaves **no** active hazard; set
`hazard_items: []` explicitly if you want that to be unambiguous, or add other
classes once the perception backends in plan.md §19 land.

## How to switch

- **UI (primary):** Controls tab → **Mission — follow item** selectbox. The
  telemetry column shows the live mission and target/hazard flags.
- **YAML (boot default):** edit `mission/follow_item` and relaunch, or just set
  it live and leave the YAML as the safe `none` default.

## Safety

`follow_item` does **not** move the rover on its own — the rover still boots
**DISARMED** and only moves once you press **ARM**. Switching to `ball`/`cone`
while ARMED starts autonomous motion immediately; switching to `none` hands
control to teleop. DISARM always wins. Speed caps in `behavior/limits` clamp
every mode. See [TELEOP.md](TELEOP.md) and `docs/HARDWARE_SETUP.md`.
