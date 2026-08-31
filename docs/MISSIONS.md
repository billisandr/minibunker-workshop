# MISSIONS — pick what the rover hunts (`follow_item`)

One knob decides what the rover does: `mission/follow_item` in
`config/minibunker.yaml`. It's read live, so you can switch it from the
Streamlit Controls tab mid-run, no relaunch needed.

| `follow_item` | Behaviour |
| --- | --- |
| `none` (default) | No autonomous driving. You drive with WASD teleop (see [TELEOP.md](TELEOP.md)). |
| `ball` | Autonomous find-and-follow of the green ball: SEARCH, then APPROACH, then COLLECT, then RETREAT, back to SEARCH, avoiding hazards along the way. |
| `cone` | Same FSM, but the cone is the thing it approaches. |

On reaching the target, within the `behavior/approach/collect_bbox_frac` stop
distance (keep a minimum gap, roughly 0.5 m, calibrated per
[ARENA.md](ARENA.md)), the rover collects: it stops, holds for
`behavior/collect/pause_sec` (5 s), then turns a random way and drives off
(retreat) so it doesn't immediately re-collect the same item, and resumes
search. Tune the cycle under `behavior/collect` in `config/minibunker.yaml`.

```yaml
mission:
  follow_item: none           # none | ball | cone
  hazard_items: [cone]        # classes treated as obstacles to AVOID; may be empty
```

## Target vs hazard (the role-based contract)

The detector publishes one `/minibunker/perception_state` array, and it's role
based: whatever `follow_item` names becomes the target (slots 0 to 3), and the
nearest of `hazard_items` becomes the hazard (slots 4 to 6). The behaviour FSM
never needs to know which physical class it's chasing, so the identical
SEARCH/APPROACH/AVOID/COLLECT logic follows a ball or a cone equally well.

| idx | role | meaning |
| --- | --- | --- |
| 0 | `target_seen` | followed class present (0/1) |
| 1 | `target_cx_norm` | target centre x, −1..1 (+ = right) |
| 2 | `target_cy_norm` | target centre y, −1..1 (+ = down) |
| 3 | `target_h_frac` | target bbox height / image height (distance proxy) |
| 4 | `hazard_seen` | nearest hazard present (0/1) |
| 5 | `hazard_danger` | hazard is big and low-centre (in the danger zone) |
| 6 | `hazard_cx_norm` | nearest hazard centre x |

The followed class is auto-excluded from its own hazard set. If you set
`follow_item: cone` with `hazard_items: [cone]`, the cone you're chasing is the
target and isn't simultaneously treated as a hazard. With only `{ball, cone}`
in play, this means following the cone currently leaves no active hazard. Set
`hazard_items: []` explicitly if you want that to be unambiguous, or add other
classes once more perception backends land.

## How to switch

- UI (primary): Controls tab, then the Mission — follow item selectbox. The
  telemetry column shows the live mission and target/hazard flags.
- YAML (boot default): edit `mission/follow_item` and relaunch, or just set it
  live and leave the YAML at the safe `none` default.

## Safety

`follow_item` doesn't move the rover on its own. The rover still boots
disarmed and only moves once you press ARM. Switching to `ball` or `cone`
while armed starts autonomous motion immediately; switching to `none` hands
control back to teleop. DISARM always wins. Speed caps in `behavior/limits`
clamp every mode. See [TELEOP.md](TELEOP.md) and `docs/HARDWARE_SETUP.md`.
