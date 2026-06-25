# ARENA — physical setup & object spec

The real arena the sim mirrors. Keep the two visually close so behaviour tuned in
sim transfers (the sim is deliberately a stand-in — see the note on cones).

---

## Footprint

- A fenced area ~**6 m × 6 m** (the visible fence is off by default in sim —
  set `arena/objects/fence/enabled: true` in `config/minibunker.yaml`).
- Flat floor, even-ish lighting (avoid hard shadows and direct sun on the ball —
  they wreck the HSV ranges and confuse the CNN).
- One obvious **start pose** for the rover (sim spawns it at the centre, x-forward).

## Objects

| Object | Role | Spec | Notes |
| --- | --- | --- | --- |
| **Green "ore" ball** | the target to approach | bright green, ~**0.20–0.30 m** dia | matte > glossy (glossy → specular highlights that read as white, not green) |
| **Construction cones** | hazards to avoid | standard orange traffic cones, 2–4 of them | the sim uses the real `model://construction_cone` mesh (vendored offline into the image); per-cone `scale` in the `arena:` block of `config/minibunker.yaml` |

The HSV defaults in `config/minibunker.yaml` assume a saturated green ball and
orange cones; tune them live in the Streamlit **HSV ranges** tab for your actual
objects + lighting.

### Sim arena objects (single source of truth)

In sim, **every** object — the `floor`, the `sun` light, the ball, cones and
fence — is defined in **one place**: the `arena:` block at the bottom of
`config/minibunker.yaml`. The `.world` file holds only physics and a baseline
ambient; `arena_setup.py` reads the block and spawns each object (floor + lights
first). Entries set `type` (`sphere`/`box`/`cylinder`/`cone_mesh`/`plane`/`fence`
/`light`), size, `pose`, `static` flag, `color`, and for dynamic props `mass`,
friction (`mu`) and contact (`kp`/`kd`); **inertia is auto-computed** from
type + size + mass (override with `inertia: [ixx,iyy,izz]` only if needed). Add,
remove, move or re-tune anything by editing that block, then relaunch (with the
dev mount, just `bash start_sim.sh` — no rebuild).

## Distance calibration (one-time, optional)

The rover has no depth sensor — it stops when the ball's bounding-box **height
fraction** reaches `behavior/approach/collect_bbox_frac` (default 0.45 ≈ "ball
fills ~half the frame height"). To set a real stop distance:

1. Place the ball at the distance you want the rover to stop at.
2. Read the live ball "size %" in the Streamlit telemetry.
3. Set `collect_bbox_frac` to that value (via the panel or the YAML).

## Safety layout

- The arena must be **fenced** and the rover **instructor-only** on real runs.
- Keep the **e-stop** reachable. Speed caps in `behavior/limits` are the software
  backstop, not a substitute for the e-stop.
- Boots **DISARMED**; ARM only when the area is clear.
