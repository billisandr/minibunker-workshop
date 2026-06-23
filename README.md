# MiniBunker SpaceMine Workshop

A Space Summer School station: an AgileX **Bunker Mini 2.0** tracked rover that uses a
**Raspberry Pi 5 + Pi Camera** and a small **CNN** to recognise a **green ball** (the
"ore sample" to approach) and **construction cones** (hazards to avoid), then drives a
reactive *space-mining* behaviour. Runs identically in **Gazebo simulation** and on the
**real robot**, fully **YAML-configurable**, fronted by a **Streamlit** knob panel.

> 🚧 **Status: planning only.** No implementation code yet. The full design lives in
> **[plan.md](plan.md)** — read it before building anything.

## What this will be

- **One ROS graph, two backends** — the same detector + behaviour nodes run in sim and
  on the real Pi; only the camera source and the velocity sink swap (`platform: sim |
  real`).
- **Two perception backends** behind one topic — a YOLOv8-nano CNN (trained on a
  Roboflow dataset) as the headline, plus a classic HSV color detector as the baseline
  for the CNN-vs-classic-CV teaching contrast.
- **Streamlit UI** — live annotated camera, telemetry, and live-tunable knobs for
  non-coders; ARM/DISARM safety gate; sim↔real toggle.

## Built on

- [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim) — Gazebo models (Bunker)
- [`agilexrobotics/bunker_ros`](https://github.com/agilexrobotics/bunker_ros) — ROS1 driver for the real Bunker Mini
- [`agilexrobotics/ugv_sdk`](https://github.com/agilexrobotics/ugv_sdk) — C++ CAN layer

## Next steps

See the phased roadmap in [plan.md](plan.md) §13. This repo currently contains Phase 0
(plan + scaffold) only.

---

*Space Summer School · Technical University of Crete · SenseLAB*
