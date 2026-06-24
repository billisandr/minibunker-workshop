# TRAINING — the CNN (green ball vs. cone)

The workshop's "how a robot learns to see" story: annotate → train → export →
deploy. The HSV baseline is the "before"; the trained YOLOv8-nano is the "after".
Runs off-robot (Colab or any CUDA box). Mirrors plan.md §10.

```bash
pip install -r training/requirements.txt
```

---

## 1. Dataset

Two classes, **in this exact order**: `[green_ball, cone]` (the order is
load-bearing — it must match `detector_node` indices 0/1 and
`config/minibunker.yaml -> detector.cnn.class_names`).

**Q6 (plan.md §15): public sets vs. real arena photos.** Two paths:

- **(a) Public Roboflow Universe** — search `traffic cone` / `construction cone`
  and `green ball` / `tennis ball` / `sports ball`, merge into one 2-class set.
  Fast, but a weaker domain match.
  ```bash
  export ROBOFLOW_API_KEY=xxxx          # never commit this
  python training/download_dataset.py --workspace WS --project PROJ --version 1
  ```
- **(b) Annotate real arena photos (recommended)** — shoot ~150–300 photos of the
  actual ball + cones under the real arena lighting/background, label in Roboflow,
  export **YOLOv8**. Best accuracy where it matters. Drop the export under
  `training/datasets/minibunker/` matching `training/data.yaml`.

If a source dataset uses different class names/order, remap labels so green_ball=0,
cone=1 before training.

---

## 2. Train

```bash
python training/train.py --data training/datasets/minibunker/data.yaml --epochs 100
# quick smoke run against the template:
python training/train.py --data training/data.yaml --epochs 5
```

YOLOv8-**nano**, ~416 input, light colour augmentation (we don't want to distort
green↔orange). Aim for a model that runs **≥10 FPS on the Pi 5 CPU**.

---

## 3. Validate

Check mAP and eyeball held-out arena frames — confirm the green ball and orange
cone are **not** confused under arena lighting (the main failure mode). Tune the
dataset/aug if they are.

---

## 4. Export to the Pi

```bash
# portable default (onnxruntime):
python training/export.py --weights training/runs/minibunker_yolov8n/weights/best.pt --format onnx
# fastest CPU on ARM:
python training/export.py --weights ... --format ncnn
```

`export.py` copies the result into
`catkin_ws/src/minibunker_perception/models/` (the path the detector loads). Then
set in `config/minibunker.yaml`:

- onnx → `detector.cnn.runtime: onnxruntime`, `weights: .../minibunker_yolov8n.onnx`
- ncnn → `detector.cnn.runtime: ncnn`, `weights: .../minibunker_yolov8n_ncnn_model`

---

## 5. Don't commit weights

Binaries are gitignored (`**/models/*.onnx`, etc.). Attach the trained file to a
GitHub release or fetch it with a `models/download.sh`. Until a model exists, run
with `detector.backend: hsv`.
