# TRAINING — the CNN (green ball vs. cone)

This covers the "how a robot learns to see" pipeline: annotate, train,
export, deploy. The HSV baseline is the "before"; the trained YOLOv8-nano is
the "after." It runs off-robot, on Colab or any CUDA box.

```bash
pip install -r training/requirements.txt
```

---

## 1. Dataset

Two classes, in this exact order: `[green_ball, cone]` (the order is
load-bearing, since it must match `detector_node` indices 0/1 and
`config/minibunker.yaml -> detector.cnn.class_names`).

Public sets versus real arena photos: there are two paths.

- Public Roboflow Universe. Search `traffic cone` / `construction cone` and
  `green ball` / `tennis ball` / `sports ball`, then merge into one 2-class
  set. Fast, but a weaker domain match.
  ```bash
  export ROBOFLOW_API_KEY=xxxx          # never commit this
  python training/download_dataset.py --workspace WS --project PROJ --version 1
  ```
- Annotate real arena photos (recommended). Shoot roughly 150 to 300 photos
  of the actual ball and cones under the real arena lighting and background,
  label them in Roboflow, and export in YOLOv8 format. This gives the best
  accuracy where it matters. Drop the export under
  `training/datasets/minibunker/`, matching `training/data.yaml`.

If a source dataset uses different class names or ordering, remap the
labels so green_ball is 0 and cone is 1 before training.

---

## 2. Train

```bash
python training/train.py --data training/datasets/minibunker/data.yaml --epochs 100
# quick smoke run against the template:
python training/train.py --data training/data.yaml --epochs 5
```

Use YOLOv8-nano, roughly 416 input, and light colour augmentation, since we
don't want to distort green versus orange. Aim for a model that runs at
least 10 FPS on the Pi 5 CPU.

---

## 3. Validate

Check mAP and eyeball held-out arena frames to confirm the green ball and
orange cone aren't confused under arena lighting, which is the main failure
mode. Tune the dataset or the augmentation if they are.

---

## 4. Export to the Pi

```bash
# portable default (onnxruntime):
python training/export.py --weights training/runs/minibunker_yolov8n/weights/best.pt --format onnx
# fastest CPU on ARM:
python training/export.py --weights ... --format ncnn
```

`export.py` copies the result into
`catkin_ws/src/minibunker_perception/models/`, the path the detector loads
from. Then set in `config/minibunker.yaml`:

- onnx: `detector.cnn.runtime: onnxruntime`, `weights: .../minibunker_yolov8n.onnx`
- ncnn: `detector.cnn.runtime: ncnn`, `weights: .../minibunker_yolov8n_ncnn_model`

---

## 5. Don't commit weights

Binaries are gitignored (`**/models/*.onnx`, and so on). Attach the trained
file to a GitHub release, or fetch it with a `models/download.sh`. Until a
model exists, run with `detector.backend: hsv`.
