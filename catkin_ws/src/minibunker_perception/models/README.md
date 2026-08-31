# models/

Trained CNN weights live here. They are **gitignored** (binaries are released or
fetched, never committed).

The detector loads whatever `config/minibunker.yaml -> detector.cnn.weights`
points at. Inside the container that path is:

```
/home/rosuser/catkin_ws/src/minibunker_perception/models/minibunker_yolov8n.onnx
```

## How weights get here

1. Train off-robot:  `python training/train.py --data <data.yaml>`
2. Export to this dir: `python training/export.py --weights <best.pt> --format onnx`
   (or `--format ncnn` for the fastest Pi-CPU runtime — then set
   `detector.cnn.runtime: ncnn` and point `weights` at the ncnn folder).

Until a model is present, run the station with `detector.backend: hsv` — the HSV
baseline needs no weights and is the Phase-2 default.
