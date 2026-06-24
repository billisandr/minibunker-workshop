#!/usr/bin/env python3
"""
download_dataset.py — fetch the 2-class (green_ball, cone) dataset from Roboflow
Universe in YOLOv8 format (plan.md §10 step 1).

The API key is read from the ROBOFLOW_API_KEY env var (never commit it — see
.gitignore). Point --workspace/--project/--version at the Roboflow dataset you
chose; OR skip this script entirely and annotate real arena photos in Roboflow
(the better domain match — see docs/TRAINING.md, §15 Q6 of plan.md).

Usage:
    export ROBOFLOW_API_KEY=xxxx
    python download_dataset.py --workspace WS --project PROJ --version 1

Output: training/datasets/<project>/  with a data.yaml YOLOv8 expects.

IMPORTANT: the class order in the dataset's data.yaml MUST be
[green_ball, cone] to match detector_node's class indices (0=green_ball, 1=cone)
and config/minibunker.yaml detector.cnn.class_names. If your source dataset uses
different names/order, remap it (see remap note in docs/TRAINING.md).
"""
import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", required=True)
    ap.add_argument("--project", required=True)
    ap.add_argument("--version", type=int, default=1)
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__), "datasets"))
    args = ap.parse_args()

    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        sys.exit("Set ROBOFLOW_API_KEY in your environment (do NOT hard-code it).")

    from roboflow import Roboflow
    os.makedirs(args.outdir, exist_ok=True)
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(args.workspace).project(args.project)
    dataset = project.version(args.version).download("yolov8", location=args.outdir)
    print("Downloaded to:", dataset.location)
    print("Check that data.yaml lists names in order [green_ball, cone].")


if __name__ == "__main__":
    main()
