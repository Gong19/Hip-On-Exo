#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision terrain classifier — D435i depth → x-z projection → CNN.

Adapted from the user's hip_on_vision research project
(/home/ros-console/Desktop/hip_on_vision/FINAL_USEFUL_FILES/src/
 projection_lmr.py + live_projection_cnn.py), copied 2026-08-18 so
hipexo_monitor.py has a small, self-contained dependency instead of
importing across into a separate research scratch folder. The model
weights (models/projection_cnn_clean.pt/.json, val accuracy 99.5% on
dataset_clean) were copied alongside into python/vision_models/.

If the terrain CNN is retrained in hip_on_vision, re-copy the updated
.pt/.json here and re-check this file against the upstream algorithm
(depth_to_projection in particular) for drift. See WORKING_LOG.md
2026-08-18 "Vision 地形识别接入" entry.

A second trained model — models/slope_cnn.pt/.json (labels: flat/
slope_up/slope_down, 91.5% validation accuracy on dataset_slope, copied
from hip_on_vision 2026-08-18 alongside the stairs model) — uses the
exact same ProjectionCNN architecture and the exact same
depth_to_projection() call (verified against
hip_on_vision/FINAL_USEFUL_FILES/src/{live_slope_cnn.py,
slope_dataset_recorder.py}: no forward_range_m/height_range_m override,
so it relies on the same defaults baked into depth_to_projection below).
That means ONE projection image can be fed to BOTH models — see
fuse_stairs_and_slope() for how VisionWorker combines their outputs into
a single label.

This module intentionally has NO dependency on PyQt — it is pure
camera-in / label-out logic, so it can be unit-tested and reused
independent of the GUI.
"""
from __future__ import annotations

import json
import os

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(_SCRIPT_DIR, "vision_models")
DEFAULT_MODEL_NAME = "projection_cnn_clean.pt"
SLOPE_MODEL_NAME = "slope_cnn.pt"

WIDTH, HEIGHT, FPS = 640, 480, 30
PROJECTION_SIZE = 100
LABELS_FALLBACK = ("flat", "stairs_up", "stairs_down")

# ── Optional heavy deps — degrade gracefully, same pattern as the other
#    optional hardware libs in hipexo_monitor.py (smbus2/spidev/SDK). ──
try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _TORCH_OK = True
    # PyTorch defaults to intra-op parallelism sized to the CPU core count
    # (e.g. 24 threads on a 48-core workstation). For a 3-layer CNN on a
    # 100x100 input run a few times a second, that parallelism buys nothing
    # but costs real CPU% in thread wake/sync overhead — measured ~125% CPU
    # (more than one full core) for a model that needs ~5ms/frame of actual
    # compute. One thread is enough; see WORKING_LOG.md 2026-08-18.
    torch.set_num_threads(1)
except ImportError:
    _TORCH_OK = False

try:
    import pyrealsense2 as rs
    _REALSENSE_OK = True
except ImportError:
    _REALSENSE_OK = False


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       DEPTH → x-z PROJECTION                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def default_intrinsics() -> dict:
    return {"fx": 615.0, "fy": 615.0, "ppx": WIDTH / 2, "ppy": HEIGHT / 2}


def intrinsics_from_realsense(depth_frame) -> dict:
    intr = depth_frame.profile.as_video_stream_profile().intrinsics
    return {"fx": float(intr.fx), "fy": float(intr.fy),
            "ppx": float(intr.ppx), "ppy": float(intr.ppy)}


def depth_to_projection(
    depth_raw: np.ndarray,
    intrinsics: dict,
    depth_scale: float = 0.001,
    forward_range_m: tuple = (0.15, 2.2),
    height_range_m: tuple = (-1.25, 0.35),
) -> np.ndarray:
    """
    Project a depth frame's central ROI into a top-down-free x-z (forward
    distance vs. height) binary occupancy image the CNN was trained on.
    Verbatim port of projection_lmr.depth_to_projection — do not change the
    ROI/range constants without retraining, the model is sensitive to them.
    """
    depth_m = depth_raw.astype(np.float32) * depth_scale
    height, width = depth_m.shape

    y1, y2 = int(height * 0.18), int(height * 0.96)
    x1, x2 = int(width * 0.12), int(width * 0.88)
    roi = depth_m[y1:y2, x1:x2]

    ys, xs = np.indices(roi.shape)
    xs = xs + x1
    ys = ys + y1

    valid = (roi >= forward_range_m[0]) & (roi <= forward_range_m[1])
    if np.count_nonzero(valid) == 0:
        return np.zeros((PROJECTION_SIZE, PROJECTION_SIZE), dtype=np.uint8)

    z = roi[valid]
    y = (ys[valid] - intrinsics["ppy"]) * z / intrinsics["fy"]

    f_min, f_max = forward_range_m
    h_min, h_max = height_range_m
    col = ((z - f_min) / (f_max - f_min) * (PROJECTION_SIZE - 1)).astype(np.int32)
    row = ((h_max - y) / (h_max - h_min) * (PROJECTION_SIZE - 1)).astype(np.int32)
    inside = (row >= 0) & (row < PROJECTION_SIZE) & (col >= 0) & (col < PROJECTION_SIZE)

    projection = np.zeros((PROJECTION_SIZE, PROJECTION_SIZE), dtype=np.uint8)
    projection[row[inside], col[inside]] = 255
    if _CV2_OK:
        projection = cv2.dilate(projection, np.ones((2, 2), dtype=np.uint8), iterations=1)
    return projection


def make_depth_vis(depth_raw: np.ndarray):
    """Colorized depth image for display (BGR uint8), or None if cv2 missing."""
    if not _CV2_OK:
        return None
    depth_8u = cv2.convertScaleAbs(depth_raw, alpha=0.03)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                              CNN MODEL                                  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

if _TORCH_OK:
    class ProjectionCNN(nn.Module):
        """Architecture must match training exactly — verbatim port."""
        def __init__(self, label_count: int = 3):
            super().__init__()
            self.conv1 = nn.Conv2d(1, 16, kernel_size=3, padding=1)
            self.conv2 = nn.Conv2d(16, 32, kernel_size=3, padding=1)
            self.conv3 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
            self.drop = nn.Dropout(0.25)
            self.fc1 = nn.Linear(64 * 12 * 12, 96)
            self.fc2 = nn.Linear(96, label_count)

        def forward(self, x):
            x = F.max_pool2d(F.relu(self.conv1(x)), 2)
            x = F.max_pool2d(F.relu(self.conv2(x)), 2)
            x = F.max_pool2d(F.relu(self.conv3(x)), 2)
            x = torch.flatten(x, 1)
            x = self.drop(F.relu(self.fc1(x)))
            return self.fc2(x)
else:
    ProjectionCNN = None  # type: ignore


def load_model(model_name: str = DEFAULT_MODEL_NAME, device=None):
    """Returns (model, labels, meta_dict). Raises if torch or the weight
    file is unavailable — caller (VisionWorker) is expected to catch this
    and report it through the same offline/status-signal path as any other
    missing-hardware/dependency condition."""
    if not _TORCH_OK:
        raise RuntimeError("PyTorch not installed — pip install torch")
    device = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_path = os.path.join(MODEL_DIR, model_name)
    checkpoint = torch.load(model_path, map_location=device)
    labels = tuple(checkpoint.get("labels", LABELS_FALLBACK))
    model = ProjectionCNN(label_count=len(labels)).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    meta = {}
    json_path = os.path.join(MODEL_DIR, os.path.splitext(model_name)[0] + ".json")
    if os.path.exists(json_path):
        try:
            with open(json_path) as f:
                meta = json.load(f)
        except Exception:
            pass
    return model, labels, meta


def classify_projection(model, labels: tuple, projection: np.ndarray, device) -> tuple:
    """Returns (label, confidence, {label: prob})."""
    binary = (projection > 0).astype(np.float32)
    x = torch.from_numpy(binary).unsqueeze(0).unsqueeze(0).to(device)
    with torch.no_grad():
        logits = model(x)
        probs = torch.softmax(logits, dim=1).cpu().numpy()[0]
    pred_id = int(np.argmax(probs))
    prob_dict = {label: float(probs[i]) for i, label in enumerate(labels)}
    return labels[pred_id], float(probs[pred_id]), prob_dict


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                  STAIRS + SLOPE MODEL FUSION                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝

FUSION_CONFIDENCE_THRESHOLD = 0.6


def fuse_stairs_and_slope(stairs_label: str, stairs_conf: float, stairs_probs: dict,
                           slope_label: str, slope_conf: float, slope_probs: dict) -> tuple:
    """
    Combine the two independently-trained classifiers (stairs model:
    flat/stairs_up/stairs_down; slope model: flat/slope_up/slope_down) that
    both run on the SAME projection image into one label.

    Heuristic (documented, not empirically tuned — revisit once real mixed
    stairs+slope+flat field data exists):
      1. A confident non-flat call from the STAIRS model wins first. A
         discrete step edge is a more specific, less ambiguous geometric
         signature than "the ground is tilted", so it's less likely to be
         a false positive when both models disagree.
      2. Otherwise a confident non-flat call from the slope model wins.
      3. Otherwise (both models see nothing but flat ground, or neither
         is confident) the result is "flat".
      4. The one pathological case — BOTH models confidently claim a
         DIFFERENT non-flat label at once (geometrically a spot can't be
         both a staircase and a ramp) — picks the higher-confidence one
         but caps the reported confidence, since model disagreement on
         the same image is itself a signal the scene is ambiguous and a
         downstream consumer (e.g. TerrainModeSwitcher) should be more
         reluctant to act on it.

    Returns (label, confidence, combined_probs) where combined_probs has
    keys for every label from both models (flat's value is the mean of
    the two models' flat estimates).
    """
    combined_probs = dict(stairs_probs)
    for k, v in slope_probs.items():
        if k == "flat" and "flat" in combined_probs:
            combined_probs["flat"] = (combined_probs["flat"] + v) / 2.0
        else:
            combined_probs[k] = v

    stairs_is_confident_nonflat = stairs_label != "flat" and stairs_conf >= FUSION_CONFIDENCE_THRESHOLD
    slope_is_confident_nonflat  = slope_label != "flat" and slope_conf >= FUSION_CONFIDENCE_THRESHOLD

    if stairs_is_confident_nonflat and slope_is_confident_nonflat:
        if stairs_conf >= slope_conf:
            return stairs_label, min(stairs_conf, 0.5), combined_probs
        return slope_label, min(slope_conf, 0.5), combined_probs
    if stairs_is_confident_nonflat:
        return stairs_label, stairs_conf, combined_probs
    if slope_is_confident_nonflat:
        return slope_label, slope_conf, combined_probs
    return "flat", min(combined_probs.get("flat", 0.0), 1.0), combined_probs


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                  FIELD DATA COLLECTION (new-terrain classes)            ║
# ║  Saves samples in the exact layout hip_on_vision's own recorders/       ║
# ║  trainers already use, so field-collected data drops straight into     ║
# ║  train_projection_cnn.py / train_slope_cnn.py with no conversion step.  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def next_dataset_index(label_dir: str) -> int:
    """Mirrors hip_on_vision/clean_dataset_recorder.py's next_index(): scan
    for the highest existing depth_raw_NNNNNN.npy in this label's folder so
    repeated collection sessions append rather than overwrite."""
    if not os.path.isdir(label_dir):
        return 0
    best = -1
    for name in os.listdir(label_dir):
        if name.startswith("depth_raw_") and name.endswith(".npy"):
            try:
                best = max(best, int(name[len("depth_raw_"):-len(".npy")]))
            except ValueError:
                pass
    return best + 1


def save_dataset_sample(output_root: str, label: str, depth_raw: np.ndarray,
                         projection: np.ndarray, depth_scale: float,
                         intrinsics: dict, extra_meta: dict = None) -> str:
    """
    Save one field-collected sample for `label` under
    {output_root}/{label}/{depth_raw,projection,depth_vis,meta}_{stem}.npy|png|json
    — same four-file layout as hip_on_vision/clean_dataset_recorder.py's
    save_sample(), so train_projection_cnn.py's collect_samples() (which
    just globs depth_raw_*.npy per label folder and reads the matching
    meta_*.json for depth_scale/intrinsics) can consume it directly.

    Returns the path to the depth_raw .npy file written.
    """
    if not _CV2_OK:
        raise RuntimeError("opencv (cv2) not available — cannot write projection/depth_vis PNGs")
    import json as _json
    import time as _time

    label_dir = os.path.join(output_root, label)
    os.makedirs(label_dir, exist_ok=True)
    stem = f"{next_dataset_index(label_dir):06d}"

    depth_path      = os.path.join(label_dir, f"depth_raw_{stem}.npy")
    projection_path = os.path.join(label_dir, f"projection_{stem}.png")
    depth_vis_path  = os.path.join(label_dir, f"depth_vis_{stem}.png")
    meta_path       = os.path.join(label_dir, f"meta_{stem}.json")

    np.save(depth_path, depth_raw)
    cv2.imwrite(projection_path, projection)
    depth_vis = make_depth_vis(depth_raw)
    if depth_vis is not None:
        cv2.imwrite(depth_vis_path, depth_vis)

    meta = {
        "label": label,
        "timestamp": _time.time(),
        "timestamp_semantics": "file_save_time_not_camera_sampling_time",
        "saved_wall_ns": _time.time_ns(),
        "saved_mono_ns": _time.perf_counter_ns(),
        "depth_scale": depth_scale,
        "depth_resolution": {"width": int(depth_raw.shape[1]), "height": int(depth_raw.shape[0])},
        "intrinsics": intrinsics,
        "projection_parameters": {"projection_size": PROJECTION_SIZE},
    }
    if extra_meta:
        meta.update(extra_meta)
    with open(meta_path, "w") as f:
        _json.dump(meta, f, indent=2)

    return depth_path
