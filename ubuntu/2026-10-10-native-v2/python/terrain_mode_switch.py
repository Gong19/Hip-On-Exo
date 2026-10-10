#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Terrain → control-mode switching framework.

Purpose
  Bridge VisionWorker's frame-level terrain classification (see
  vision_terrain.py / hipexo_monitor.py) to the exoskeleton's control
  parameters (MOTOR_PARAMS), so the assistance strategy can automatically
  change when the operator walks from flat ground onto stairs, and back.

Why this is a SEPARATE, coarser gate on top of the vision system's own
10-frame majority vote (see WORKING_LOG.md 2026-08-18)
  The CNN's "Stable" label already smooths single-frame noise, but it can
  still flip within ~1 second (10 frames @ 8-10 Hz) if the operator is
  standing at a stair edge, glancing sideways, etc. Flipping real motor
  control gains that fast is a physical safety hazard, not just a UI
  annoyance — a wrong/rapid assistance-mode switch on stairs could
  destabilize the user. The literature on continuous locomotion-mode
  recognition for exoskeletons (see the papers under
  hip_on_vision/advanced/, e.g. "Real-Time Continuous Locomotion Mode
  Recognition and Transition Prediction...", the gaze+vision fusion paper,
  and the probability-fusion paper) consistently uses a second, slower
  confirmation stage — confidence gating plus a minimum dwell/consensus
  window — before committing to a mode transition, precisely for this
  reason. TerrainModeSwitcher implements that second stage:

    1. Confidence gate  — a prediction below MIN_CONFIDENCE never starts a
       transition, regardless of the label.
    2. Dwell gate       — a *different* label than the currently committed
       mode must be seen continuously for MIN_DWELL_S before it is
       committed. A single flicker back to the old label resets the timer.
    3. Fail-static       — if the camera goes offline, or auto-switching is
       disabled, the switcher does nothing and holds the last committed
       mode. It never guesses a mode when it doesn't have good data.
    4. Manual override always wins — `enabled=False` (the default) makes
       every update() call a no-op; `force_mode()` lets an operator pin a
       mode directly regardless of what the camera sees.

Safety note on TERRAIN_CONTROL_PRESETS
  The preset gains below are intentionally NOT tuned assistance profiles —
  they mirror the existing MOTOR_PARAMS safe/passive defaults (DQ mode,
  zero torque) for every terrain. Wiring this module up in
  hipexo_monitor.py therefore changes NOTHING about real motor behavior
  until a controls engineer fills in validated per-terrain gains here.
  Do not enable "auto mode" for real assistance before that happens.
  See WORKING_LOG.md 2026-08-18 "环境识别驱动的控制模式切换框架" entry.

This module has NO PyQt / hardware dependency — it is a pure state machine
so it can be (and is, see test_terrain_mode_switch.py) unit tested without
a camera or motors attached.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field


# Geometric terrain classes. The first three (flat/stairs_up/stairs_down)
# have a trained model (vision_terrain.py + models/projection_cnn_clean.pt).
# The rest are taxonomy placeholders from the 2026-08-18 literature/scenario
# expansion (see hip_on_vision/TERRAIN_EXPANSION_RESEARCH_CN.md) — NO
# training data or model exists for them yet. They are listed here so the
# rest of the pipeline (DataManager numeric encoding, preset lookup) is
# forward-compatible the moment a retrained model starts emitting these
# labels, without needing further code changes:
#   - slope_up / slope_down: a separate slope CNN already exists in
#     hip_on_vision (models/slope_cnn.pt) but isn't wired into
#     hipexo_monitor.py's VisionWorker yet (it only loads projection_cnn_clean).
#   - gravel_rocky_uneven, sand_soft, mud_wet_soft: no model at all yet —
#     literature review found vision alone is not reliable for these
#     (their defining property is mechanical/deformable, e.g. how much a
#     surface gives underfoot, which vision can't directly see — see the
#     research doc). Treat these as requiring IMU+Force fusion, not a
#     depth-CNN label alone, once real data collection starts.
LABELS = ("flat", "stairs_up", "stairs_down",
          "slope_up", "slope_down",
          "gravel_rocky_uneven", "sand_soft", "mud_wet_soft")

# ── Tunable gate parameters (placeholders — revisit with real data) ────────
MIN_CONFIDENCE = 0.85   # a prediction below this never starts a transition
MIN_DWELL_S    = 1.0    # a candidate label must hold for this long, uninterrupted,
                         # before it's committed as the new control mode


def _safe_passive_preset() -> dict:
    """DQ mode, zero effort — produces no assistance. This is the ONLY
    preset shape currently defined for any terrain (see safety note in the
    module docstring): filling in real per-terrain gains is a controls/
    biomechanics task for after bench validation, not something to invent
    here. Returns a fresh dict each call so presets never share mutable
    state (see test_terrain_mode_switch.py)."""
    return {
        "MODE": "DQ", "KP": 0.0, "KD": 0.0, "TAU": 0.0,
        "Q_SET": 0.0, "DQ_SET": None, "DQ_SCALE": "GEAR",
        "K": 0.0, "D": 0.0, "M": 0.0,
        "TRAJ": {"type": "CONST", "q0": 0.0, "amp": 0.0, "freq": 0.5, "phase": 0.0},
    }


# ── Per-terrain control presets ─────────────────────────────────────────────
# Every entry is currently the identical safe/passive placeholder above.
# TODO (controls/biomechanics work, not a code change): once each terrain
# has a validated assistance strategy, replace the corresponding entry's
# gains — the dwell/confidence gating in TerrainModeSwitcher stays the same
# regardless of what the presets actually contain.
TERRAIN_CONTROL_PRESETS = {label: _safe_passive_preset() for label in LABELS}


@dataclass
class SwitchEvent:
    t: float
    from_mode: str | None
    to_mode: str
    confidence: float


class TerrainModeSwitcher:
    """
    Feed it every VisionWorker.sig_update tick via update(); it tells you
    (via the return value) when — if ever — a control-mode switch should
    actually be committed. Committing is the CALLER's job (this class does
    not touch MOTOR_PARAMS itself, keeping it hardware/GUI-independent and
    trivially unit-testable).
    """

    def __init__(self,
                 min_confidence: float = MIN_CONFIDENCE,
                 min_dwell_s: float = MIN_DWELL_S,
                 initial_mode: str | None = None):
        self.enabled = False          # operator must explicitly arm auto mode
        self.min_confidence = min_confidence
        self.min_dwell_s = min_dwell_s
        self.committed_mode: str | None = initial_mode
        self._candidate: str | None = None
        self._candidate_since: float = 0.0
        self.history: list[SwitchEvent] = []

    # ── manual control ───────────────────────────────────────────────────
    def enable(self):
        self.enabled = True
        self._candidate = None

    def disable(self):
        self.enabled = False
        self._candidate = None

    def force_mode(self, mode: str, t: float | None = None) -> SwitchEvent:
        """Operator-pinned mode change — bypasses confidence/dwell gating
        entirely (an explicit human decision doesn't need machine
        confirmation) but is still recorded in history for the log."""
        t = t if t is not None else time.time()
        ev = SwitchEvent(t=t, from_mode=self.committed_mode, to_mode=mode, confidence=1.0)
        self.committed_mode = mode
        self._candidate = None
        self.history.append(ev)
        return ev

    # ── vision-driven updates ────────────────────────────────────────────
    def update(self, stable_label: str, confidence: float, camera_online: bool,
               t: float | None = None) -> SwitchEvent | None:
        """
        Call this once per VisionWorker.sig_update tick. Returns a
        SwitchEvent if (and only if) a new mode was just committed this
        call, else None.
        """
        t = t if t is not None else time.time()

        if not self.enabled or not camera_online:
            self._candidate = None
            return None

        if confidence < self.min_confidence:
            # Low-confidence frame: don't let it start OR continue building
            # a candidate — require a clean run of confident frames.
            self._candidate = None
            return None

        if stable_label == self.committed_mode:
            self._candidate = None
            return None

        if stable_label != self._candidate:
            self._candidate = stable_label
            self._candidate_since = t
            return None

        if (t - self._candidate_since) < self.min_dwell_s:
            return None

        ev = SwitchEvent(t=t, from_mode=self.committed_mode,
                          to_mode=stable_label, confidence=confidence)
        self.committed_mode = stable_label
        self._candidate = None
        self.history.append(ev)
        return ev

    def apply_to_motor_params(self, motor_params: dict, motor_ids) -> None:
        """
        Push TERRAIN_CONTROL_PRESETS[self.committed_mode] into
        hipexo_monitor.MOTOR_PARAMS for the given motor ids, in place —
        mirrors how MotorSettingsDialog._on_apply() already mutates
        MOTOR_PARAMS live, which MotorWorker reads fresh every control-loop
        iteration (no restart/snapshot needed).
        """
        if self.committed_mode is None:
            return
        preset = TERRAIN_CONTROL_PRESETS.get(self.committed_mode)
        if preset is None:
            return
        for mid in motor_ids:
            motor_params[mid] = dict(preset)
