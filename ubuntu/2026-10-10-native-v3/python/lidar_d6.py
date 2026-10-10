#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COIN-D6 (国科光芯 / CSPC Tech) 360° 2D scanning dToF LiDAR — serial
protocol parser + scan → ground-profile geometry.

Hardware summary (from the official spec sheet, verified 2026-08-20 —
see /home/ros-console/Desktop/D6-TOF/国科光芯 激光雷达/1 D6激光雷达手册资料/
COIN-D6单线激光雷达规格书A1.pdf):
  - 360° mechanical rotating dToF (brushless motor, wireless power to the
    spinning head), NOT a volumetric sensor — pitch/elevation FOV is only
    0-2°, i.e. it genuinely only sees one thin scanning PLANE.
  - Range 0.05-12m (90% diffuse reflectivity), ±10mm accuracy <1m,
    ±30-40mm beyond 1m.
  - 0.9° angular resolution, ~4000 points/sec, rotates at 10-15Hz
    (closed-loop, nominally 10Hz) → ~400 points/revolution.
  - Ambient-light immunity spec: 60klx (direct sunlight is roughly
    32,000-130,000 lux depending on time/location/reflective surfaces —
    this covers most daytime conditions but isn't an absolute guarantee
    against the very brightest cases; still a fundamentally different
    failure mode than the D435i's IR-structured-light pattern washing out).
  - 40g, 52.7×45.2×34.4mm, IPX4, 5V DC supply, ~200-240mA typical /
    800mA peak (~1-1.2W typical, up to ~2.5-4W peak), UART LVTTL 3.3V
    @ 230400 bps via a 4-pin 1.5mm connector.

Protocol (verified against THREE independent sources, not just the PDF,
since the PDF's math notation was OCR-mangled in places):
  1. COIN-D6激光雷达数据格式标准说明V1.0.pdf (official protocol doc)
  2. COIN-D6 雷达采集数据.txt — a real captured byte stream from the
     vendor (copied into tests/fixtures/coin_d6_sample_capture.txt and
     used as golden-data regression test input, see
     tests/test_lidar_d6.py)
  3. STM32F103RCT6_COIN-D6_Demo_V1.3.0/Driver/ax_laser.c — the vendor's
     own reference parser. Cross-checking it against source (2) resolved
     the OCR ambiguity AND surfaced a genuine bug in the vendor's own
     demo: its "start packet" branch reads sample bytes at fixed offsets
     `12*3`/`11*3` (=36/33) instead of the same `10+i*3` pattern its own
     "normal packet" branch correctly uses for i=0 (=12/11). This
     implementation always uses the correct `10+i*3` formula (verified
     against the real capture) — it does NOT replicate that vendor bug.

Frame layout (10-byte header + LSN×3 data bytes):
  offset 0-1   : 0xAA 0x55 header
  offset 2     : M&T byte — bit0 = 1 means "start-of-revolution" packet
                 (contains exactly 1 point, marks the beginning of a new
                 360° scan); bit0 = 0 means a normal mid-scan packet.
  offset 3     : LSN — number of samples in this packet (1 for a start
                 packet, ~25 for a normal packet in the vendor's own
                 firmware, though this parser doesn't assume that exact
                 count).
  offset 4-5   : FSA — first sample's angle (little-endian, see
                 decode_angle()).
  offset 6-7   : LSA — last sample's angle, same encoding. Angles for
                 samples in between are linearly interpolated.
  offset 8-9   : XOR checksum, validated using the vendor ROS SDK layout:
                 header words XOR each sample's first byte and last word.
  offset 10+   : LSN samples, 3 bytes each (Si_L, Si_2nd, Si_H) — see
                 decode_sample().

This module has NO PyQt / pyserial hard dependency for the parsing logic
itself (only LidarSerialReader needs pyserial, imported lazily) — the
byte-level parser and geometry functions are pure and unit-testable
without hardware, same design principle as vision_terrain.py.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field

HEADER = b"\xAA\x55"
BAUD_RATE = 230400
MAX_REASONABLE_LSN = 40   # sanity bound; real packets carry ~1 or ~25


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                        BYTE-LEVEL DECODING                              ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def decode_angle(lo: int, hi: int) -> float:
    """FSA/LSA 2-byte little-endian angle field -> degrees.
    raw16 bit0 is a fixed check bit (always 1, per spec); angle = (raw16>>1)/64."""
    raw16 = lo | (hi << 8)
    return (raw16 >> 1) / 64.0


def decode_sample(lo: int, mid: int, hi: int) -> tuple:
    """One 3-byte Si sample -> (distance_mm, intensity, high_reflection)."""
    distance_mm = (hi << 6) + (mid >> 2)
    intensity = (mid & 0x03) * 64 + (lo >> 2)
    high_reflection = bool(lo & 0x01)
    return distance_mm, intensity, high_reflection


@dataclass
class LidarPoint:
    angle_deg: float
    distance_mm: int
    intensity: int
    high_reflection: bool


def packet_checksum(frame: bytes) -> int:
    value = 0
    for offset in (0, 2, 4, 6):
        value ^= int.from_bytes(frame[offset:offset + 2], "little")
    for offset in range(10, len(frame), 3):
        value ^= frame[offset]
        value ^= int.from_bytes(frame[offset + 1:offset + 3], "little")
    return value


def parse_packet(frame: bytes):
    """
    Parse ONE complete frame (header through its last data byte — caller
    is responsible for framing/length, see D6StreamParser).
    Returns (is_start: bool, points: list[LidarPoint]) or None if `frame`
    is too short / doesn't start with the header / has an unreasonable LSN.
    """
    if len(frame) < 10 or frame[0:2] != HEADER:
        return None
    mt = frame[2]
    lsn = frame[3]
    if lsn == 0 or lsn > MAX_REASONABLE_LSN:
        return None
    frame_len = 10 + lsn * 3
    if len(frame) != frame_len:
        return None
    is_start = bool(mt & 0x01)
    if is_start and lsn != 1:
        return None
    if not (frame[4] & 1 and frame[6] & 1):
        return None
    if packet_checksum(frame) != int.from_bytes(frame[8:10], "little"):
        return None

    angle_start = decode_angle(frame[4], frame[5])
    angle_end   = decode_angle(frame[6], frame[7])
    if angle_start >= 360 or angle_end >= 360:
        return None
    if lsn == 1:
        angles = [angle_start]
    else:
        step = ((angle_end - angle_start) % 360) / (lsn - 1)
        angles = [(angle_start + step * i) % 360 for i in range(lsn)]

    points = []
    for i in range(lsn):
        off = 10 + i * 3
        lo, mid, hi = frame[off], frame[off + 1], frame[off + 2]
        distance_mm, intensity, high_reflection = decode_sample(lo, mid, hi)
        points.append(LidarPoint(angles[i], distance_mm, intensity, high_reflection))
    return is_start, points


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                    STREAMING FRAME SYNC / ASSEMBLY                      ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class D6StreamParser:
    """
    Feed it raw bytes as they arrive from the serial port (in any chunk
    size — UART reads don't respect frame boundaries); it finds the
    0xAA55 sync, extracts complete frames, and assembles them into full
    360° revolutions (from one start-packet to the next). Call feed()
    repeatedly; it returns the list of any revolutions that completed
    during that call (usually 0 or 1).
    """
    def __init__(self):
        self._buf = bytearray()
        self._current_scan: list = []
        self._started = False
        self.valid_packets = 0
        self.bad_packets = 0
        self.reported_hz = 0.0

    def feed(self, data: bytes) -> list:
        self._buf += data
        completed = []
        while True:
            idx = self._buf.find(HEADER)
            if idx < 0:
                # Keep a possible partial header (lone 0xAA at the very end)
                if self._buf[-1:] == HEADER[0:1]:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 4:
                break
            lsn = self._buf[3]
            if lsn == 0 or lsn > MAX_REASONABLE_LSN:
                # False-positive header match — skip past it and resync.
                del self._buf[:1]
                self.bad_packets += 1
                self._started = False
                self._current_scan.clear()
                continue
            frame_len = 10 + lsn * 3
            if len(self._buf) < frame_len:
                break
            frame = bytes(self._buf[:frame_len])
            parsed = parse_packet(frame)
            if parsed is None:
                del self._buf[:1]
                self.bad_packets += 1
                self._started = False
                self._current_scan.clear()
                continue
            del self._buf[:frame_len]
            self.valid_packets += 1
            is_start, points = parsed
            if is_start:
                self.reported_hz = (frame[2] >> 1) / 10.0
                if self._started and self._current_scan:
                    completed.append(self._current_scan)
                self._current_scan = list(points)
                self._started = True
            elif self._started:
                self._current_scan.extend(points)
                if len(self._current_scan) > 2000:
                    self._current_scan.clear()
                    self._started = False
        return completed


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║              SCAN -> GROUND PROFILE (mount geometry)                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _wrap_angle(a: float) -> float:
    """Normalize an angle in degrees to (-180, 180]."""
    a = a % 360.0
    if a > 180.0:
        a -= 360.0
    return a


def scan_to_ground_profile(points: list, mount_tilt_deg: float, mount_height_m: float,
                            forward_sector_deg: tuple = (-70.0, 70.0),
                            max_range_m: float = 3.0) -> list:
    """
    Project the forward-facing angular slice of a 360° scan onto a simple
    ground profile: list of (forward_distance_m, height_relative_to_mount_m),
    sorted by forward distance.

    Since the sensor only ever sees a single plane (pitch FOV ~0-2°, see
    module docstring), a full 3D rotation isn't needed — the unit is
    assumed mounted tilted `mount_tilt_deg` below horizontal (0 = scanning
    straight ahead, 90 = straight down), at `mount_height_m` above the
    ground when standing on flat terrain. Angle 0 in the scan is assumed
    aligned with straight-ahead; only points within `forward_sector_deg`
    of that (left/right in the tilted plane) are kept — the rest of the
    360° revolution is pointing at the wearer's own leg/body and is
    discarded. These mounting parameters (tilt, height, zero-angle
    alignment) need real calibration once physically mounted — see
    WORKING_LOG.md 2026-08-20.
    """
    lo, hi = forward_sector_deg
    tilt_rad = math.radians(mount_tilt_deg)
    cos_tilt, sin_tilt = math.cos(tilt_rad), math.sin(tilt_rad)

    profile = []
    for p in points:
        a = _wrap_angle(p.angle_deg)
        if not (lo <= a <= hi):
            continue
        r = p.distance_mm / 1000.0
        if r <= 0.0 or r > max_range_m:
            continue
        a_rad = math.radians(a)
        cos_a = math.cos(a_rad)
        forward_m = r * cos_a * cos_tilt
        height_m  = mount_height_m - r * cos_a * sin_tilt
        profile.append((forward_m, height_m))
    profile.sort(key=lambda fp: fp[0])
    return profile


def analyze_profile(profile: list, flat_height_tolerance_m: float = 0.03,
                     edge_height_jump_m: float = 0.08) -> dict:
    """
    Simple, explainable geometric heuristics on a ground profile — NOT a
    learned model (the point count per revolution is too low/sparse to
    usefully train a CNN on, unlike the dense D435i projection image).
    Thresholds are reasonable starting guesses, not empirically tuned —
    revisit once real mounted-on-exoskeleton data exists.
    """
    if len(profile) < 3:
        return {"valid": False, "n_points": len(profile)}

    heights = [h for _, h in profile]
    roughness_m = statistics.pstdev(heights)

    max_jump_m = 0.0
    jump_at_m = None
    for (f0, h0), (f1, h1) in zip(profile, profile[1:]):
        jump = abs(h1 - h0)
        if jump > max_jump_m:
            max_jump_m = jump
            jump_at_m = (f0 + f1) / 2.0

    flat_confidence = 1.0 - min(1.0, roughness_m / flat_height_tolerance_m) if flat_height_tolerance_m > 0 else 0.0

    return {
        "valid": True,
        "n_points": len(profile),
        "mean_height_m": statistics.mean(heights),
        "roughness_m": roughness_m,
        "max_height_jump_m": max_jump_m,
        "edge_detected": max_jump_m >= edge_height_jump_m,
        "edge_forward_m": jump_at_m,
        "flat_confidence": flat_confidence,
    }
