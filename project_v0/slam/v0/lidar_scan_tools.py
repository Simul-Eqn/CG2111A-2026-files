#!/usr/bin/env python3
"""
Helpers for validating and stitching LiDAR scan rounds.
"""

from __future__ import annotations

from collections.abc import Sequence


def normalize_angle_deg(angle: float) -> float:
    """Return angle in the half-open range [0, 360)."""
    value = float(angle) % 360.0
    return value if value >= 0.0 else value + 360.0


def sort_scan_pairs(
    angles: Sequence[float],
    distances: Sequence[float],
) -> tuple[list[float], list[float]]:
    """Sort parallel angle and distance arrays by normalized angle."""
    pairs = sorted(
        (
            normalize_angle_deg(angle),
            float(distance),
        )
        for angle, distance in zip(angles, distances)
    )
    return [angle for angle, _ in pairs], [distance for _, distance in pairs]


def merge_scan_segments(
    *segments: tuple[Sequence[float], Sequence[float]],
) -> tuple[list[float], list[float]]:
    """Concatenate scan fragments and return them sorted by angle."""
    merged_angles: list[float] = []
    merged_distances: list[float] = []

    for angles, distances in segments:
        merged_angles.extend(angles)
        merged_distances.extend(distances)

    return sort_scan_pairs(merged_angles, merged_distances)


def circular_coverage_deg(angles: Sequence[float]) -> float:
    """Estimate how much of the 360-degree circle the scan covers."""
    if len(angles) < 2:
        return 0.0

    values = sorted(normalize_angle_deg(angle) for angle in angles)
    largest_gap = 0.0

    for prev, curr in zip(values, values[1:]):
        largest_gap = max(largest_gap, curr - prev)

    wrap_gap = (values[0] + 360.0) - values[-1]
    largest_gap = max(largest_gap, wrap_gap)

    return max(0.0, 360.0 - largest_gap)


def scan_is_complete(
    angles: Sequence[float],
    *,
    min_points: int,
    min_coverage_deg: float,
) -> bool:
    """Return True when a scan looks like a full rotation."""
    if len(angles) < int(min_points):
        return False

    return circular_coverage_deg(angles) >= float(min_coverage_deg)
