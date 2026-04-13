#!/usr/bin/env python3

import os
import sys
import unittest


sys.path.insert(0, os.path.dirname(__file__))

from lidar_scan_tools import (  # noqa: E402
    circular_coverage_deg,
    merge_scan_segments,
    normalize_angle_deg,
    scan_is_complete,
    sort_scan_pairs,
)


class LidarScanToolsTests(unittest.TestCase):
    def test_normalize_angle_deg_wraps_into_circle(self):
        self.assertAlmostEqual(normalize_angle_deg(-10.0), 350.0)
        self.assertAlmostEqual(normalize_angle_deg(370.0), 10.0)

    def test_circular_coverage_detects_full_rotation(self):
        angles = [float(angle) for angle in range(360)]
        self.assertGreaterEqual(circular_coverage_deg(angles), 359.0)

    def test_scan_is_complete_rejects_small_partial_segment(self):
        angles = [float(angle) for angle in range(25)]
        self.assertFalse(
            scan_is_complete(
                angles,
                min_points=120,
                min_coverage_deg=300.0,
            )
        )

    def test_merge_scan_segments_restores_tiny_split_rotation(self):
        first_angles = [0.0]
        first_distances = [1000.0]
        second_angles = [float(angle) for angle in range(1, 360)]
        second_distances = [1000.0] * len(second_angles)

        merged_angles, merged_distances = merge_scan_segments(
            (first_angles, first_distances),
            (second_angles, second_distances),
        )

        self.assertEqual(len(merged_angles), 360)
        self.assertEqual(len(merged_distances), 360)
        self.assertTrue(
            scan_is_complete(
                merged_angles,
                min_points=120,
                min_coverage_deg=300.0,
            )
        )

    def test_sort_scan_pairs_orders_wraparound_segments(self):
        angles, distances = sort_scan_pairs(
            [350.0, -5.0, 10.0],
            [1.0, 2.0, 3.0],
        )

        self.assertEqual(angles, [10.0, 350.0, 355.0])
        self.assertEqual(distances, [3.0, 1.0, 2.0])


if __name__ == "__main__":
    unittest.main()
