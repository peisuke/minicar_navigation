#!/usr/bin/env python3
"""
Path Generation Analysis Tool

Analyze why specific path candidates appear/disappear between frames.
"""

import json
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from minicar_navigation.planner.local_path_planner import (
    LocalPathPlanner, PathPlannerConfig, PathPipeline, PeakDetectorParams
)


def analyze_path_generation(session_path: str, frame_indices: list):
    """Analyze path generation details for specific frames."""

    json_path = os.path.join(session_path, 'all_frames.json')
    with open(json_path) as f:
        data = json.load(f)

    frames = data['frames']
    config = PathPlannerConfig()
    planner = LocalPathPlanner(config)
    pipeline = PathPipeline(config=config)

    for frame_idx in frame_indices:
        frame = frames[frame_idx]
        scan = frame.get('scan', {})

        if not scan or 'ranges' not in scan:
            continue

        ranges = np.array(scan['ranges'], dtype=np.float32)
        angle_min = scan['angle_min']
        angle_max = scan['angle_max']
        angle_increment = scan['angle_increment']

        angles = np.arange(angle_min, angle_max + angle_increment/2, angle_increment, dtype=np.float32)
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        valid_mask = (ranges > 0.1) & (ranges < 10.0)
        ranges = np.where(valid_mask, ranges, 10.0)

        # Convert to view points
        view_points = planner._polar_to_view_points(ranges, angles, config.HEIGHT, config.WIDTH)

        print(f"\n{'='*80}")
        print(f"Frame {frame_idx} - Path Generation Analysis")
        print(f"{'='*80}")

        # Run pipeline and get intermediate results
        peak_detector_params = PeakDetectorParams(
            H=config.HEIGHT, W=config.WIDTH,
            R_in=np.array(config.RING_RADII, dtype=np.int32),
            ring_thickness=config.RING_THICKNESS,
            front_deg=config.FRONT_DEG,
            dist_thresh=config.LOCAL_DIST_THRESH,
            border_thickness=config.BORDER_THICKNESS
        )

        peaks, centered_points, sorted_labels, dist_inside, dist_norm, center_x, center_y = \
            pipeline.peak_detector.detect_from_view_points(view_points, peak_detector_params)

        print(f"\n1. Peak Detection:")
        print(f"   Total peaks found: {len(peaks)}")
        print(f"   Centered points shape: {centered_points.shape}")
        print(f"   Unique labels: {np.unique(sorted_labels)}")

        # Show peak details
        print(f"\n   Peak details (x, y from center, dist_value, label):")
        for i, (px, py, val, label) in enumerate(peaks[:10]):  # Show first 10
            # Calculate angle from center
            cx, cy = center_x, center_y
            dx, dy = px - cx, py - cy
            angle_deg = np.degrees(np.arctan2(dy, dx))
            dist_from_center = np.sqrt(dx**2 + dy**2) * config.MAP_RESOLUTION
            print(f"     Peak[{i}]: angle={angle_deg:.1f}°, dist={dist_from_center:.2f}m, value={val:.0f}, label={label}")

        # Build paths
        coordinate_paths = pipeline.path_searcher.build_and_deduplicate_paths(
            sorted_labels, centered_points, dist_inside, center_x, center_y,
            config.LOCAL_DIST_THRESH, config.PATH_FRONT_DEG, config.START_IDS
        )

        print(f"\n2. Path Construction:")
        print(f"   Rough paths generated: {len(coordinate_paths)}")

        for i, path in enumerate(coordinate_paths):
            if len(path) >= 2:
                # Path direction from start to end
                dx = path[-1, 0] - path[0, 0]
                dy = path[-1, 1] - path[0, 1]
                angle_deg = np.degrees(np.arctan2(dy, dx))
                length = np.sqrt(dx**2 + dy**2) * config.MAP_RESOLUTION
                print(f"     Path[{i}]: angle={angle_deg:.1f}°, length={length:.2f}m, points={len(path)}")

                # Show path node angles
                node_angles = []
                for pt in path:
                    a = np.degrees(np.arctan2(pt[1], pt[0]))
                    node_angles.append(f"{a:.0f}°")
                print(f"              nodes: {' → '.join(node_angles)}")

        # Analyze why certain peaks don't form paths
        print(f"\n3. Path Candidate Analysis:")

        # Group peaks by rough angle
        peak_angles = []
        for px, py, val, label in peaks:
            dx, dy = px - center_x, py - center_y
            angle = np.degrees(np.arctan2(dy, dx))
            peak_angles.append(angle)

        # Check if there are peaks in the ~10-15° range
        forward_peaks = [(i, peaks[i]) for i, a in enumerate(peak_angles) if 0 <= a <= 30]
        left_peaks = [(i, peaks[i]) for i, a in enumerate(peak_angles) if 40 <= a <= 70]

        print(f"   Forward peaks (0-30°): {len(forward_peaks)}")
        for i, (px, py, val, label) in forward_peaks[:5]:
            dx, dy = px - center_x, py - center_y
            angle = np.degrees(np.arctan2(dy, dx))
            dist = np.sqrt(dx**2 + dy**2) * config.MAP_RESOLUTION
            print(f"     [{i}] angle={angle:.1f}°, dist={dist:.2f}m, label={label}")

        print(f"   Left peaks (40-70°): {len(left_peaks)}")
        for i, (px, py, val, label) in left_peaks[:5]:
            dx, dy = px - center_x, py - center_y
            angle = np.degrees(np.arctan2(dy, dx))
            dist = np.sqrt(dx**2 + dy**2) * config.MAP_RESOLUTION
            print(f"     [{i}] angle={angle:.1f}°, dist={dist:.2f}m, label={label}")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--frames', type=str, default='352,353,354,355',
                        help='Comma-separated frame indices')
    args = parser.parse_args()

    frame_indices = [int(f) for f in args.frames.split(',')]
    analyze_path_generation(args.session, frame_indices)


if __name__ == '__main__':
    main()
