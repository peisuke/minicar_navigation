#!/usr/bin/env python3
"""
Analyze path edge construction to find instability source.
"""

import json
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from minicar_navigation.planner.local_path_planner import (
    LocalPathPlanner, PathPlannerConfig, PeakDetector, GraphPathSearcher,
    PeakDetectorParams, DistanceFieldGenerator
)


def analyze_edge_construction(session_path: str, frame_indices: list):
    """Analyze edge construction details."""

    json_path = os.path.join(session_path, 'all_frames.json')
    with open(json_path) as f:
        data = json.load(f)

    frames = data['frames']
    config = PathPlannerConfig()
    planner = LocalPathPlanner(config)
    peak_detector = PeakDetector(config=config)
    path_searcher = GraphPathSearcher(config)

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

        view_points = planner._polar_to_view_points(ranges, angles, config.HEIGHT, config.WIDTH)

        print(f"\n{'='*80}")
        print(f"Frame {frame_idx} - Edge Construction Analysis")
        print(f"{'='*80}")

        # Step 1: Peak detection
        peak_params = PeakDetectorParams(
            H=config.HEIGHT, W=config.WIDTH,
            R_in=np.array(config.RING_RADII, dtype=np.int32),
            ring_thickness=config.RING_THICKNESS,
            front_deg=config.FRONT_DEG,
            dist_thresh=config.LOCAL_DIST_THRESH,
            border_thickness=config.BORDER_THICKNESS
        )

        peaks, centered_points, sorted_labels, dist_inside, dist_norm, cx, cy = \
            peak_detector.detect_from_view_points(view_points, peak_params)

        print(f"\n1. Peaks and Labels:")
        print(f"   sorted_labels: {sorted_labels}")

        # Show peaks grouped by label
        unique_labels = np.unique(sorted_labels)
        print(f"   Unique labels (distance layers): {unique_labels}")

        for label in unique_labels:
            mask = sorted_labels == label
            indices = np.where(mask)[0]
            print(f"\n   Layer {label}:")
            for idx in indices:
                pt = centered_points[idx]
                angle = np.degrees(np.arctan2(pt[1], pt[0]))
                dist = np.linalg.norm(pt) * config.MAP_RESOLUTION
                print(f"     Peak[{idx}]: angle={angle:.1f}°, dist={dist:.2f}m, pos=({pt[0]}, {pt[1]})")

        # Step 2: Edge construction (detailed)
        print(f"\n2. Edge Construction:")

        # Get intermediate results from build_valid_paths_coords
        result = path_searcher.build_valid_paths_coords(
            labels=sorted_labels,
            centered_pts=centered_points,
            dist_map=dist_inside,
            cx=cx, cy=cy,
            thr=config.LOCAL_DIST_THRESH,
            front_deg=config.PATH_FRONT_DEG,
            start_ids=list(config.START_IDS)
        )

        paths_ids = result["paths_ids"]
        flat_to_pts = result["flat_to_pts"]

        print(f"   Raw paths found: {len(paths_ids)}")

        for i, path_ids in enumerate(paths_ids):
            print(f"\n   Path {i}: {path_ids}")
            angles_in_path = []
            for node_id in path_ids:
                pt_idx = flat_to_pts[node_id]
                pt = centered_points[pt_idx]
                angle = np.degrees(np.arctan2(pt[1], pt[0]))
                angles_in_path.append(f"{angle:.0f}°")
            print(f"     Angles: {' → '.join(angles_in_path)}")

        # Step 3: Analyze edge validity between layers
        print(f"\n3. Edge Validity Analysis (between adjacent layers):")

        # Build layer indices
        layer_idx = path_searcher._build_layer_indices(sorted_labels)
        offsets, counts = path_searcher._build_offsets(layer_idx)

        print(f"   Layer structure:")
        for i, idx_list in enumerate(layer_idx):
            if len(idx_list) > 0:
                angles = [np.degrees(np.arctan2(centered_points[j][1], centered_points[j][0]))
                          for j in idx_list]
                print(f"     Layer {i}: {len(idx_list)} peaks, angles: {[f'{a:.0f}°' for a in angles]}")

        # Check edges between layers
        print(f"\n   Edge validity between layers:")
        for layer_i in range(len(layer_idx) - 1):
            if len(layer_idx[layer_i]) == 0 or len(layer_idx[layer_i + 1]) == 0:
                continue

            print(f"\n   Layer {layer_i} → Layer {layer_i + 1}:")

            for src_local in range(len(layer_idx[layer_i])):
                src_idx = layer_idx[layer_i][src_local]
                src_pt = centered_points[src_idx]
                src_angle = np.degrees(np.arctan2(src_pt[1], src_pt[0]))

                for dst_local in range(len(layer_idx[layer_i + 1])):
                    dst_idx = layer_idx[layer_i + 1][dst_local]
                    dst_pt = centered_points[dst_idx]
                    dst_angle = np.degrees(np.arctan2(dst_pt[1], dst_pt[0]))

                    # Check midpoint validity
                    mid = (src_pt + dst_pt) * 0.5
                    mid_x = int(np.rint(mid[0])) + cx
                    mid_y = int(np.rint(mid[1])) + cy
                    mid_x = np.clip(mid_x, 0, dist_inside.shape[1] - 1)
                    mid_y = np.clip(mid_y, 0, dist_inside.shape[0] - 1)
                    mid_dist = dist_inside[mid_y, mid_x]

                    # Check direction
                    direction = dst_pt - src_pt
                    is_forward = direction[0] >= 0

                    # Check gradient
                    src_x = int(np.rint(src_pt[0])) + cx
                    src_y = int(np.rint(src_pt[1])) + cy
                    src_x = np.clip(src_x, 0, dist_inside.shape[1] - 1)
                    src_y = np.clip(src_y, 0, dist_inside.shape[0] - 1)
                    src_dist = dist_inside[src_y, src_x]

                    dst_x = int(np.rint(dst_pt[0])) + cx
                    dst_y = int(np.rint(dst_pt[1])) + cy
                    dst_x = np.clip(dst_x, 0, dist_inside.shape[1] - 1)
                    dst_y = np.clip(dst_y, 0, dist_inside.shape[0] - 1)
                    dst_dist = dist_inside[dst_y, dst_x]

                    gradient = dst_dist - src_dist
                    gradient_ok = gradient >= -config.MAX_GRADIENT_DROP

                    valid = (mid_dist >= config.LOCAL_DIST_THRESH) and is_forward and gradient_ok

                    status = "✓" if valid else "✗"
                    reason = ""
                    if not valid:
                        if mid_dist < config.LOCAL_DIST_THRESH:
                            reason = f"mid_dist={mid_dist:.1f}<{config.LOCAL_DIST_THRESH}"
                        elif not is_forward:
                            reason = "backward"
                        elif not gradient_ok:
                            reason = f"gradient={gradient:.1f}<-{config.MAX_GRADIENT_DROP}"

                    print(f"     {src_angle:.0f}° → {dst_angle:.0f}°: {status} {reason}")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--frames', type=str, default='352,353')
    args = parser.parse_args()

    frame_indices = [int(f) for f in args.frames.split(',')]
    analyze_edge_construction(args.session, frame_indices)


if __name__ == '__main__':
    main()
