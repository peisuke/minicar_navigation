#!/usr/bin/env python3
"""
Path Oscillation Analysis Tool

Replay path selection from debug data to understand why direction jumps occur.
"""

import json
import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from minicar_navigation.planner.local_path_planner import LocalPathPlanner, PathPlannerConfig


def analyze_frames(session_path: str, frame_range: tuple):
    """Analyze path selection for a range of frames."""

    json_path = os.path.join(session_path, 'all_frames.json')
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        return

    with open(json_path) as f:
        data = json.load(f)

    frames = data['frames']
    print(f"Total frames: {len(frames)}")

    # Create a fresh planner for replay
    config = PathPlannerConfig()
    planner = LocalPathPlanner(config)

    start_frame, end_frame = frame_range

    print(f"\n{'='*80}")
    print(f"REPLAY ANALYSIS: Frames {start_frame}-{end_frame}")
    print(f"{'='*80}")

    prev_angle = None

    for frame_idx in range(start_frame, min(end_frame + 1, len(frames))):
        frame = frames[frame_idx]
        scan = frame.get('scan')

        if not scan or 'ranges' not in scan:
            print(f"\n--- Frame {frame_idx}: No scan data ---")
            continue

        # Reconstruct LiDAR data
        ranges = np.array(scan['ranges'], dtype=np.float32)
        angle_min = scan['angle_min']
        angle_max = scan['angle_max']
        angle_increment = scan['angle_increment']

        angles = np.arange(angle_min, angle_max + angle_increment/2, angle_increment, dtype=np.float32)
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        # Filter invalid ranges
        valid_mask = (ranges > 0.1) & (ranges < 10.0)
        ranges = np.where(valid_mask, ranges, 10.0)

        lidar_data = {
            'ranges': ranges,
            'angles': angles
        }

        # Generate paths
        try:
            paths, rough_paths = planner.generate_local_paths(lidar_data)
        except Exception as e:
            print(f"\n--- Frame {frame_idx}: Error generating paths: {e} ---")
            continue

        # Calculate scores for all paths
        path_scores = []
        for i, path in enumerate(paths):
            score, confidence, combined = planner._score_path(path)
            direction = planner._extract_path_direction(path)
            if direction is not None:
                angle_deg = np.degrees(np.arctan2(direction[1], direction[0]))
            else:
                angle_deg = None

            # Calculate path physical length
            if len(path) > 1:
                diffs = np.diff(path, axis=0)
                length = float(np.sum(np.linalg.norm(diffs, axis=1)))
            else:
                length = 0.0

            path_scores.append({
                'idx': i,
                'score': score,
                'confidence': confidence,
                'combined': combined,
                'angle_deg': angle_deg,
                'length': length,
                'num_points': len(path)
            })

        # Select best path (this also updates belief)
        selected_path, selected_idx = planner.select_outermost_path(paths)

        # Get selected path angle
        if len(selected_path) > 0:
            selected_dir = planner._extract_path_direction(selected_path)
            if selected_dir is not None:
                selected_angle = np.degrees(np.arctan2(selected_dir[1], selected_dir[0]))
            else:
                selected_angle = None
        else:
            selected_angle = None

        # Check for direction jump
        is_jump = False
        if prev_angle is not None and selected_angle is not None:
            angle_diff = abs(selected_angle - prev_angle)
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            is_jump = angle_diff > 20

        # Print analysis
        jump_marker = " *** JUMP ***" if is_jump else ""
        print(f"\n--- Frame {frame_idx}{jump_marker} ---")

        # Belief state
        belief_angle = np.degrees(np.arctan2(planner._belief_direction[1], planner._belief_direction[0]))
        print(f"  Belief: [{planner._belief_direction[0]:.3f}, {planner._belief_direction[1]:.3f}] = {belief_angle:.1f}°")
        print(f"  Belief initialized: {planner._belief_initialized}")

        # All path candidates
        print(f"  Path candidates: {len(paths)}")
        path_scores_sorted = sorted(path_scores, key=lambda x: x['score'], reverse=True)

        for ps in path_scores_sorted[:5]:  # Show top 5
            marker = " <-- SELECTED" if ps['idx'] == selected_idx else ""
            angle_str = f"{ps['angle_deg']:.1f}°" if ps['angle_deg'] is not None else "N/A"
            print(f"    [{ps['idx']}] score={ps['score']:.3f} conf={ps['confidence']:.3f} "
                  f"combined={ps['combined']:.3f} angle={angle_str} len={ps['length']:.2f}m{marker}")

        # Selected result
        if selected_angle is not None:
            print(f"  Selected: path[{selected_idx}] angle={selected_angle:.1f}°")
            if prev_angle is not None:
                print(f"  Angle change: {prev_angle:.1f}° → {selected_angle:.1f}° (diff={angle_diff:.1f}°)")
        else:
            print(f"  Selected: None")

        prev_angle = selected_angle


def find_direction_jumps(session_path: str, threshold: float = 20.0):
    """Find all direction jumps in a session."""

    json_path = os.path.join(session_path, 'all_frames.json')
    with open(json_path) as f:
        data = json.load(f)

    frames = data['frames']

    config = PathPlannerConfig()
    planner = LocalPathPlanner(config)

    prev_angle = None
    jumps = []

    for frame_idx, frame in enumerate(frames):
        scan = frame.get('scan')
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

        lidar_data = {'ranges': ranges, 'angles': angles}

        try:
            paths, _ = planner.generate_local_paths(lidar_data)
            selected_path, _ = planner.select_outermost_path(paths)

            if len(selected_path) > 0:
                direction = planner._extract_path_direction(selected_path)
                if direction is not None:
                    current_angle = np.degrees(np.arctan2(direction[1], direction[0]))

                    if prev_angle is not None:
                        diff = abs(current_angle - prev_angle)
                        if diff > 180:
                            diff = 360 - diff
                        if diff > threshold:
                            jumps.append({
                                'frame': frame_idx,
                                'from_angle': prev_angle,
                                'to_angle': current_angle,
                                'diff': diff
                            })

                    prev_angle = current_angle
        except Exception:
            continue

    return jumps


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Analyze path oscillation')
    parser.add_argument('--session', type=str, required=True, help='Session directory path')
    parser.add_argument('--start', type=int, default=350, help='Start frame')
    parser.add_argument('--end', type=int, default=360, help='End frame')
    parser.add_argument('--find-jumps', action='store_true', help='Find all direction jumps')
    args = parser.parse_args()

    if args.find_jumps:
        print("Finding all direction jumps...")
        jumps = find_direction_jumps(args.session)
        print(f"\nFound {len(jumps)} direction jumps:")
        for j in jumps:
            print(f"  Frame {j['frame']}: {j['from_angle']:.1f}° → {j['to_angle']:.1f}° (diff={j['diff']:.1f}°)")
    else:
        analyze_frames(args.session, (args.start, args.end))


if __name__ == '__main__':
    main()
