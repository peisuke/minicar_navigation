#!/usr/bin/env python3
"""
Debug Frame Plotter

指定したフレームのセンサデータとローカルパスをプロットする
"""

import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# path plannerをインポート
sys.path.insert(0, str(Path(__file__).parent.parent / 'minicar_navigation' / 'planner'))
from local_path_planner import LocalPathPlanner, PathPlannerConfig


def load_debug_data(json_path: str) -> dict:
    """デバッグデータを読み込む"""
    with open(json_path, 'r') as f:
        return json.load(f)


def process_scan(scan: dict) -> dict:
    """scanデータをlidar_data形式に変換"""
    ranges = np.asarray(scan['ranges'], dtype=np.float32)
    n = len(ranges)
    angles = scan['angle_min'] + np.arange(n, dtype=np.float32) * scan['angle_increment']
    max_r = float(scan.get('range_max', 200.0)) if scan.get('range_max', 0.0) > 0.0 else 200.0
    range_min = float(scan.get('range_min', 0.0)) if scan.get('range_min', 0.0) > 0.0 else 0.0
    ranges = np.where(np.isfinite(ranges), ranges, max_r)
    ranges = np.where((ranges >= range_min) & (ranges <= max_r), ranges, max_r)
    return {'ranges': ranges, 'angles': angles}


def scan_to_cartesian(scan: dict) -> np.ndarray:
    """scanデータを直交座標に変換"""
    lidar_data = process_scan(scan)
    ranges = lidar_data['ranges']
    angles = lidar_data['angles']
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.stack([x, y], axis=1)


def plot_frame(data: dict, frame_idx: int, output_path: str = None, show_regenerated: bool = True):
    """フレームをプロット"""
    frames = data['frames']
    total_frames = data['total_frames']

    if frame_idx < 0 or frame_idx >= total_frames:
        print(f"Error: frame_idx must be 0-{total_frames-1}")
        return

    frame = frames[frame_idx]
    scan = frame['scan']
    logged_path = frame['local_path']
    odom = frame['odom']

    # センサデータを直交座標に変換
    scan_points = scan_to_cartesian(scan)

    # ログされたパスを取得
    logged_poses = np.array([[p['x'], p['y']] for p in logged_path['poses']])

    # 再生成パス（1フレーム前のscanを使用）
    regenerated_path = None
    if show_regenerated and frame_idx > 0:
        prev_scan = frames[frame_idx - 1]['scan']
        lidar_data = process_scan(prev_scan)

        config = PathPlannerConfig()
        planner = LocalPathPlanner(config)

        try:
            paths, _ = planner.generate_local_paths(lidar_data)
            if paths:
                regenerated_path, _ = planner.select_outermost_path(paths)
        except Exception as e:
            print(f"Warning: Failed to regenerate path: {e}")

    # プロット作成
    fig, ax = plt.subplots(figsize=(10, 10))

    # センサデータ（点群）
    ax.scatter(scan_points[:, 0], scan_points[:, 1],
               s=2, c='gray', alpha=0.5, label='LiDAR Scan')

    # ログされたパス
    if len(logged_poses) > 0:
        ax.plot(logged_poses[:, 0], logged_poses[:, 1],
                'b-', linewidth=2, label='Logged Path')
        ax.scatter(logged_poses[:, 0], logged_poses[:, 1],
                   c='blue', s=30, zorder=5)

    # 再生成パス
    if regenerated_path is not None and len(regenerated_path) > 0:
        ax.plot(regenerated_path[:, 0], regenerated_path[:, 1],
                'r--', linewidth=2, label='Regenerated Path (scan N-1)')
        ax.scatter(regenerated_path[:, 0], regenerated_path[:, 1],
                   c='red', s=20, zorder=5, marker='x')

    # ロボット位置（原点）
    ax.scatter([0], [0], c='green', s=100, marker='o', zorder=10, label='Robot')
    ax.arrow(0, 0, 0.1, 0, head_width=0.02, head_length=0.01, fc='green', ec='green')

    # オドメトリ情報
    qz = odom['orientation']['z']
    qw = odom['orientation']['w']
    yaw = 2 * np.arctan2(qz, qw)

    # 軸設定
    ax.set_xlim(-3.0, 3.0)
    ax.set_ylim(-3.0, 3.0)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)

    # ラベル
    ax.set_xlabel('X [m] (Forward)')
    ax.set_ylabel('Y [m] (Left)')
    ax.set_title(f'Frame {frame_idx} / {total_frames-1}\n'
                 f'Odom: ({odom["position"]["x"]:.3f}, {odom["position"]["y"]:.3f}), '
                 f'Yaw: {np.degrees(yaw):.1f}°')
    ax.legend(loc='upper right')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Saved to {output_path}")
    else:
        plt.show()

    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Plot debug frame data')
    parser.add_argument('json_path', type=str,
                        help='Path to all_frames.json')
    parser.add_argument('frame_idx', type=int,
                        help='Frame index to plot')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output image path (if not specified, show interactively)')
    parser.add_argument('--no-regenerate', action='store_true',
                        help='Do not show regenerated path')

    args = parser.parse_args()

    # データ読み込み
    print(f"Loading {args.json_path}...")
    data = load_debug_data(args.json_path)
    print(f"Total frames: {data['total_frames']}")

    # プロット
    plot_frame(data, args.frame_idx, args.output, not args.no_regenerate)


if __name__ == '__main__':
    main()
