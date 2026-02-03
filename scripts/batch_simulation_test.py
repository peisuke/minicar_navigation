#!/usr/bin/env python3
"""
Batch Simulation Test

複数のシードでシミュレーションを実行し、パス安定性をテストする
"""

import subprocess
import time
import signal
import sys
import os
import json
import numpy as np
from pathlib import Path
from datetime import datetime


def run_simulation(seed: int, duration: int = 60, gui: bool = False) -> dict:
    """シミュレーションを実行して結果を返す"""

    result = {
        'seed': seed,
        'duration': duration,
        'success': False,
        'debug_dir': None,
        'error': None
    }

    sim_process = None
    nav_process = None

    try:
        print(f"\n{'='*60}")
        print(f"Seed {seed}: Starting simulation...")
        print(f"{'='*60}")

        # シミュレーション起動
        sim_cmd = [
            'ros2', 'launch', 'minicar_simulation', 'road_env_ackermann.launch.py',
            f'seed:={seed}',
            f'gui:={"true" if gui else "false"}'
        ]
        sim_process = subprocess.Popen(
            sim_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        # シミュレーション起動待ち
        print(f"Seed {seed}: Waiting for simulation to start...")
        time.sleep(10)

        # ナビゲーション起動（record_scan:=falseで軽量記録）
        nav_cmd = [
            'ros2', 'launch', 'minicar_navigation', 'local_nav.launch.py',
            'input_sim:=true', 'output_sim:=true',
            'input_real:=false', 'output_real:=false',
            'record:=true', 'record_scan:=false', 'robot_type:=ackermann'
        ]
        nav_process = subprocess.Popen(
            nav_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        print(f"Seed {seed}: Running for {duration} seconds...")
        time.sleep(duration)

        result['success'] = True

    except Exception as e:
        result['error'] = str(e)
        print(f"Seed {seed}: Error - {e}")

    finally:
        # プロセス終了
        print(f"Seed {seed}: Stopping processes...")

        if nav_process:
            try:
                os.killpg(os.getpgid(nav_process.pid), signal.SIGINT)
                nav_process.wait(timeout=10)
            except:
                os.killpg(os.getpgid(nav_process.pid), signal.SIGKILL)

        if sim_process:
            try:
                os.killpg(os.getpgid(sim_process.pid), signal.SIGINT)
                sim_process.wait(timeout=10)
            except:
                os.killpg(os.getpgid(sim_process.pid), signal.SIGKILL)

        time.sleep(3)

    # 最新のデバッグディレクトリを取得
    debug_base = Path('/tmp/debug_data')
    if debug_base.exists():
        sessions = sorted(debug_base.glob('session_*'), key=lambda p: p.stat().st_mtime)
        if sessions:
            result['debug_dir'] = str(sessions[-1])

    return result


def analyze_session(session_dir: str) -> dict:
    """セッションデータを分析"""

    analysis = {
        'total_frames': 0,
        'frames_with_path': 0,
        'direction_jumps': 0,
        'jump_frames': [],
        'avg_path_length': 0,
        'path_coverage': 0,
        'stability_score': 0
    }

    json_path = Path(session_dir) / 'all_frames.json'
    if not json_path.exists():
        return analysis

    with open(json_path) as f:
        data = json.load(f)

    frames = data['frames']
    analysis['total_frames'] = len(frames)

    prev_angle = None
    jump_threshold = 20  # degrees
    path_lengths = []

    for i, frame in enumerate(frames):
        if frame['local_path'] and frame['local_path']['num_points'] > 0:
            analysis['frames_with_path'] += 1
            path_lengths.append(frame['local_path']['num_points'])

            # パス方向を計算
            poses = frame['local_path']['poses']
            if len(poses) >= 2:
                dx = poses[-1]['x'] - poses[0]['x']
                dy = poses[-1]['y'] - poses[0]['y']
                angle = np.degrees(np.arctan2(dy, dx))

                if prev_angle is not None:
                    diff = abs(angle - prev_angle)
                    if diff > 180:
                        diff = 360 - diff
                    if diff > jump_threshold:
                        analysis['direction_jumps'] += 1
                        analysis['jump_frames'].append(i)

                prev_angle = angle

    if path_lengths:
        analysis['avg_path_length'] = np.mean(path_lengths)

    analysis['path_coverage'] = analysis['frames_with_path'] / max(1, analysis['total_frames'])

    # 安定性スコア (0-100)
    # - ジャンプが少ないほど良い
    # - パスカバレッジが高いほど良い
    jump_penalty = min(analysis['direction_jumps'] * 5, 50)
    coverage_score = analysis['path_coverage'] * 50
    analysis['stability_score'] = max(0, 100 - jump_penalty + coverage_score - 50)

    return analysis


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Batch simulation test')
    parser.add_argument('--seeds', type=str, default='1,2,3,4,5,6',
                       help='Comma-separated seed values')
    parser.add_argument('--duration', type=int, default=60,
                       help='Duration per seed in seconds')
    parser.add_argument('--gui', action='store_true',
                       help='Show GUI')
    args = parser.parse_args()

    seeds = [int(s.strip()) for s in args.seeds.split(',')]

    print(f"Running batch simulation test")
    print(f"Seeds: {seeds}")
    print(f"Duration: {args.duration}s per seed")
    print(f"GUI: {args.gui}")

    results = []

    for seed in seeds:
        result = run_simulation(seed, args.duration, args.gui)

        if result['debug_dir']:
            analysis = analyze_session(result['debug_dir'])
            result['analysis'] = analysis

            print(f"\nSeed {seed} Analysis:")
            print(f"  Frames: {analysis['total_frames']}")
            print(f"  Path coverage: {analysis['path_coverage']*100:.1f}%")
            print(f"  Direction jumps: {analysis['direction_jumps']}")
            print(f"  Stability score: {analysis['stability_score']:.1f}/100")

        results.append(result)

    # サマリー
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"{'Seed':<6} {'Frames':<8} {'Coverage':<10} {'Jumps':<8} {'Score':<8}")
    print("-" * 40)

    for r in results:
        if 'analysis' in r:
            a = r['analysis']
            print(f"{r['seed']:<6} {a['total_frames']:<8} {a['path_coverage']*100:>6.1f}%   {a['direction_jumps']:<8} {a['stability_score']:<8.1f}")
        else:
            print(f"{r['seed']:<6} {'ERROR':<8}")

    # 結果をJSONで保存
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_path = f'/tmp/batch_test_{timestamp}.json'
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nResults saved to: {output_path}")


if __name__ == '__main__':
    main()
