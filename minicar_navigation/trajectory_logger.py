#!/usr/bin/env python3
"""
Trajectory Logger

ロボットの軌跡を記録してJSONファイルに保存する
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import json
import os
import time
import signal
import sys
from datetime import datetime


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        # パラメータ
        self.declare_parameter('output_dir', '/tmp/batch_test')
        self.declare_parameter('duration', 60.0)  # 記録時間（秒）
        self.declare_parameter('seed', 0)

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.seed = self.get_parameter('seed').get_parameter_value().integer_value

        # 出力ディレクトリ作成
        self.session_dir = os.path.join(self.output_dir, f'seed_{self.seed}')
        os.makedirs(self.session_dir, exist_ok=True)

        # データ格納
        self.trajectory = []
        self.start_time = time.time()
        self.recording = True
        self.should_exit = False  # 終了フラグ

        # サブスクライバー（Gazebo ground truth）
        self.model_name = 'minicar'
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )
        self.prev_vx = 0.0
        self.prev_wz = 0.0

        # タイマー（10Hz で状態チェック）
        self.timer = self.create_timer(0.1, self.check_status)

        # シグナルハンドラ
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info(f'Trajectory Logger started')
        self.get_logger().info(f'  Output: {self.session_dir}')
        self.get_logger().info(f'  Duration: {self.duration}s')
        self.get_logger().info(f'  Seed: {self.seed}')

    def model_states_callback(self, msg: ModelStates):
        if not self.recording:
            return

        # minicarのインデックスを探す
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return  # minicarが見つからない

        elapsed = time.time() - self.start_time
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # 速度はworld座標系なので、そのまま使用
        vx = (twist.linear.x**2 + twist.linear.y**2)**0.5  # 2D速度の大きさ
        wz = twist.angular.z

        self.trajectory.append({
            'timestamp': elapsed,
            'x': pose.position.x,
            'y': pose.position.y,
            'vx': vx,
            'wz': wz
        })

    def check_status(self):
        if not self.recording:
            return

        elapsed = time.time() - self.start_time

        # 進捗表示
        if int(elapsed) % 10 == 0 and len(self.trajectory) > 0:
            last = self.trajectory[-1]
            self.get_logger().info(
                f'Recording: {elapsed:.0f}s, points={len(self.trajectory)}, '
                f'pos=({last["x"]:.2f}, {last["y"]:.2f})'
            )

        # 時間経過で終了
        if elapsed >= self.duration:
            self.recording = False
            self.save_and_exit()

    def signal_handler(self, signum, frame):
        self.get_logger().info('Signal received, saving data...')
        self.recording = False
        self.save_and_exit()

    def save_and_exit(self):
        self.save_trajectory()
        self.get_logger().info('Exiting...')
        self.should_exit = True
        raise SystemExit(0)  # spinから抜けるためにSystemExitを発生

    def save_trajectory(self):
        if len(self.trajectory) == 0:
            self.get_logger().warn('No trajectory data recorded!')
            return

        # 軌跡データ保存
        output_file = os.path.join(self.session_dir, 'trajectory.json')

        # 統計情報を計算
        xs = [p['x'] for p in self.trajectory]
        ys = [p['y'] for p in self.trajectory]

        total_distance = 0.0
        for i in range(1, len(self.trajectory)):
            dx = self.trajectory[i]['x'] - self.trajectory[i-1]['x']
            dy = self.trajectory[i]['y'] - self.trajectory[i-1]['y']
            total_distance += (dx**2 + dy**2) ** 0.5

        data = {
            'seed': self.seed,
            'duration': self.duration,
            'recorded_at': datetime.now().isoformat(),
            'num_points': len(self.trajectory),
            'total_distance': total_distance,
            'start_pos': {'x': xs[0], 'y': ys[0]} if xs else None,
            'end_pos': {'x': xs[-1], 'y': ys[-1]} if xs else None,
            'bounds': {
                'x_min': min(xs), 'x_max': max(xs),
                'y_min': min(ys), 'y_max': max(ys)
            } if xs else None,
            'trajectory': self.trajectory
        }

        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Saved {len(self.trajectory)} points to {output_file}')
        self.get_logger().info(f'Total distance: {total_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # should_exitがTrueの場合は既に保存済み
        if not node.should_exit:
            node.save_trajectory()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
