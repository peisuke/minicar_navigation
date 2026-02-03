#!/usr/bin/env python3
"""
Debug Data Recorder for Local Path Planner

フレームごとのデータを記録してデバッグ用に保存
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import json
import time
import os
from datetime import datetime


class DebugRecorder(Node):
    def __init__(self):
        super().__init__('debug_recorder')

        # パラメータ
        self.declare_parameter('output_dir', '/tmp/debug_data')
        self.declare_parameter('max_frames', 1000)
        self.declare_parameter('input_sim', True)
        self.declare_parameter('input_real', False)
        self.declare_parameter('sim_ns', 'sim_robot')
        self.declare_parameter('real_ns', 'real_robot')
        self.declare_parameter('robot_type', 'diff')
        self.declare_parameter('record_scan', False)  # LiDARデータを記録するか（容量大）

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_frames = self.get_parameter('max_frames').get_parameter_value().integer_value
        self.record_scan = self.get_parameter('record_scan').get_parameter_value().bool_value

        # 名前空間の決定
        input_sim = self.get_parameter('input_sim').get_parameter_value().bool_value
        input_real = self.get_parameter('input_real').get_parameter_value().bool_value
        sim_ns = self.get_parameter('sim_ns').get_parameter_value().string_value
        real_ns = self.get_parameter('real_ns').get_parameter_value().string_value

        if input_real:
            self.ns = real_ns
        elif input_sim:
            self.ns = sim_ns
        else:
            self.ns = sim_ns  # fallback

        # 出力ディレクトリ作成
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_dir, f'session_{timestamp}')
        os.makedirs(self.session_dir, exist_ok=True)

        # データ格納
        self.frames = []
        self.current_frame = {
            'timestamp': None,
            'scan': None,
            'odom': None,
            'local_path': None,
            'cmd_vel': None
        }
        self.frame_count = 0
        self.recording = True

        # robot_typeに応じたtopic名
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        if robot_type == 'ackermann':
            controller_prefix = 'ackermann_steering_controller'
            odom_suffix = 'odometry'
            cmd_vel_suffix = 'reference_unstamped'
        else:
            controller_prefix = 'diff_drive_controller'
            odom_suffix = 'odom'
            cmd_vel_suffix = 'cmd_vel_unstamped'

        # サブスクライバー
        scan_topic = f'/{self.ns}/scan'
        odom_topic = f'/{self.ns}/{controller_prefix}/{odom_suffix}'
        cmd_vel_topic = f'/{self.ns}/{controller_prefix}/{cmd_vel_suffix}'

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/local_paths', self.path_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info(f'Subscribing to: {scan_topic}, {odom_topic}, {cmd_vel_topic}')

        # フレーム保存タイマー (10Hz)
        self.timer = self.create_timer(0.1, self.save_frame)

        self.get_logger().info(f'Debug Recorder started. Output: {self.session_dir}')
        self.get_logger().info(f'Max frames: {self.max_frames}, Record scan: {self.record_scan}')

    def scan_callback(self, msg: LaserScan):
        self.current_frame['scan'] = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'ranges': list(msg.ranges),
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def odom_callback(self, msg: Odometry):
        self.current_frame['odom'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y
            },
            'angular_velocity': msg.twist.twist.angular.z
        }

    def path_callback(self, msg: Path):
        poses = []
        for pose in msg.poses:
            poses.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            })
        self.current_frame['local_path'] = {
            'frame_id': msg.header.frame_id,
            'poses': poses,
            'num_points': len(poses)
        }

    def cmd_vel_callback(self, msg: Twist):
        self.current_frame['cmd_vel'] = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }

    def save_frame(self):
        if not self.recording:
            return

        if self.frame_count >= self.max_frames:
            self.recording = False
            self.save_all_frames()
            self.get_logger().info(f'Recording complete. {self.frame_count} frames saved.')
            return

        # フレームをコピーして保存
        frame = {
            'frame_id': self.frame_count,
            'timestamp': time.time(),
            'scan': self.current_frame['scan'] if self.record_scan else None,
            'odom': self.current_frame['odom'],
            'local_path': self.current_frame['local_path'],
            'cmd_vel': self.current_frame['cmd_vel']
        }
        self.frames.append(frame)

        # 進捗表示
        if self.frame_count % 50 == 0:
            self.get_logger().info(f'Recorded {self.frame_count} frames...')

        self.frame_count += 1

    def save_all_frames(self):
        # 全フレームを1つのJSONファイルに保存
        output_file = os.path.join(self.session_dir, 'all_frames.json')
        with open(output_file, 'w') as f:
            json.dump({
                'total_frames': len(self.frames),
                'frames': self.frames
            }, f, indent=2)
        self.get_logger().info(f'Saved to {output_file}')

        # サマリー保存
        summary_file = os.path.join(self.session_dir, 'summary.txt')
        with open(summary_file, 'w') as f:
            f.write(f'Total frames: {len(self.frames)}\n')
            f.write(f'Duration: {self.frames[-1]["timestamp"] - self.frames[0]["timestamp"]:.2f}s\n')

            # パス統計
            path_counts = [f['local_path']['num_points'] if f['local_path'] else 0 for f in self.frames]
            f.write(f'Path points (avg): {sum(path_counts)/len(path_counts):.1f}\n')

        self.get_logger().info(f'Summary saved to {summary_file}')


def main(args=None):
    rclpy.init(args=args)
    node = DebugRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted. Saving data...')
        node.save_all_frames()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
