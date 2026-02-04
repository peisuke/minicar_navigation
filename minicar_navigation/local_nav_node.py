#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
import threading

from .planner import local_path_planner
from .controller import ControllerFactory

class LocalNavNode(Node):
    # 緊急停止パラメータ
    EMERGENCY_STOP_DIST = 0.10  # 10cm以内に障害物があれば停止
    EMERGENCY_STOP_CONE_DEG = 15.0  # 前方±15°のコーンを検査
    EMERGENCY_STOP_RATIO = 0.3  # コーン内の30%が閾値以下で停止（ノイズ除去）

    def __init__(self):
        super().__init__("local_nav_node", automatically_declare_parameters_from_overrides=True)

        # パラメータはautomatically_declare_parameters_from_overridesで自動宣言される
        self.declare_parameter("controller_type_override", "")

        # robot_type: diff or ackermann (デフォルト: diff)
        if not self.has_parameter("robot_type"):
            self.declare_parameter("robot_type", "diff")

        # プランナーパラメータを明示的に宣言（動的変更可能にするため）
        # YAMLで既に宣言されている場合はスキップ
        if not self.has_parameter("planner.direction_bias_deg"):
            self.declare_parameter("planner.direction_bias_deg", 0.0)

        # ---- Subscriber (LiDAR) ----
        self.scan_sub = self._setup_input_subscribers()

        # ---- Publisher (cmd_vel) ----
        self.cmd_publishers = self._setup_output_publishers()
        
        # ---- Publisher (paths for visualization) ----
        self.path_pub = self.create_publisher(
            Path,
            "/local_paths",
            10,
        )

        # ---- Timer for control loop ----
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info("LocalNavNode started")

        # 以前の形式に合わせるための保存領域
        self.received_scan = False
        self.latest_lidar_data = None  # {'ranges': np.ndarray, 'angles': np.ndarray}

        # プランナーとコントローラーの初期化
        planner_config = self._create_planner_config()
        self.planner = local_path_planner.LocalPathPlanner(config=planner_config)        
        
        self._ctrl_lock = threading.Lock()
        self.add_on_set_parameters_callback(self._on_parameters_updated)
        
        self._initialize_controller()

    def scan_callback(self, msg: LaserScan):
        """
        LaserScan -> generate_local_paths向けの lidar_data に変換して保存
        """
        # angles: angle_min + i * angle_increment
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # ranges: inf/nan を range_max に置換（従来の max_range 的な扱い）
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        # LaserScan の range_max は「測距可能最大距離」。
        # もし従来の 200 など固定値で統一したいならここを上書きしてOK。
        max_r = float(msg.range_max) if msg.range_max > 0.0 else 200.0

        # 無効値を max_r に
        ranges = np.where(np.isfinite(ranges), ranges, max_r)

        # 範囲外（range_min 未満、range_max 超え）も max_r 扱いに揃える
        rmin = float(msg.range_min) if msg.range_min > 0.0 else 0.0
        ranges = np.where((ranges >= rmin) & (ranges <= max_r), ranges, max_r)

        self.latest_lidar_data = {
            "ranges": ranges,
            "angles": angles,
        }
        
        if not self.received_scan:
            self.get_logger().info(f"Received LaserScan: beams={n}, max_range={max_r}")
            self.received_scan = True

    def control_loop(self):
        # scan がまだなら何もしない（または停止指令）
        if self.latest_lidar_data is None:
            return
        if not hasattr(self, "controller") or self.controller is None:
            return
        lidar_data = self.latest_lidar_data

        # 緊急停止チェック（最優先 - パス計算より先に判定）
        if self._check_emergency_stop(lidar_data):
            stop_cmd = Twist()
            self._publish_cmd_vel(stop_cmd)
            return

        # プランナーでパス生成して出力確認
        if self.planner is not None:
            try:
                paths, extra = self.planner.generate_local_paths(lidar_data)
                self.get_logger().info(f"Generated {len(paths)} local paths")

                # パスの詳細情報を出力
                for i, path in enumerate(paths[:3]):  # 最初の3つのパスのみ表示
                    self.get_logger().info(f"Path {i}: {len(path)} points")

                # 最適パスを選択してrvizで可視化
                selected_path, selected_idx = self.planner.select_outermost_path(paths)
                self.get_logger().info(f"Selected path {selected_idx} with {len(selected_path)} points")
                self.publish_path_for_visualization(selected_path)

                # パス追従制御
                if len(selected_path) > 0:
                    # controller をローカルに退避（差し替え中に参照が壊れないように）
                    with self._ctrl_lock:
                        controller = self.controller

                    linear_vel, angular_vel = self.controller.compute_control(selected_path)

                    # 制御コマンドを送信
                    cmd = Twist()
                    cmd.linear.x = linear_vel
                    cmd.angular.z = angular_vel
                    self._publish_cmd_vel(cmd)

                    self.get_logger().info(f"Control: linear={linear_vel:.3f}, angular={angular_vel:.3f}")

                    # ゴール到達判定
                    if self.controller.is_goal_reached(selected_path):
                        self.get_logger().info("Goal reached!")
                        self.controller.reset()
                else:
                    # パスがない場合は停止
                    stop_cmd = Twist()
                    self._publish_cmd_vel(stop_cmd)
                        
            except Exception as e:
                self.get_logger().error(f"generate_local_paths failed: {e}")
                import traceback
                self.get_logger().error(f"Traceback: {traceback.format_exc()}")
                
                # エラー時は停止
                stop_cmd = Twist()
                self._publish_cmd_vel(stop_cmd)

        # 動作確認のため直進（パス追従制御時はコメントアウト）
        # cmd = Twist()
        # cmd.linear.x = 0.3  # 前進
        # self.cmd_pub.publish(cmd)
    
    def publish_path_for_visualization(self, path):
        """選択されたパスをrvizで表示するためにpublish"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "laser"  # パスの座標系をlaserフレームに設定
        
        # パスの各ポイントをPathメッセージに変換
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # 無回転
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().debug(f"Published path with {len(path_msg.poses)} poses")
    
    def _setup_input_subscribers(self):
        """Setup input subscribers based on namespace parameters"""
        # Get namespace parameters
        input_sim = self.get_parameter('input_sim').get_parameter_value().bool_value
        input_real = self.get_parameter('input_real').get_parameter_value().bool_value
        sim_ns = self.get_parameter('sim_ns').get_parameter_value().string_value
        real_ns = self.get_parameter('real_ns').get_parameter_value().string_value
        
        # Validation
        if not (input_sim or input_real):
            raise ValueError("At least one of input_sim or input_real must be true")
        
        # For now, create subscriber for the primary input source
        # TODO: Handle multiple input sources (data fusion)
        if input_sim:
            scan_topic = f"/{sim_ns}/scan" if sim_ns else "/scan"
            self.get_logger().info(f"Subscribing to sim LiDAR: {scan_topic}")
        else:
            scan_topic = f"/{real_ns}/scan" if real_ns else "/scan"
            self.get_logger().info(f"Subscribing to real LiDAR: {scan_topic}")
            
        return self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )
    
    def _setup_output_publishers(self):
        """Setup output publishers based on namespace parameters"""
        # Get namespace parameters
        output_sim = self.get_parameter('output_sim').get_parameter_value().bool_value
        output_real = self.get_parameter('output_real').get_parameter_value().bool_value
        sim_ns = self.get_parameter('sim_ns').get_parameter_value().string_value
        real_ns = self.get_parameter('real_ns').get_parameter_value().string_value

        # Get robot type (diff or ackermann)
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value

        # Determine topic suffix based on robot type
        if robot_type == "ackermann":
            topic_suffix = "ackermann_steering_controller/reference_unstamped"
        else:
            topic_suffix = "diff_drive_controller/cmd_vel_unstamped"

        publishers = {}

        if output_sim:
            sim_topic = f"/{sim_ns}/{topic_suffix}"
            publishers['sim'] = self.create_publisher(Twist, sim_topic, 10)
            self.get_logger().info(f"Publishing to sim robot ({robot_type}): {sim_topic}")

        if output_real:
            real_topic = f"/{real_ns}/{topic_suffix}"
            publishers['real'] = self.create_publisher(Twist, real_topic, 10)
            self.get_logger().info(f"Publishing to real robot ({robot_type}): {real_topic}")

        if not publishers:
            self.get_logger().warn("No output publishers configured!")

        return publishers
    
    def _publish_cmd_vel(self, cmd_msg):
        """Publish command to all configured output publishers"""
        for name, publisher in self.cmd_publishers.items():
            publisher.publish(cmd_msg)

    def _check_emergency_stop(self, lidar_data: dict) -> bool:
        """
        前方に障害物があるかチェックする緊急停止判定

        Returns:
            True: 緊急停止が必要（前方コーン内の30%以上が閾値以内）
            False: 通常走行可能
        """
        ranges = lidar_data["ranges"]
        angles = lidar_data["angles"]

        # 前方コーン（±15°）内のレンジを抽出
        # 角度は0〜2πの範囲なので、前方(0°)付近と2π付近の両方をチェック
        cone_rad = np.radians(self.EMERGENCY_STOP_CONE_DEG)
        front_mask = (angles <= cone_rad) | (angles >= 2 * np.pi - cone_rad)

        if not np.any(front_mask):
            return False

        front_ranges = ranges[front_mask]
        total_points = len(front_ranges)

        # 閾値以下の点の割合をチェック（ノイズ除去）
        close_points = np.sum(front_ranges < self.EMERGENCY_STOP_DIST)
        close_ratio = close_points / total_points

        if close_ratio >= self.EMERGENCY_STOP_RATIO:
            min_front_dist = np.min(front_ranges)
            self.get_logger().warn(
                f"EMERGENCY STOP: {close_ratio:.0%} points at <{self.EMERGENCY_STOP_DIST}m "
                f"(min: {min_front_dist:.3f}m)"
            )
            return True

        return False
            
    def _initialize_controller(self):
        """パラメータに基づいてコントローラーを初期化"""
        # コントローラータイプの決定（オーバーライド優先）
        controller_type_override = self.get_parameter('controller_type_override').get_parameter_value().string_value
        if controller_type_override:
            controller_type = controller_type_override
        else:
            controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        
        # 共通パラメータを取得
        common_params = self._get_parameters_as_dict('common')
        
        # コントローラー固有パラメータを取得
        controller_specific_params = self._get_parameters_as_dict(f'controllers.{controller_type}')
        
        # パラメータをマージ
        params = {**common_params, **controller_specific_params}
        
        try:
            self.controller = ControllerFactory.create_controller(controller_type, params)
            self.get_logger().info(f"Initialized {controller_type.upper()} controller")
            self._log_controller_config(controller_type, params)
                
        except ValueError as e:
            self.get_logger().error(f"Controller creation failed: {e}")
            self.get_logger().error(f"Supported controllers: {ControllerFactory.get_supported_controllers()}")
            raise
    
    def _get_parameters_as_dict(self, prefix: str) -> dict:
        """指定されたプレフィックスのパラメータを辞書として取得"""
        param_dict = {}
        try:
            # get_parameters_by_prefix は辞書を返し、keyからprefixが自動で除去される
            params = self.get_parameters_by_prefix(prefix)
            for key, param in params.items():
                param_dict[key] = param.value

        except Exception as e:
            self.get_logger().warn(f"Failed to get parameters with prefix '{prefix}': {e}")

        return param_dict

    def _create_planner_config(self) -> local_path_planner.PathPlannerConfig:
        """プランナー設定を作成"""
        planner_params = self._get_parameters_as_dict('planner')

        direction_bias_deg = planner_params.get('direction_bias_deg', 0.0)

        self.get_logger().info(f"Planner config: direction_bias_deg={direction_bias_deg:.1f}")

        return local_path_planner.PathPlannerConfig(
            DIRECTION_BIAS_DEG=direction_bias_deg
        )
    
    def _log_controller_config(self, controller_type: str, params: dict):
        """コントローラー設定をログ出力"""
        lookahead = params.get('lookahead_distance', 'N/A')
        velocity = params.get('target_velocity', 'N/A')
        max_angular = params.get('max_angular_velocity', 'N/A')
        
        # 数値の場合のみフォーマット
        lookahead_str = f"{lookahead:.2f}" if isinstance(lookahead, (int, float)) else str(lookahead)
        velocity_str = f"{velocity:.2f}" if isinstance(velocity, (int, float)) else str(velocity)
        max_angular_str = f"{max_angular:.2f}" if isinstance(max_angular, (int, float)) else str(max_angular)
        
        self.get_logger().info(
            f"Config: lookahead={lookahead_str}m, "
            f"velocity={velocity_str}m/s, "
            f"max_angular={max_angular_str}rad/s"
        )
        
        if controller_type == 'pd':
            kp = params.get('kp_angular', 'N/A')
            kd = params.get('kd_angular', 'N/A')
            
            kp_str = f"{kp:.2f}" if isinstance(kp, (int, float)) else str(kp)
            kd_str = f"{kd:.2f}" if isinstance(kd, (int, float)) else str(kd)
            
            self.get_logger().info(f"PD gains: KP={kp_str}, KD={kd_str}")
            
    def _on_parameters_updated(self, params: list[Parameter]) -> SetParametersResult:
        """
        パラメータ更新を検知して controller/planner を再初期化する
        """

        # controller / common / controllers.* に関係する変更
        controller_relevant = any(
            p.name == "controller_type"
            or p.name.startswith("common.")
            or p.name.startswith("controllers.")
            for p in params
        )

        # planner.* に関係する変更
        planner_relevant = any(
            p.name.startswith("planner.")
            for p in params
        )

        if not controller_relevant and not planner_relevant:
            return SetParametersResult(successful=True)

        # パラメータが適用された「後」に再初期化したいので遅延実行
        def apply():
            with self._ctrl_lock:
                if controller_relevant:
                    self._initialize_controller()
                    self.get_logger().info("Controller re-initialized due to parameter update.")
                if planner_relevant:
                    planner_config = self._create_planner_config()
                    self.planner = local_path_planner.LocalPathPlanner(config=planner_config)
                    self.get_logger().info("Planner re-initialized due to parameter update.")
            # タイマーを破棄して一度だけ実行
            if hasattr(self, '_param_update_timer') and self._param_update_timer:
                self._param_update_timer.cancel()
                self._param_update_timer = None

        # 既存のタイマーがあればキャンセル
        if hasattr(self, '_param_update_timer') and self._param_update_timer:
            self._param_update_timer.cancel()
        self._param_update_timer = self.create_timer(0.01, apply)
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = LocalNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._publish_cmd_vel(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()