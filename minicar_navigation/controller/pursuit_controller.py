#!/usr/bin/env python3
"""
Pure Pursuit Controller for Local Path Following

機能:
- ローカルパスの追従制御
- Look-ahead距離に基づく目標点選択  
- 差動駆動ロボット用制御コマンド生成
"""

import numpy as np
import math
from typing import Tuple, Optional
from dataclasses import dataclass

from .base_controller import BaseController

@dataclass(frozen=True)
class PursuitControllerConfig:
    """Pursuit Controllerの設定"""
    # Pure Pursuit パラメータ
    LOOKAHEAD_DISTANCE: float = 0.3  # Look-ahead距離 [m]
    
    # 速度制御パラメータ
    TARGET_VELOCITY: float = 0.3     # 目標速度 [m/s]
    MAX_ANGULAR_VELOCITY: float = 1.0  # 最大角速度 [rad/s]
    
    # ゴール判定
    GOAL_TOLERANCE: float = 0.1      # ゴール到達判定距離 [m]


class PursuitController(BaseController):
    """Pure Pursuit アルゴリズムによるパス追従コントローラー"""
    
    def __init__(self, config: PursuitControllerConfig = None):
        """
        Args:
            config: コントローラー設定
        """
        self.config = config or PursuitControllerConfig()
        
    def compute_control(self, path: np.ndarray, robot_state: Tuple[float, float, float] = None) -> Tuple[float, float]:
        """
        パス追従制御コマンドを計算
        
        Args:
            path: 追従するパス, shape=(N, 2), ロボット座標系, メートル単位
            robot_state: 現在のロボット状態 (x, y, yaw), Noneなら原点 (0, 0, 0)
            
        Returns:
            (linear_velocity, angular_velocity): 制御コマンド [m/s, rad/s]
        """
        if len(path) == 0:
            return 0.0, 0.0
            
        # 現在位置（デフォルトは原点、前方向き）
        if robot_state is None:
            current_x, current_y, current_yaw = 0.0, 0.0, 0.0
        else:
            current_x, current_y, current_yaw = robot_state
        
        # Look-ahead点を見つける
        target_point = self._find_lookahead_point(path, current_x, current_y)
        
        if target_point is None:
            return 0.0, 0.0
        
        # Pure Pursuitアルゴリズムで制御コマンドを計算
        linear_vel, angular_vel = self._compute_pure_pursuit(
            target_point, current_x, current_y, current_yaw
        )
        
        return linear_vel, angular_vel
        
    def is_goal_reached(self, path: np.ndarray, robot_state: Tuple[float, float, float] = None) -> bool:
        """
        ゴール到達判定
        
        Args:
            path: パス
            robot_state: 現在のロボット状態
            
        Returns:
            ゴールに到達したかどうか
        """
        if len(path) == 0:
            return True
            
        # 現在位置
        if robot_state is None:
            current_x, current_y = 0.0, 0.0
        else:
            current_x, current_y = robot_state[0], robot_state[1]
        
        # パスの最後の点との距離
        goal_x, goal_y = path[-1]
        distance_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        
        return distance_to_goal < self.config.GOAL_TOLERANCE
    
    def reset(self):
        """コントローラーの状態をリセット"""
        pass  # Pure Pursuitは状態を持たないため実装不要
    
    def _find_lookahead_point(self, path: np.ndarray, current_x: float, current_y: float) -> Optional[Tuple[float, float]]:
        """
        Look-ahead距離に基づいて目標点を選択
        
        Args:
            path: パス点群
            current_x, current_y: 現在位置
            
        Returns:
            目標点 (x, y) or None
        """
        lookahead_dist = self.config.LOOKAHEAD_DISTANCE
        
        # 現在位置から各パス点までの距離を計算
        distances = np.sqrt(
            (path[:, 0] - current_x) ** 2 + (path[:, 1] - current_y) ** 2
        )
        
        # Look-ahead距離以上の点を探す
        valid_indices = np.where(distances >= lookahead_dist)[0]
        
        if len(valid_indices) == 0:
            # Look-ahead距離内に点がない場合は最後の点を選択
            return tuple(path[-1])
        
        # 最初に見つかったLook-ahead距離以上の点を選択
        target_idx = valid_indices[0]
        return tuple(path[target_idx])
    
    def _compute_pure_pursuit(self, target_point: Tuple[float, float], 
                            current_x: float, current_y: float, current_yaw: float) -> Tuple[float, float]:
        """
        Pure Pursuitアルゴリズムによる制御計算
        
        Args:
            target_point: 目標点 (x, y)
            current_x, current_y, current_yaw: 現在位置・姿勢
            
        Returns:
            (linear_velocity, angular_velocity)
        """
        target_x, target_y = target_point
        
        # 目標点までの距離
        dx = target_x - current_x
        dy = target_y - current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # ゴール近辺では停止
        if distance_to_target < self.config.GOAL_TOLERANCE:
            return 0.0, 0.0
        
        # 目標点への角度（グローバル座標系）
        target_angle = math.atan2(dy, dx)
        
        # ロボットの向きとの角度差
        angle_diff = target_angle - current_yaw
        
        # 角度を[-π, π]に正規化
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 線形速度の計算（角度差に応じて速度調整）
        speed_factor = max(0.3, 1.0 - abs(angle_diff) / math.pi)
        linear_vel = self.config.TARGET_VELOCITY * speed_factor
        
        # 角速度の計算（P制御）
        angular_gain = 2.0
        angular_vel = angular_gain * angle_diff
        
        # 角速度制限
        angular_vel = max(-self.config.MAX_ANGULAR_VELOCITY,
                         min(self.config.MAX_ANGULAR_VELOCITY, angular_vel))
        
        return linear_vel, angular_vel