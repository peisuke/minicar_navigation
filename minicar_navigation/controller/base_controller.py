#!/usr/bin/env python3
"""
Base Controller Interface for Path Following

各種コントローラー（Pure Pursuit, MPC等）の共通インターフェースを定義
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple, Optional


class BaseController(ABC):
    """パス追従コントローラーのベースクラス"""
    
    @abstractmethod
    def compute_control(self, path: np.ndarray, robot_state: Tuple[float, float, float] = None) -> Tuple[float, float]:
        """
        パス追従制御コマンドを計算
        
        Args:
            path: 追従するパス, shape=(N, 2), ロボット座標系, メートル単位
                  [[x0, y0], [x1, y1], ..., [xN, yN]]
            robot_state: 現在のロボット状態 (x, y, yaw), Noneなら原点 (0, 0, 0)
            
        Returns:
            (linear_velocity, angular_velocity): 制御コマンド [m/s, rad/s]
        """
        pass
        
    @abstractmethod
    def is_goal_reached(self, path: np.ndarray, robot_state: Tuple[float, float, float] = None) -> bool:
        """
        ゴール到達判定
        
        Args:
            path: パス
            robot_state: 現在のロボット状態
            
        Returns:
            ゴールに到達したかどうか
        """
        pass
    
    @abstractmethod
    def reset(self):
        """
        コントローラーの内部状態をリセット
        新しいパスを開始する際に呼び出される
        """
        pass
        
    def get_controller_info(self) -> dict:
        """
        コントローラーの情報を取得（オプション）
        
        Returns:
            コントローラーの状態や設定情報
        """
        return {
            "controller_type": self.__class__.__name__,
            "config": getattr(self, 'config', None)
        }