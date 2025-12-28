#!/usr/bin/env python3
"""
Controller Factory for creating controller instances with configurations

各コントローラーの生成を一元管理し、パラメータから適切なコントローラーを作成
"""

from typing import Dict, Any
from .base_controller import BaseController
from .pursuit_controller import PursuitController, PursuitControllerConfig
from .pd_pursuit_controller import PDPursuitController, PDPursuitControllerConfig


class ControllerFactory:
    """コントローラーファクトリークラス"""
    
    @staticmethod
    def create_controller(controller_type: str, params: Dict[str, Any]) -> BaseController:
        """
        パラメータに基づいてコントローラーを作成
        
        Args:
            controller_type: コントローラータイプ ('p', 'pd', 'mpc', etc.)
            params: パラメータ辞書
            
        Returns:
            BaseController: 作成されたコントローラーインスタンス
            
        Raises:
            ValueError: 未対応のコントローラータイプが指定された場合
        """
        if controller_type == 'p':
            return ControllerFactory._create_p_controller(params)
        elif controller_type == 'pd':
            return ControllerFactory._create_pd_controller(params)
        else:
            raise ValueError(f"Unsupported controller type: {controller_type}")
    
    @staticmethod
    def _create_p_controller(params: Dict[str, Any]) -> PursuitController:
        """P制御コントローラーを作成"""
        config = PursuitControllerConfig(
            LOOKAHEAD_DISTANCE=params.get('lookahead_distance', 0.3),
            TARGET_VELOCITY=params.get('target_velocity', 0.3),
            MAX_ANGULAR_VELOCITY=params.get('max_angular_velocity', 1.0),
            GOAL_TOLERANCE=params.get('goal_tolerance', 0.1)
        )
        return PursuitController(config)
    
    @staticmethod
    def _create_pd_controller(params: Dict[str, Any]) -> PDPursuitController:
        """PD制御コントローラーを作成"""
        config = PDPursuitControllerConfig(
            LOOKAHEAD_DISTANCE=params.get('lookahead_distance', 0.3),
            TARGET_VELOCITY=params.get('target_velocity', 0.3),
            MAX_ANGULAR_VELOCITY=params.get('max_angular_velocity', 1.0),
            KP_ANGULAR=params.get('kp_angular', 2.0),
            KD_ANGULAR=params.get('kd_angular', 0.5),
            GOAL_TOLERANCE=params.get('goal_tolerance', 0.1)
        )
        return PDPursuitController(config)
    
    @staticmethod
    def get_supported_controllers() -> list:
        """サポートされているコントローラータイプのリストを返す"""
        return ['p', 'pd']