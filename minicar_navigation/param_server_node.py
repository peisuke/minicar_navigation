#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


class SimpleParamServer(Node):
    def __init__(self):
        super().__init__('param_server_node')
        
        # 設定
        self.declare_parameter('config_path', '/tmp/nav_config.yaml')
        self.declare_parameter('target_node', '/local_nav_node')
        
        self.config_path = self.get_parameter('config_path').value
        self.target_node = self.get_parameter('target_node').value
        
        # パラメータ変更監視
        self.param_event_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.on_parameter_event,
            10
        )
        
        # 対象ノードのサービスクライアント
        self.get_params_cli = self.create_client(
            GetParameters, 
            f'{self.target_node}/get_parameters'
        )
        self.set_params_cli = self.create_client(
            SetParameters,
            f'{self.target_node}/set_parameters'
        )
        
        self.get_logger().info(f"Monitoring parameters for {self.target_node}")

    def on_parameter_event(self, msg: ParameterEvent):
        """パラメータ変更イベントを監視"""
        # 対象ノードの変更のみ処理
        if msg.node != self.target_node:
            return
            
        for changed_param in msg.changed_parameters:
            param_name = changed_param.name
            param_value = self._extract_value(changed_param.value)
            
            self.get_logger().info(f"Parameter changed: {param_name} = {param_value}")
            
            # YAMLファイルに保存
            self.save_to_yaml(param_name, param_value)

    def _extract_value(self, param_value: ParameterValue):
        """ParameterValueから実際の値を取得"""
        if param_value.type == Parameter.Type.BOOL.value:
            return param_value.bool_value
        elif param_value.type == Parameter.Type.INTEGER.value:
            return param_value.integer_value
        elif param_value.type == Parameter.Type.DOUBLE.value:
            return param_value.double_value
        elif param_value.type == Parameter.Type.STRING.value:
            return param_value.string_value
        else:
            return None

    def save_to_yaml(self, param_name: str, param_value):
        """パラメータをYAMLファイルに保存"""
        try:
            # 既存YAML読み込み
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}
                
            # ノード構造を確保
            node_key = self.target_node.lstrip('/')
            if node_key not in data:
                data[node_key] = {'ros__parameters': {}}
            if 'ros__parameters' not in data[node_key]:
                data[node_key]['ros__parameters'] = {}
                
            # 階層パラメータ処理 (common.target_velocity -> common: {target_velocity: ...})
            params = data[node_key]['ros__parameters']
            parts = param_name.split('.')
            
            current = params
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
            current[parts[-1]] = param_value
            
            # YAML保存
            with open(self.config_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
                
            self.get_logger().info(f"Saved {param_name}={param_value} to {self.config_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML: {e}")

    def update_parameter(self, param_name: str, value):
        """プログラムから対象ノードのパラメータを変更"""
        if not self.set_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Set parameters service not available")
            return False
            
        # Parameter作成
        param = Parameter(param_name, value=value).to_parameter_msg()
        
        # サービス呼び出し
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.set_params_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().results[0].successful:
            self.get_logger().info(f"Successfully updated {param_name}")
            return True
        else:
            self.get_logger().error(f"Failed to update {param_name}")
            return False


def main():
    rclpy.init()
    node = SimpleParamServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
