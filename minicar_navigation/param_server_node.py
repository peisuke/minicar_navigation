#!/usr/bin/env python3
from __future__ import annotations

import os
import yaml
from typing import Any, Dict, Tuple, List

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import DescribeParameters, SetParametersAtomically
from rcl_interfaces.msg import ParameterType, Parameter as ParameterMsg, ParameterValue


def flatten_dict(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """{"a":{"b":1}} -> {"a.b":1}"""
    out: Dict[str, Any] = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(flatten_dict(v, key))
        else:
            out[key] = v
    return out


def get_ros_parameters_block(yaml_obj: Dict[str, Any], node_name: str) -> Dict[str, Any]:
    """
    YAML形式:
      local_nav_node:
        ros__parameters:
          common:
            target_velocity: 0.3
    を想定して ros__parameters 以下の dict を返す
    """
    if node_name not in yaml_obj:
        raise KeyError(f"Node key '{node_name}' not found in YAML.")
    node_block = yaml_obj[node_name]
    if "ros__parameters" not in node_block:
        raise KeyError(f"'{node_name}.ros__parameters' not found in YAML.")
    params_block = node_block["ros__parameters"]
    if not isinstance(params_block, dict):
        raise TypeError("ros__parameters must be a dict.")
    return params_block


def python_value_to_parameter_value(value: Any, ptype: int) -> ParameterValue:
    pv = ParameterValue()
    pv.type = ptype

    # scalar
    if ptype == ParameterType.PARAMETER_BOOL:
        pv.bool_value = bool(value)
    elif ptype == ParameterType.PARAMETER_INTEGER:
        pv.integer_value = int(value)
    elif ptype == ParameterType.PARAMETER_DOUBLE:
        pv.double_value = float(value)
    elif ptype == ParameterType.PARAMETER_STRING:
        pv.string_value = str(value)

    # arrays
    elif ptype == ParameterType.PARAMETER_BOOL_ARRAY:
        pv.bool_array_value = [bool(x) for x in value]
    elif ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
        pv.integer_array_value = [int(x) for x in value]
    elif ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
        pv.double_array_value = [float(x) for x in value]
    elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
        pv.string_array_value = [str(x) for x in value]
    else:
        raise ValueError(f"Unsupported parameter type: {ptype}")
    return pv


class ParamServerNode(Node):
    """
    - config_path の YAML を正本として読み込む
    - YAML内の local_nav_node.ros__parameters を flatten して "a.b.c" にする
    - /local_nav_node/describe_parameters で型を問い合わせ
    - 型に従って /local_nav_node/set_parameters_atomically で注入
    """

    def __init__(self) -> None:
        super().__init__("param_server_node")

        # ロジック非依存な運用パラメータだけ
        self.declare_parameter("config_path", os.path.expanduser("~/.config/my_robot/controllers.yaml"))
        self.declare_parameter("target_node", "/local_nav_node")
        self.declare_parameter("target_node_key_in_yaml", "local_nav_node")  # YAMLのキー名

        self.config_path = self.get_parameter("config_path").value
        self.target_node = self.get_parameter("target_node").value
        self.yaml_node_key = self.get_parameter("target_node_key_in_yaml").value

        self._describe_cli = self.create_client(DescribeParameters, f"{self.target_node}/describe_parameters")
        self._set_cli = self.create_client(SetParametersAtomically, f"{self.target_node}/set_parameters_atomically")

        # 起動時に1回だけ適用（まずここから）
        self.create_timer(0.1, self._one_shot_apply)

        self._applied_once = False
        self.get_logger().info(f"param_server_node started. target_node={self.target_node}, config_path={self.config_path}")

    def _one_shot_apply(self) -> None:
        if self._applied_once:
            return
        self._applied_once = True

        ok, msg = self.apply_yaml_to_target()
        if ok:
            self.get_logger().info(f"Applied YAML to {self.target_node}: {msg}")
        else:
            self.get_logger().error(f"Failed to apply YAML: {msg}")

    def apply_yaml_to_target(self) -> Tuple[bool, str]:
        # 1) YAML読み込み
        if not os.path.exists(self.config_path):
            return False, f"Config not found: {self.config_path}"

        try:
            with open(self.config_path, "r") as f:
                yobj = yaml.safe_load(f) or {}
            params_block = get_ros_parameters_block(yobj, self.yaml_node_key)
            flat = flatten_dict(params_block)
        except Exception as e:
            return False, f"YAML parse error: {e}"

        # 2) navnodeが生きてるか
        if not self._describe_cli.wait_for_service(timeout_sec=1.0):
            return False, "describe_parameters not available (is /local_nav_node running?)"
        if not self._set_cli.wait_for_service(timeout_sec=1.0):
            return False, "set_parameters_atomically not available (is /local_nav_node running?)"

        # 3) 型問い合わせ
        names = list(flat.keys())
        req = DescribeParameters.Request()
        req.names = names

        future = self._describe_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return False, "describe_parameters call failed"

        descs = future.result().descriptors
        if len(descs) != len(names):
            # 名前に対応するdescriptorが返らない＝未declareの可能性
            return False, "descriptor count mismatch (some params may be undeclared on navnode)"

        # 4) ParameterMsgを作る（型に従う）
        param_msgs: List[ParameterMsg] = []
        for name, desc in zip(names, descs):
            # desc.type は ParameterType enum
            ptype = int(desc.type)

            # 未宣言/動的不可に対する安全策（type=NOT_SET等）
            if ptype == ParameterType.PARAMETER_NOT_SET:
                return False, f"Param '{name}' is NOT_SET (likely undeclared on navnode)."

            value = flat[name]
            try:
                pm = ParameterMsg()
                pm.name = name
                pm.value = python_value_to_parameter_value(value, ptype)
                param_msgs.append(pm)
            except Exception as e:
                return False, f"Type convert failed for '{name}': {e}"

        # 5) Atomically set
        sreq = SetParametersAtomically.Request()
        sreq.parameters = param_msgs
        sfut = self._set_cli.call_async(sreq)
        rclpy.spin_until_future_complete(self, sfut, timeout_sec=2.0)
        if sfut.result() is None:
            return False, "set_parameters_atomically call failed"

        result = sfut.result().result
        if not result.successful:
            return False, f"navnode rejected params: {result.reason}"

        return True, "ok"


def main() -> None:
    rclpy.init()
    node = ParamServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
