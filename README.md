# minicar_navigation

ROS2用のミニカーナビゲーションパッケージです。LiDARデータを使用した局所経路計画と経路追従制御機能を提供します。

## 概要

このパッケージは以下の機能を提供します：

- LiDARベースの局所経路計画
- 複数の経路追従コントローラー（Pure Pursuit、PD制御）
- 動的パラメータ調整システム
- テレオペレーション機能（キーボード・ジョイスティック）
- シミュレーション・実機対応

## アーキテクチャ

### 主要コンポーネント

- **local_nav_node**: メインナビゲーションノード
- **param_server_node**: 動的パラメータ管理ノード
- **LocalPathPlanner**: 局所経路計画器
- **ControllerFactory**: コントローラー管理システム

### コントローラー

- **PursuitController**: Pure Pursuit制御
- **PDPursuitController**: PD制御ベースのPure Pursuit

## 使用方法

### 基本的な起動

```bash
# ナビゲーションシステム起動
ros2 launch minicar_navigation local_nav.launch.py

# テレオペレーション起動
ros2 launch minicar_navigation teleop.launch.py
```

### launchファイル

#### `local_nav.launch.py`

ローカルナビゲーションシステムを起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `params_file` | `~/controllers.yaml` | コントローラーパラメータファイル |
| `sim_ns` | `sim_robot` | シミュレーション用名前空間 |
| `real_ns` | `real_robot` | 実機用名前空間 |
| `input_sim` | `true` | シミュレーションからの入力を使用 |
| `input_real` | `false` | 実機からの入力を使用 |
| `output_sim` | `true` | シミュレーションへの出力を使用 |
| `output_real` | `false` | 実機への出力を使用 |
| `use_sim_time` | `false` | シミュレーション時間の使用 |

**使用例：**

```bash
# 実機モードで起動
ros2 launch minicar_navigation local_nav.launch.py input_real:=true output_real:=true input_sim:=false output_sim:=false

# カスタムパラメータファイルを使用
ros2 launch minicar_navigation local_nav.launch.py params_file:=/path/to/custom_params.yaml

# シミュレーション時間を使用
ros2 launch minicar_navigation local_nav.launch.py use_sim_time:=true
```

#### `teleop.launch.py`

テレオペレーション機能を起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 選択肢 | 説明 |
|-----------|-------------|--------|------|
| `teleop` | `keyboard` | `keyboard`, `joystick` | テレオペレーション方法 |
| `sim_ns` | `sim_robot` | - | シミュレーション用名前空間 |
| `real_ns` | `real_robot` | - | 実機用名前空間 |
| `output_sim` | `true` | - | シミュレーションへの出力 |
| `output_real` | `false` | - | 実機への出力 |

**使用例：**

```bash
# キーボード操作でシミュレーションロボット制御
ros2 launch minicar_navigation teleop.launch.py

# ジョイスティック操作で実機制御
ros2 launch minicar_navigation teleop.launch.py teleop:=joystick output_real:=true output_sim:=false

# 両方同時制御
ros2 launch minicar_navigation teleop.launch.py output_real:=true output_sim:=true
```

## パラメータ設定

### controllers.yaml

コントローラーの動作パラメータを設定します：

```yaml
local_nav_node:
  ros__parameters:
    controller_type: "pd"  # "p" または "pd"
    
    common:
      lookahead_distance: 0.3    # 先読み距離 (m)
      target_velocity: 0.3       # 目標速度 (m/s)
      max_angular_velocity: 1.0  # 最大角速度 (rad/s)
      goal_tolerance: 0.1        # ゴール許容誤差 (m)
    
    controllers:
      pd:
        kp_angular: 2.0  # 比例ゲイン
        kd_angular: 0.5  # 微分ゲイン
```

### 動的パラメータ変更

実行中にパラメータを変更できます：

```bash
# パラメータ確認
ros2 param list /local_nav_node

# パラメータ変更
ros2 param set /local_nav_node controller_type pd
ros2 param set /local_nav_node common.target_velocity 0.5
```

## トピック

### 購読トピック

- `/{sim_ns}/scan`: LiDARデータ (sensor_msgs/LaserScan)
- `/{real_ns}/scan`: LiDARデータ (sensor_msgs/LaserScan)

### パブリッシュトピック

- `/{sim_ns}/diff_drive_controller/cmd_vel_unstamped`: 速度指令 (geometry_msgs/Twist)
- `/{real_ns}/diff_drive_controller/cmd_vel_unstamped`: 速度指令 (geometry_msgs/Twist)
- `/local_paths`: 計画経路の可視化 (nav_msgs/Path)

## デバッグ・テストツール

### debug_recorder

センサデータと推論結果を記録します。

```bash
# シミュレーション用
ros2 run minicar_navigation debug_recorder

# 実機用（トピックリマップ）
python3 ~/ros2_ws/src/minicar_navigation/minicar_navigation/debug_recorder.py \
  --ros-args \
  -r /sim_robot/scan:=/real_robot/scan \
  -r /sim_robot/diff_drive_controller/odom:=/real_robot/diff_drive_controller/odom
```

出力先: `/tmp/debug_data/session_YYYYMMDD_HHMMSS/all_frames.json`

### trajectory_logger

ロボットの軌跡を記録します（batch_test.shから使用）。

```bash
ros2 run minicar_navigation trajectory_logger
```

### plot_debug_frame.py

記録したデータを可視化します。

```bash
# 特定フレームを表示
python3 ~/ros2_ws/src/minicar_navigation/scripts/plot_debug_frame.py <json_file> <frame_idx>

# 画像として保存
python3 ~/ros2_ws/src/minicar_navigation/scripts/plot_debug_frame.py data.json 50 -o frame50.png
```

### batch_test.sh

複数シードでバッチテストを実行します。

```bash
# シード0-10でテスト
./scripts/batch_test.sh 0 10
```

出力先: `/tmp/batch_test/`

## スケール調整

シミュレーションと実環境のスケールが異なる場合は、[docs/scale_adjustment.md](docs/scale_adjustment.md) を参照してください。

## 依存関係

- ROS2 Humble以上
- sensor_msgs
- geometry_msgs
- nav_msgs
- teleop_twist_keyboard
- teleop_twist_joy
- joy

## トラブルシューティング

### LiDARデータが受信されない場合

- センサーノードが起動しているか確認
- トピック名とnamespaceが正しいか確認

### コントローラーが動作しない場合

- パラメータファイルのパスが正しいか確認
- controller_typeが有効な値（"p"、"pd"）か確認

### テレオペレーションが動作しない場合

- ジョイスティックモードの場合、`/dev/input/js0`が存在するか確認
- 出力先namespace設定が正しいか確認