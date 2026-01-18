# スケール調整ガイド

シミュレーション環境と実環境のスケールが異なる場合の調整方法です。

## 概要

基準スケール（1倍）はシミュレーション用の小規模コースです。
実環境に合わせてN倍にスケールする場合、以下のパラメータを調整します。

## 調整が必要なファイルとパラメータ

### 1. ローカルパスプランナー

**ファイル**: `minicar_navigation/minicar_navigation/planner/local_path_planner.py`

```python
@dataclass(frozen=True)
class PathPlannerConfig:
    MAP_RESOLUTION: float = 0.005 * N  # 基準: 0.005 m/pixel
```

| スケール | MAP_RESOLUTION |
|----------|----------------|
| 1倍 | 0.005 |
| 3倍 | 0.015 |
| 5倍 | 0.025 |

---

### 2. コース生成

**ファイル**: `minicar_simulation/scripts/generate_course.py`

#### 2.1 楕円サイズ（コース全体の大きさ）

```python
config = GeneratorConfig(
    ellipse=EllipseParams(
        semi_major=3.0 * N,    # 基準: 3.0m
        semi_minor=1.8 * N,    # 基準: 1.8m
        ...
    ),
```

| スケール | semi_major | semi_minor |
|----------|------------|------------|
| 1倍 | 3.0 | 1.8 |
| 3倍 | 9.0 | 5.4 |
| 5倍 | 15.0 | 9.0 |

#### 2.2 ウェイポイント間隔

```python
    min_waypoint_interval=0.2 * N,  # 基準: 0.2m
    max_waypoint_interval=0.6 * N,  # 基準: 0.6m
```

| スケール | min | max |
|----------|-----|-----|
| 1倍 | 0.2 | 0.6 |
| 3倍 | 0.6 | 1.8 |
| 5倍 | 1.0 | 3.0 |

#### 2.3 オフセット（コースの歪み量）

```python
    min_offset=-0.5 * N,           # 基準: -0.5m
    max_offset=0.5 * N,            # 基準: 0.5m
    shortcut_min_offset=-0.3 * N,  # 基準: -0.3m
    shortcut_max_offset=0.3 * N,   # 基準: 0.3m
```

| スケール | offset | shortcut_offset |
|----------|--------|-----------------|
| 1倍 | ±0.5 | ±0.3 |
| 3倍 | ±1.5 | ±0.9 |
| 5倍 | ±2.5 | ±1.5 |

#### 2.4 壁生成パラメータ

```python
triangles, polys = routes_to_wall_triangles(
    road_width_m=0.6 * N,        # 基準: 0.6m
    wall_thickness_m=0.05 * N,   # 基準: 0.05m
    meters_per_pixel=0.01 * N,   # 基準: 0.01 m/px
    img_wh=(800, 500),           # 固定
)
```

| スケール | road_width | wall_thickness | meters_per_pixel | カバー範囲 |
|----------|------------|----------------|------------------|------------|
| 1倍 | 0.6m | 0.05m | 0.01 | 8m x 5m |
| 3倍 | 1.8m | 0.15m | 0.03 | 24m x 15m |
| 5倍 | 3.0m | 0.25m | 0.05 | 40m x 25m |

#### 2.5 プレビュー画像

```python
save_course_image(routes, image_path,
    img_wh=(800, 500),
    meters_per_pixel=0.01 * N,  # 壁生成と同じ値
    road_width_px=60)           # 固定
```

---

## クイックリファレンス

### 3倍スケールにする場合

```bash
# 1. local_path_planner.py
MAP_RESOLUTION: float = 0.015

# 2. generate_course.py - ellipse
semi_major=9.0
semi_minor=5.4

# 3. generate_course.py - waypoint
min_waypoint_interval=0.6
max_waypoint_interval=1.8

# 4. generate_course.py - offset
min_offset=-1.5
max_offset=1.5
shortcut_min_offset=-0.9
shortcut_max_offset=0.9

# 5. generate_course.py - wall
road_width_m=1.8
wall_thickness_m=0.15
meters_per_pixel=0.03

# 6. generate_course.py - preview
meters_per_pixel=0.03
```

---

---

## 3. 制御パラメータ

**ファイル**: `minicar_navigation/config/controllers.yaml`

### 3.1 共通パラメータ

```yaml
common:
  lookahead_distance: 0.2 * N   # 基準: 0.2m
  target_velocity: 0.3~0.5      # 実機の動作に依存
  max_angular_velocity: 1.5~3.0 # 実機の旋回能力に依存
  goal_tolerance: 0.1 * N       # 基準: 0.1m
```

| スケール | lookahead | goal_tolerance |
|----------|-----------|----------------|
| 1倍 | 0.2m | 0.1m |
| 3倍 | 0.6m | 0.3m |
| 5倍 | 1.0m | 0.5m |

### 3.2 PDコントローラー

```yaml
pd:
  kp_angular: 4.5~8.0          # 実機で旋回が弱い場合は増加
  kd_angular: 0.8~1.2
  curvature_lookahead: 10 * N  # 基準: 10ポイント
  min_velocity: 0.08~0.15      # 実機のデッドゾーンに依存
```

| スケール | curvature_lookahead |
|----------|---------------------|
| 1倍 | 10 |
| 3倍 | 20 |
| 5倍 | 30 |

### 3.3 実機調整のポイント

- **旋回が弱い**: `kp_angular` を上げる、`max_angular_velocity` を上げる
- **動き出さない**: `min_velocity` を上げる、`target_velocity` を上げる
- **振動する**: `kd_angular` を上げる、`kp_angular` を下げる

---

## ビルドと確認

```bash
cd ~/ros2_ws
colcon build --packages-select minicar_navigation minicar_simulation
source install/setup.bash

# コース生成テスト
python3 src/minicar_simulation/scripts/generate_course.py --seed 0
```

---

*最終更新: 2026-01-18*
