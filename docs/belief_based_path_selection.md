# Belief-Based Path Selection

## 概要

時間的一貫性を持つパス選択アルゴリズム。フレーム間でのパス方向の急激な変化（オシレーション）を防ぎ、安定した走行を実現する。

## アルゴリズム

### 1. パス生成パイプライン

```
LiDARデータ
    ↓
距離場生成 (Distance Field)
    ↓
ピーク検出 (Peak Detection)
    ↓  ← MIN_PEAK_DIST_VALUE でフィルタ
エッジ検証 (Edge Validation)
    ↓  ← MAX_GRADIENT_DROP でフィルタ
パス構築 (Path Construction)
    ↓
パス選択 (Belief-based Selection)
    ↓
選択されたパス
```

### 2. スコアリング

各パス候補に対してスコアを計算:

```
score = confidence + BELIEF_CONSISTENCY_WEIGHT × combined
```

- **confidence**: パスの信頼度（長いパスほど高い）
  ```
  confidence = tanh(path_length / BELIEF_CONFIDENCE_LENGTH_SCALE)
  ```

- **combined**: 信念との一致度とバイアスの平均
  ```
  consistency = dot(belief_direction, path_direction)  # -1 ~ +1
  bias_preference = dot(bias_direction, path_direction)
  combined = (consistency + bias_preference) / 2
  ```

### 3. 信念更新

選択されたパスの方向で信念をEMA更新:

```
effective_alpha = BELIEF_EMA_ALPHA × confidence
belief = normalize((1 - effective_alpha) × belief + effective_alpha × path_direction)
```

高信頼度のパスほど大きく信念を動かす。

## パラメータ

### PathPlannerConfig

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `BELIEF_EMA_ALPHA` | 0.3 | 信念更新の指数移動平均係数 (0=更新なし, 1=即時置換) |
| `BELIEF_CONFIDENCE_LENGTH_SCALE` | 2.0 | 信頼度算出のパス長スケール(m) |
| `BELIEF_MIN_CONFIDENCE` | 0.1 | 最低信頼度閾値（未満のパスは選択候補外）|
| `BELIEF_CONSISTENCY_WEIGHT` | 2.0 | スコア中の一貫性重み（加算式）|
| `DIRECTION_BIAS_DEG` | 0.0 | 方向バイアス（度）正=左寄り、負=右寄り |
| `MAX_GRADIENT_DROP` | 24.0 | エッジの距離場勾配の最大許容下降量(px) |
| `MIN_PEAK_DIST_VALUE` | 100.0 | ピーク検出時の最低距離場値（0-255）|

### パラメータ調整ガイド

- **オシレーションが多い場合**: `BELIEF_CONSISTENCY_WEIGHT` を上げる（例: 2.0 → 3.0）
- **方向変化が遅すぎる場合**: `BELIEF_EMA_ALPHA` を上げる（例: 0.3 → 0.5）
- **壁方向のパスが選択される場合**: `MIN_PEAK_DIST_VALUE` を上げる（例: 100 → 120）
- **前方パスが生成されない場合**: `MAX_GRADIENT_DROP` を上げる（例: 24 → 32）

## フィルタリング

### 1. ピーク検出フィルタ (MIN_PEAK_DIST_VALUE)

距離場の値が低いピーク（壁に近い）を除外:

```python
if dist_value >= MIN_PEAK_DIST_VALUE:
    peaks.append(peak)
```

- 壁方向（scan_range < 2m）のピークは通常 dist_value < 100
- 走行可能方向のピークは通常 dist_value > 150

### 2. エッジ勾配フィルタ (MAX_GRADIENT_DROP)

壁に向かうエッジを除外:

```python
gradient = dst_dist_value - src_dist_value
valid = gradient >= -MAX_GRADIENT_DROP
```

- 閾値が厳しすぎると前方パスが不安定に棄却される
- 閾値が緩すぎると後方パスも生成される

## テスト結果

60シード × 30秒のテストで検証:

| 指標 | 改善前 | 改善後 |
|------|--------|--------|
| 平均ジャンプ数 | 3-5回 | 0-2回 |
| 平均安定性スコア | 60-80 | 85-99 |
| パスカバレッジ | 95% | 99% |

## 関連ファイル

- `minicar_navigation/planner/local_path_planner.py`: メイン実装
- `scripts/analyze_path_oscillation.py`: オシレーション分析ツール
- `scripts/analyze_path_generation.py`: パス生成分析ツール
- `scripts/analyze_path_edges.py`: エッジ検証分析ツール
