#!/bin/bash
#
# Batch Navigation Test Script
# 様々なシード値でナビゲーションテストを実行し、軌跡を記録する
#

# 設定
OUTPUT_DIR="/tmp/batch_test"
DURATION=60  # 各テストの時間（秒）
SEEDS=(0 1 2 3 4 5 6 7 8 9)  # テストするシード値

# コマンドライン引数でシード範囲を指定可能
if [ "$1" != "" ]; then
    START_SEED=$1
    END_SEED=${2:-$1}
    SEEDS=()
    for ((i=START_SEED; i<=END_SEED; i++)); do
        SEEDS+=($i)
    done
fi

# 出力ディレクトリ作成
mkdir -p "$OUTPUT_DIR"

echo "========================================"
echo "Batch Navigation Test"
echo "========================================"
echo "Output directory: $OUTPUT_DIR"
echo "Duration per test: ${DURATION}s"
echo "Seeds to test: ${SEEDS[*]}"
echo "========================================"

# 結果サマリー用の配列
declare -A RESULTS

# クリーンアップ関数
cleanup() {
    echo "Cleaning up..."
    # ROS2ノードを停止
    pkill -f "ros2.*minicar" 2>/dev/null
    pkill -f "gzserver" 2>/dev/null
    pkill -f "gzclient" 2>/dev/null
    pkill -f "trajectory_logger" 2>/dev/null
    pkill -f "local_nav" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "spawn_entity" 2>/dev/null
    pkill -f "param_server_node" 2>/dev/null
    sleep 2
}

# 中断時のハンドラ（クリーンアップして終了）
interrupt_handler() {
    echo ""
    echo "Interrupted! Cleaning up..."
    cleanup
    exit 130
}

# Ctrl+C (SIGINT), SIGTERM で中断ハンドラ、EXIT でクリーンアップ
trap interrupt_handler INT TERM
trap cleanup EXIT

# 各シードでテスト実行
for seed in "${SEEDS[@]}"; do
    echo ""
    echo "========================================"
    echo "Testing seed: $seed"
    echo "========================================"

    # 前回のプロセスをクリーンアップ
    cleanup

    # シミュレーション起動
    echo "Starting simulation (seed=$seed)..."
    ros2 launch minicar_simulation road_env_minicar.launch.py seed:=$seed gui:=false &
    SIM_PID=$!

    # シミュレーションの起動を待つ
    echo "Waiting for simulation to initialize..."
    sleep 10

    # ナビゲーション起動
    echo "Starting navigation..."
    ros2 launch minicar_navigation local_nav.launch.py &
    NAV_PID=$!

    # ナビゲーションの起動を待つ
    sleep 3

    # 軌跡ロガー起動
    echo "Starting trajectory logger..."
    ros2 run minicar_navigation trajectory_logger \
        --ros-args \
        -p output_dir:="$OUTPUT_DIR" \
        -p duration:=$DURATION.0 \
        -p seed:=$seed &
    LOGGER_PID=$!

    # テスト実行を待つ（軌跡ロガーが終了するまで）
    echo "Running test for ${DURATION}s..."
    wait $LOGGER_PID 2>/dev/null

    # 結果確認
    RESULT_FILE="$OUTPUT_DIR/seed_$seed/trajectory.json"
    if [ -f "$RESULT_FILE" ]; then
        # JSONから移動距離を抽出
        DISTANCE=$(python3 -c "import json; d=json.load(open('$RESULT_FILE')); print(f\"{d['total_distance']:.2f}\")" 2>/dev/null || echo "N/A")
        POINTS=$(python3 -c "import json; d=json.load(open('$RESULT_FILE')); print(d['num_points'])" 2>/dev/null || echo "N/A")
        echo "Seed $seed: distance=${DISTANCE}m, points=${POINTS}"
        RESULTS[$seed]="OK: ${DISTANCE}m"
    else
        echo "Seed $seed: FAILED (no trajectory file)"
        RESULTS[$seed]="FAILED"
    fi

    # プロセス停止
    kill $NAV_PID 2>/dev/null
    kill $SIM_PID 2>/dev/null

    echo "Test for seed $seed completed."
done

# サマリー表示
echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
for seed in "${SEEDS[@]}"; do
    echo "Seed $seed: ${RESULTS[$seed]}"
done

# サマリーをファイルに保存
SUMMARY_FILE="$OUTPUT_DIR/summary.txt"
{
    echo "Batch Test Summary"
    echo "Date: $(date)"
    echo "Duration: ${DURATION}s per test"
    echo ""
    for seed in "${SEEDS[@]}"; do
        echo "Seed $seed: ${RESULTS[$seed]}"
    done
} > "$SUMMARY_FILE"

echo ""
echo "Results saved to: $OUTPUT_DIR"
echo "Summary: $SUMMARY_FILE"
