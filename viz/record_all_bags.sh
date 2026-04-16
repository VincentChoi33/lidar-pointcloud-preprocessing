#!/bin/bash
set -euo pipefail

# Record processed output while replaying a directory of rosbags.
# Usage:
#   TOPIC=/robot/livox_front/points_processed BAGS_DIR=/ws/bags OUT_DIR=/ws ./viz/record_all_bags.sh

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

TOPIC="${TOPIC:-/robot/livox_front/points_processed}"
BAGS_DIR="${BAGS_DIR:-/ws/bags}"
OUT_DIR="${OUT_DIR:-/ws/processed}"
mkdir -p "$OUT_DIR"

shopt -s nullglob
bags=("$BAGS_DIR"/*.bag)

if [ ${#bags[@]} -eq 0 ]; then
    echo "No bag files found in $BAGS_DIR"
    exit 1
fi

for bag in "${bags[@]}"; do
    name=$(basename "$bag" .bag)
    echo "=== Processing $name ==="

    rosbag record -O "$OUT_DIR/${name}_processed" "$TOPIC" &
    record_pid=$!

    sleep 2
    rosbag play "$bag" --clock 2>/dev/null
    sleep 2

    kill "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true

    echo "=== Done: $name ==="
done

echo "All bags processed."
