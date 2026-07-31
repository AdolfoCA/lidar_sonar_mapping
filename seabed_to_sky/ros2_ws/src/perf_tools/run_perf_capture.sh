#!/usr/bin/env bash
# =============================================================================
# run_perf_capture.sh — launch the three perf loggers with a shared run id,
# dump reproducibility metadata, wait for the operator to start the bag replay,
# and shut everything down cleanly on Ctrl-C.
#
# Usage:
#   ./run_perf_capture.sh --bag /path/to/rosbag_dir [--run-id ID] [--config FILE]
#                         [--out-root DIR] [--interval SEC]
#
# Typical session (two terminals):
#   T1:  ros2 launch spark_fast_lio os2.launch.yaml use_sim_time:=true
#        ros2 launch sonar_map sonar_mapping_no_semantic.launch.py use_sim_time:=true
#   T2:  ./run_perf_capture.sh --bag ~/bags/mission_1h
#        # then, when prompted, in a third terminal:
#        ros2 bag play ~/bags/mission_1h --clock --rate 1.0
#   Ctrl-C in T2 when the bag finishes.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BAG=""
RUN_ID=""
CONFIG=""
OUT_ROOT=""
INTERVAL="1.0"

usage() {
    sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag)      BAG="$2"; shift 2 ;;
        --run-id)   RUN_ID="$2"; shift 2 ;;
        --config)   CONFIG="$2"; shift 2 ;;
        --out-root) OUT_ROOT="$2"; shift 2 ;;
        --interval) INTERVAL="$2"; shift 2 ;;
        -h|--help)  usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done

RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
CONFIG="${CONFIG:-$SCRIPT_DIR/config.yaml}"
OUT_ROOT="${OUT_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)/perf_data}"
RUN_DIR="$OUT_ROOT/$RUN_ID"
mkdir -p "$RUN_DIR"

export PERF_RUN_ID="$RUN_ID"

echo "=============================================================="
echo " perf_tools capture"
echo "   run id   : $RUN_ID"
echo "   out dir  : $RUN_DIR"
echo "   config   : $CONFIG"
echo "=============================================================="

# --- Reproducibility metadata (each command guarded; skip if absent on PC) ---
dump() {  # dump <outfile> <cmd...>
    local out="$1"; shift
    if command -v "$1" >/dev/null 2>&1; then
        echo "[meta] $* -> $out"
        "$@" >"$RUN_DIR/$out" 2>&1 || echo "(command failed)" >>"$RUN_DIR/$out"
    else
        echo "[meta] skip '$1' (not found on this host)"
        echo "SKIPPED: '$1' not available on this host" >"$RUN_DIR/$out"
    fi
}

if [[ -n "$BAG" ]]; then
    dump bag_info.txt ros2 bag info "$BAG"
else
    echo "WARNING: --bag not given; bag_info.txt will be empty (needed for drop-rate analysis)" >&2
    echo "NO BAG PROVIDED" >"$RUN_DIR/bag_info.txt"
fi
dump nvpmodel.txt nvpmodel -q
dump jetson_release.txt jetson_release
# also stash the config used, for full reproducibility
cp -f "$CONFIG" "$RUN_DIR/config.used.yaml" 2>/dev/null || true

# --- Launch the three loggers in the background ------------------------------
PIDS=()
launch() {  # launch <name> <module>
    echo "[start] $1"
    python3 -m "perf_tools.$2" --run-id "$RUN_ID" --config "$CONFIG" \
        --out-root "$OUT_ROOT" &
    PIDS+=($!)
}

cleanup() {
    echo
    echo "[stop] shutting down loggers..."
    for pid in "${PIDS[@]:-}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]:-}"; do
        wait "$pid" 2>/dev/null || true
    done
    echo "[done] capture written to $RUN_DIR"
    echo "       analyze with: python3 $SCRIPT_DIR/perf_tools/analyze_perf.py --run-dir $RUN_DIR"
}
trap cleanup INT TERM

# resource_logger takes --interval; the rclpy loggers do not
echo "[start] resource_logger"
python3 -m perf_tools.resource_logger --run-id "$RUN_ID" --config "$CONFIG" \
    --out-root "$OUT_ROOT" --interval "$INTERVAL" &
PIDS+=($!)
launch latency_sniffer latency_sniffer
launch map_growth_logger map_growth_logger

sleep 2
echo
echo ">>> Loggers are running. NOW start the bag replay in another terminal:"
if [[ -n "$BAG" ]]; then
    echo "      ros2 bag play \"$BAG\" --clock --rate 1.0"
else
    echo "      ros2 bag play <BAG> --clock --rate 1.0"
fi
echo ">>> Make sure the pipeline nodes were launched with use_sim_time:=true."
echo ">>> Press Ctrl-C here when the bag finishes to stop capture."
echo

# wait on all logger pids; Ctrl-C triggers cleanup via trap
wait
