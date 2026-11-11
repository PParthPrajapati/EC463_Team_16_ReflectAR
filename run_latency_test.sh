#!/bin/bash
# Test 1 (latency), one command: perception with per-stage timing + the Unreal mirror app, then the
# stage table. In the app: press L (prediction off, magenta wrist marker, logging on), move for
# ~60 s, press L again, then quit the app (Cmd+Q). The marker is also what the slow-motion video
# (Method A) films.
#
#   ./run_latency_test.sh            (asks for your password: macOS only lets root open the D415)
#
# Results: test_runs/latency_<time>/ (both CSVs + report.md).
cd "$(dirname "$0")" || exit 1
ROOT=$PWD
UE_ROOT="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
EDITOR="$UE_ROOT/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
[ -x "$EDITOR" ] || { echo "Unreal Editor not found at $EDITOR (set UE_ROOT)"; exit 1; }
OUT="$ROOT/test_runs/latency_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
UNREAL_CSV="$ROOT/unreal/Saved/ReflectAR/latency_unreal.csv"
rm -f "$UNREAL_CSV"

sudo -v || exit 1
# A perception left running (an earlier run stopped with Ctrl-C) holds the camera and port 5566:
# stop it first, and always stop ours on the way out, Ctrl-C included.
stop_perception() { sudo pkill -INT -f "server.py" 2>/dev/null; sleep 2; sudo pkill -9 -f "server.py" 2>/dev/null; }
if pgrep -f "server.py" >/dev/null; then
    echo "[test] stopping a perception service left running by an earlier run"
    stop_perception
fi
trap 'stop_perception' EXIT
trap 'echo; echo "[test] interrupted"; exit 130' INT TERM
# macOS's camera service (UVCAssistant) races librealsense for the D415 and causes "failed to set
# power state". Each try restarts it (launchd relaunches it at once). Tries 1-2 skip perception's
# power-cycle (which lets the service grab the camera again); tries 3-4 use it, which unsticks a
# camera left streaming nothing by an earlier run.
ok=0
for try in 1 2 3 4; do
    sudo killall UVCAssistant 2>/dev/null; sleep 1
    NORESET=$([ $try -le 2 ] && echo 1)
    echo "[test] starting perception, try $try (log: $OUT/perception.log)"
    (cd perception && sudo env PYTHONDONTWRITEBYTECODE=1 RS_NO_RESET=$NORESET .venv/bin/python -u server.py \
        --latency-log "$OUT/latency_perception.csv" > "$OUT/perception.log" 2>&1) &
    PID=$!
    # wait until it's serving (camera open) or it has died
    for i in $(seq 1 60); do
        grep -q "listening" "$OUT/perception.log" 2>/dev/null && break
        kill -0 $PID 2>/dev/null || break
        sleep 1
    done
    sleep 2
    if grep -q "\[cam\] RealSense" "$OUT/perception.log" && kill -0 $PID 2>/dev/null; then ok=1; break; fi
    echo "[test]   perception didn't start: $(grep -E 'Error|error:' "$OUT/perception.log" | tail -1)"
    stop_perception
done
if [ $ok = 0 ]; then
    echo "[test] perception couldn't open the camera after 4 tries: unplug the D415, plug it back in, run again"
    exit 1
fi

echo "[test] starting the mirror app: press L, move ~60 s, press L, then Cmd+Q to finish"
(cd unreal && "$EDITOR" "$ROOT/unreal/ReflectAR.uproject" -game -windowed -ResX=1600 -ResY=900 > "$OUT/unreal.log" 2>&1)

stop_perception
sudo chown -R "$(id -u):$(id -g)" "$OUT"
[ -f "$UNREAL_CSV" ] && cp "$UNREAL_CSV" "$OUT/" || echo "[test] no Unreal log: was L pressed?"
perception/.venv/bin/python perception/tools/latency_report.py "$OUT/latency_perception.csv" \
    $( [ -f "$OUT/latency_unreal.csv" ] && echo "$OUT/latency_unreal.csv") | tee "$OUT/report.md"
echo "[test] results in $OUT"
