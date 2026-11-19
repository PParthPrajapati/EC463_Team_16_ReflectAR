#!/bin/bash
# Test 2 (shoulder alignment), one command: capture 10 frames of the subject standing still, click
# the true shoulder joints in a window, then the offset table (BlazePose vs your clicks).
#
#   ./run_shoulder_test.sh [label]      e.g. ./run_shoulder_test.sh 1.4m_0deg
#
# Stand facing the camera, arms down, and hold still for ~5 s while it captures. Then click, in each
# frame, both SHOULDER JOINTS in either order: the top of the joint, where the collarbone ends and
# the arm begins (not the armpit).
# Keys: U undo, S skip, Q quit. Asks for your password (macOS only lets root open the D415).
#
# Results: test_runs/shoulder_<time>/ (frames, depth maps, marks, report.md).
cd "$(dirname "$0")" || exit 1
ROOT=$PWD
LABEL="${1:-demo}"
OUT="$ROOT/test_runs/shoulder_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

sudo -v || exit 1
ok=0
for i in 1 2 3 4; do
    echo "[test] capture, try $i (hold still, facing the camera)"
    # restart macOS's camera service so it lets go of the D415, and skip the reset (see realsense.py)
    sudo killall UVCAssistant 2>/dev/null; sleep 1
    if (cd perception && sudo env PYTHONDONTWRITEBYTECODE=1 RS_NO_RESET=1 .venv/bin/python -u \
            tools/shoulder_click.py capture --out "$OUT" --label "$LABEL" --n 10); then ok=1; break; fi
    sleep 2
done
sudo chown -R "$(id -u):$(id -g)" "$OUT"
[ $ok = 1 ] || { echo "[test] the camera wouldn't open: replug it and run again"; exit 1; }

echo "[test] a window opens: click both shoulder joints on each frame (any order; top of the joint, not the armpit)"
perception/.venv/bin/python perception/tools/shoulder_click.py mark --out "$OUT" --marker "$(id -un)"
perception/.venv/bin/python perception/tools/shoulder_click.py report --out "$OUT" | tee "$OUT/report.md"
echo "[test] results in $OUT  (mark again with: perception/.venv/bin/python perception/tools/shoulder_click.py mark --out $OUT --marker pass2)"
