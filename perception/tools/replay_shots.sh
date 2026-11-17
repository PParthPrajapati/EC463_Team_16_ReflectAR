#!/bin/bash
# Replay a recording (tools/record_spin.py) through perception and the real renderer, and collect
# the renderer's screenshots:  tools/replay_shots.sh <recording dir> <frames a:b> <out dir> <count> [editor args...]
# e.g. tools/replay_shots.sh ../test_runs/spin_... 60:330 ../test_runs/ue/still 6 -ReflectARShots=3 -ReflectARCycle=4
# Editor args: -ReflectARProxy (show the occluders), -ReflectARNoStab, -ReflectARNoMask, -stdout -FullStdOutLogOutput
# (prints "[ReflectAR body]" lines: measured yaw and sizes). SERVER_ARGS adds perception flags, e.g. --no-shape.
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
UE_ROOT="${UE_ROOT:-/Users/Shared/Epic Games/UE_5.8}"
REC="$(cd "$1" && pwd)"; FR=$2; OUT=$3; N=$4; shift 4
LOG="${TMPDIR:-/tmp}"
pkill -f 'server.py.*--recording' 2>/dev/null; pkill -f 'UnrealEditor.*-game' 2>/dev/null; sleep 1
rm -f "$ROOT"/unreal/Saved/Screenshots/shot_*.png
(cd "$ROOT/perception" && .venv/bin/python -u server.py --log --recording "$REC" --frames "$FR" $SERVER_ARGS > "$LOG/replay_server.log" 2>&1 &)
sleep 4
("$UE_ROOT/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" "$ROOT/unreal/ReflectAR.uproject" -game -windowed \
    -ResX=1280 -ResY=832 -nosplash -unattended "$@" > "$LOG/replay_renderer.log" 2>&1 &)
for i in $(seq 1 100); do
    sleep 3
    [ "$(find "$ROOT/unreal/Saved/Screenshots" -name 'shot_*.png' 2>/dev/null | wc -l)" -ge "$N" ] && break
    pgrep -q UnrealEditor || { echo "the renderer exited: see $LOG/replay_renderer.log"; break; }
done
pkill -f 'UnrealEditor.*-game'; pkill -f 'server.py.*--recording'
mkdir -p "$OUT" && cd "$ROOT" && mv "$ROOT"/unreal/Saved/Screenshots/shot_*.png "$OUT"/ && echo "$(ls "$OUT" | wc -l | tr -d ' ') screenshots in $OUT"
