"""Per-stage latency breakdown (Test 1, Method B) from the two latency-test logs.

1. Perception:  sudo .venv/bin/python server.py --latency-log latency_perception.csv
2. Unreal:      press L in the mirror app (prediction off, wrist marker, logs to
                unreal/Saved/ReflectAR/latency_unreal.csv). Move around for 60 s, press L again.
3. Report:      .venv/bin/python tools/latency_report.py latency_perception.csv [latency_unreal.csv]

Only frames with a tracked person count. Unreal's times are on its own clock; they're mapped to
perception's with the clock offset it logged (same machine, so the offset is exact to ~0.1 ms).
"""
import csv, sys
import numpy as np


def load(path):
    with open(path) as f:
        return {int(r["seq"]): r for r in csv.DictReader(f)}


def stats(ms):
    a = np.asarray(ms)
    return len(a), np.median(a), np.mean(a), np.percentile(a, 95), np.max(a)


if len(sys.argv) < 2:
    sys.exit(__doc__)
perc = {s: r for s, r in load(sys.argv[1]).items() if r["tracked"] == "1"}
unreal = load(sys.argv[2]) if len(sys.argv) > 2 else {}
if not perc:
    sys.exit("no tracked frames in the perception log: was someone in view?")

rows = {k: [] for k in ("exposure -> frame on Mac", "frame on Mac -> landmarks ready",
                        "landmarks -> joints ready (depth, 3D, filters)", "joints ready -> sent",
                        "sent -> renderer receives", "renderer receives -> game thread picks up",
                        "game thread -> on screen (present)", "TOTAL exposure -> present (software)",
                        "TOTAL exposure -> sent (perception only)")}
for seq, r in perc.items():
    t = {k: float(r[k]) for k in ("t_exposure", "t_arrive", "t_pose", "t_joints", "t_send")}
    rows["exposure -> frame on Mac"].append(t["t_arrive"] - t["t_exposure"])
    rows["frame on Mac -> landmarks ready"].append(t["t_pose"] - t["t_arrive"])
    rows["landmarks -> joints ready (depth, 3D, filters)"].append(t["t_joints"] - t["t_pose"])
    rows["joints ready -> sent"].append(t["t_send"] - t["t_joints"])
    rows["TOTAL exposure -> sent (perception only)"].append(t["t_send"] - t["t_exposure"])
    u = unreal.get(seq)
    if u:
        off = float(u["clock_offset"])
        ua, up, upr = (float(u[k]) - off for k in ("t_arrive_local", "t_poll_local", "t_present_local"))
        rows["sent -> renderer receives"].append(ua - t["t_send"])
        rows["renderer receives -> game thread picks up"].append(up - ua)
        rows["game thread -> on screen (present)"].append(upr - up)
        rows["TOTAL exposure -> present (software)"].append(upr - t["t_exposure"])

print(f"{len(perc)} tracked frames in the perception log, {sum(s in unreal for s in perc)} matched in the Unreal log\n")
print("| Stage | Frames | Median (ms) | Mean (ms) | 95th pct (ms) | Max (ms) |")
print("| --- | --- | --- | --- | --- | --- |")
for name, v in rows.items():
    if v:
        n, med, mean, p95, mx = stats(np.asarray(v) * 1000)
        print(f"| {name} | {n} | {med:.1f} | {mean:.1f} | {p95:.1f} | {mx:.1f} |")
if not unreal:
    print("\n(no Unreal log given: renderer and display stages not included)")
print("\nThe display's own delay (monitor/TV processing + refresh) is not visible to software: "
      "Method A total - 'TOTAL exposure -> present' estimates it.")
