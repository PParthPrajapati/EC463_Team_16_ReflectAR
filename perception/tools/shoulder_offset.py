"""Shoulder alignment test (Test 2): hand-placed shoulder stickers vs the pipeline's 3D shoulders.

The subject wears one bright sticker on the front of each shoulder joint. For each recording
you tape-measure the straight-line range from the D415's colour (RGB) lens to each sticker, and
the sticker-to-sticker shoulder width. The script records a few seconds of frames and, per
frame and shoulder, compares:

  ground truth  = the ray through the sticker's pixel, scaled to the tape-measured range
  pipeline      = BlazePose's shoulder pixel at the RealSense surface depth there, back-projected
                  exactly as server.py does, but BEFORE the surface-to-joint offset (the
                  stickers are on the surface)

and reports the 3D offset, its across-image and depth parts, the camera's own depth error at
the sticker, and the shoulder-width error.

Run from perception/ (macOS needs sudo):
  check the stickers are found (writes OUT/preview.jpg, no measurement):
    sudo .venv/bin/python tools/shoulder_offset.py preview --out OUT
  record one position:
    sudo .venv/bin/python tools/shoulder_offset.py record --out OUT --label 1.5m_0deg \\
        --range-left 1.52 --range-right 1.50 --width 0.39 [--secs 3] [--color green]
  table over every recording in OUT (no camera needed):
    .venv/bin/python tools/shoulder_offset.py summary --out OUT

"left"/"right" are the subject's own shoulders (BlazePose landmarks 11 and 12).
"""
import argparse, csv, json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
L_SH, R_SH = 11, 12
TORSO = (11, 12, 23, 24)
# sticker colours in OpenCV HSV (H 0-179): (low, high). Orange is tuned to the team's stickers
# under warm room light (measured H 16-26, S 65-175, V 60-140); skin sits at H ~11-13.
COLORS = {"green": ((40, 90, 60), (85, 255, 255)), "pink": ((140, 90, 80), (172, 255, 255)),
          "orange": ((14, 90, 50), (32, 255, 255)), "blue": ((95, 120, 60), (128, 255, 255))}


def find_stickers(bgr, color, uv, min_area=15):
    """Sticker centre (u, v) near each BlazePose shoulder: the largest blob of the sticker colour
    within 0.4 shoulder-widths of the landmark, so skin, the wall and other warm things elsewhere
    in the frame can't be picked. Returns ({11: centre or None, 12: ...}, colour mask)."""
    lo, hi = COLORS[color]
    m = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), np.array(lo), np.array(hi))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(m)
    r = 0.4 * float(np.linalg.norm(uv[L_SH] - uv[R_SH]))
    out = {}
    for i in (L_SH, R_SH):
        near = [(int(st[k, cv2.CC_STAT_AREA]), k) for k in range(1, n)
                if st[k, cv2.CC_STAT_AREA] >= min_area and np.linalg.norm(cen[k] - uv[i]) < r]
        out[i] = np.array(cen[max(near)[1]]) if near else None
    return out, m


def ray(K, u, v):
    d = np.array([(u - K["cx"]) / K["fx"], (v - K["cy"]) / K["fy"], 1.0])
    return d / np.linalg.norm(d)


def backproject(K, u, v, z):
    return np.array([(u - K["cx"]) * z / K["fx"], (v - K["cy"]) * z / K["fy"], z])


def chown_out(out):
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid:                                          # written as root: hand back to the user
        for f in [out] + [os.path.join(out, x) for x in os.listdir(out)]:
            os.chown(f, int(uid), int(gid))


def open_pipeline():
    import realsense
    from backend import Backend, warmup
    from blazepose import BlazePose, DET_SIZE, LM_SIZE
    cfg = json.load(open(os.path.join(HERE, "calib", "config.json")))
    backend = Backend(cfg.get("compute", {}))
    pose = BlazePose(backend)
    warmup(pose.det, (1, DET_SIZE, DET_SIZE, 3)); warmup(pose.lm, (1, LM_SIZE, LM_SIZE, 3))
    rc = dict(cfg["camera"].get("realsense", {}))
    if os.environ.get("RS_NO_RESET"):               # first try without the power-cycle (see record_retry)
        rc["reset_on_start"] = False
    cam = realsense.RealSenseCamera(rc)
    print("[cam]", cam.desc)
    return cfg, pose, cam, realsense.SensorDepth(cam)


def shoulders(res, depth, W, H):
    """Pipeline surface depth (m, camera z) at each shoulder, the way server.py picks it:
    the torso's measured depth plus BlazePose's relative z as the guess, then the nearest body
    surface near that guess at the landmark pixel."""
    uv, world, vis = res["uv"], res["world"], res["vis"]
    samples = [s for s in (depth.sample(*uv[i], W, H) for i in TORSO if vis[i] > 0.5) if s]
    if not samples:
        return None
    z_root = float(np.median(samples))
    out = {}
    for i in (L_SH, R_SH):
        guess = z_root + (world[i, 2] - world[list(TORSO), 2].mean())
        out[i] = depth.joint(*uv[i], W, H, guess) if vis[i] > 0.5 else None
    return out


def annotate(frame, res, stickers):
    dbg = frame.copy()
    if res:
        for i in (L_SH, R_SH):
            u, v = res["uv"][i]
            cv2.drawMarker(dbg, (int(u), int(v)), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
    for u, v in stickers:
        cv2.circle(dbg, (int(u), int(v)), 12, (255, 0, 255), 2)
    cv2.putText(dbg, "yellow + = BlazePose shoulder   magenta o = sticker", (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return dbg


def preview(args):
    """5 s of frames: how often each sticker colour finds a sticker at both shoulders."""
    os.makedirs(args.out, exist_ok=True)
    cfg, pose, cam, depth = open_pipeline()
    colors = sorted(COLORS) if args.color == "auto" else [args.color]
    hits = {c: 0 for c in colors}
    seq, t_end, n = 0, time.time() + 5, 0
    frame, res = None, None
    while time.time() < t_end:
        frame, _, seq = cam.read(seq)
        res = pose(frame)
        n += 1
        if res is not None:
            for c in colors:
                st, _ = find_stickers(frame, c, res["uv"])
                hits[c] += st[L_SH] is not None and st[R_SH] is not None
    best = max(colors, key=lambda c: hits[c])
    st, mask = find_stickers(frame, best, res["uv"]) if res else ({}, np.zeros(frame.shape[:2], np.uint8))
    cv2.imwrite(os.path.join(args.out, "preview_raw.png"), frame)
    path = os.path.join(args.out, "preview.jpg")
    cv2.imwrite(path, np.hstack([annotate(frame, res, [c for c in st.values() if c is not None]),
                                 cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
    chown_out(args.out)
    for c in colors:
        print(f"  {c:7s}: a sticker found at both shoulders in {hits[c]}/{n} frames")
    print(f"person {'found' if res else 'NOT found'} in the last frame. Best colour: {best}. Wrote {path} "
          f"(right half = what the {best} filter sees)")


def record(args):
    os.makedirs(args.out, exist_ok=True)
    cfg, pose, cam, depth = open_pipeline()
    K = cam.intrinsics
    off = cfg["depth"]["torso_surface_offset_m"]
    truth = {L_SH: args.range_left, R_SH: args.range_right}
    rows, seq, snap = [], 0, None
    why = {"frames": 0, "no person": 0, "left sticker missing": 0, "right sticker missing": 0, "no depth": 0}
    last = None
    print(f"[record] {args.label}: hold still for {args.secs:.0f} s...")
    t_end = time.time() + args.secs
    while time.time() < t_end:
        frame, stamp, seq = cam.read(seq)
        H, W = frame.shape[:2]
        why["frames"] += 1
        res = pose(frame)
        last = (frame, res, [])
        if res is None:
            why["no person"] += 1
            continue
        stick, mask = find_stickers(frame, args.color, res["uv"])
        last = (frame, res, [c for c in stick.values() if c is not None], mask)
        if stick[L_SH] is None or stick[R_SH] is None:
            why["left sticker missing"] += stick[L_SH] is None
            why["right sticker missing"] += stick[R_SH] is None
            continue
        z = shoulders(res, depth, W, H)
        if z is None or z[L_SH] is None or z[R_SH] is None:
            why["no depth"] += 1
            continue
        pts = {}
        for i, side in ((L_SH, "left"), (R_SH, "right")):
            (u, v), (su, sv) = res["uv"][i], stick[i]
            p = backproject(K, u, v, z[i])                    # pipeline, shoulder surface
            g = ray(K, su, sv) * truth[i]                      # ground truth, sticker
            # camera's own depth at the sticker: the body surface there (not the wall behind a
            # sticker near the silhouette edge), picked like the pipeline does
            zs = depth.joint(su, sv, W, H, z[i], r=3)
            px = float(np.hypot(u - su, v - sv))
            pts[i] = p
            rows.append({"t": round(stamp, 4), "side": side,
                         "offset_3d_cm": 100 * float(np.linalg.norm(p - g)),
                         "lateral_cm": 100 * px * z[i] / K["fx"],
                         "pixel_offset": px,
                         "depth_err_cm": 100 * (float(np.linalg.norm(p)) - truth[i]),
                         "camera_depth_err_cm": 100 * (float(np.linalg.norm(backproject(K, su, sv, zs))) - truth[i])
                         if zs else float("nan"),
                         "z_surface_m": z[i], "z_joint_m": z[i] + off})
        w3 = float(np.linalg.norm(pts[L_SH] - pts[R_SH]))
        rows.append({"t": round(stamp, 4), "side": "width", "width_m": w3,
                     "width_err_cm": 100 * (w3 - args.width) if args.width else float("nan")})
        snap = annotate(frame, res, [stick[L_SH], stick[R_SH]])
    print("[record] frames:", why)
    if not rows:
        if last is not None:
            dbg = annotate(last[0], last[1], last[2])
            if len(last) > 3:
                dbg = np.hstack([dbg, cv2.cvtColor(last[3], cv2.COLOR_GRAY2BGR)])
            cv2.imwrite(os.path.join(args.out, f"{args.label}_FAILED.jpg"), dbg)
            chown_out(args.out)
        raise RuntimeError(f"no usable frames ({why}); last frame saved as {args.label}_FAILED.jpg")
    keys = ["t", "side", "offset_3d_cm", "lateral_cm", "pixel_offset", "depth_err_cm", "camera_depth_err_cm",
            "width_err_cm", "width_m", "z_surface_m", "z_joint_m"]
    with open(os.path.join(args.out, f"{args.label}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} for r in rows)
    cv2.imwrite(os.path.join(args.out, f"{args.label}.jpg"), snap)
    meta = {"label": args.label, "range_left_m": args.range_left, "range_right_m": args.range_right,
            "width_m": args.width, "secs": args.secs, "color": args.color, "camera": cam.desc,
            "date": time.strftime("%Y-%m-%d %H:%M")}
    json.dump(meta, open(os.path.join(args.out, f"{args.label}.json"), "w"), indent=1)
    chown_out(args.out)
    n = sum(r["side"] == "width" for r in rows)
    print(f"[record] {n} frames kept")
    print_table(args.out, [args.label])


def summarise(rows):
    sh = [r for r in rows if r["side"] in ("left", "right")]
    wrows = [r for r in rows if r["side"] == "width"]
    wd = [float(r["width_err_cm"]) for r in wrows if r["width_err_cm"] not in ("", "nan")]
    col = lambda k: np.array([float(r[k]) for r in sh if r[k] not in ("", "nan")])
    o, lat, dep, cam = col("offset_3d_cm"), col("lateral_cm"), col("depth_err_cm"), col("camera_depth_err_cm")
    return {"frames": len(wrows), "offset_mean": o.mean(), "offset_p95": np.percentile(o, 95), "offset_max": o.max(),
            "lateral_p95": np.percentile(lat, 95), "depth_p95": np.percentile(np.abs(dep), 95),
            "depth_bias": dep.mean(), "camera_depth_p95": np.percentile(np.abs(cam), 95) if cam.size else float("nan"),
            "width_err_mean": np.mean(wd) if wd else float("nan"),
            "width_m": np.median([float(r["width_m"]) for r in wrows]) if wrows and "width_m" in wrows[0] else float("nan")}


def print_table(out, labels=None):
    labels = labels or sorted(f[:-5] for f in os.listdir(out) if f.endswith(".json"))
    print("| Recording | Frames | 3D offset mean / 95th / max (cm) | Lateral 95th (cm) | Depth err 95th (cm) "
          "| Depth bias (cm) | Camera depth err 95th (cm) | Pipeline width (m) | Width err mean (cm) | <3 cm? |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    allrows = []
    for lb in labels:
        with open(os.path.join(out, f"{lb}.csv")) as f:
            rows = list(csv.DictReader(f))
        allrows += rows
        s = summarise(rows)
        print(f"| {lb} | {s['frames']} | {s['offset_mean']:.1f} / {s['offset_p95']:.1f} / {s['offset_max']:.1f} | "
              f"{s['lateral_p95']:.1f} | {s['depth_p95']:.1f} | {s['depth_bias']:+.1f} | {s['camera_depth_p95']:.1f} | "
              f"{s['width_m']:.3f} | {s['width_err_mean']:+.1f} | {'PASS' if s['offset_p95'] < 3.0 else 'FAIL'} |")
    if len(labels) > 1:
        s = summarise(allrows)
        print(f"| **all** | {s['frames']} | {s['offset_mean']:.1f} / {s['offset_p95']:.1f} / {s['offset_max']:.1f} | "
              f"{s['lateral_p95']:.1f} | {s['depth_p95']:.1f} | {s['depth_bias']:+.1f} | {s['camera_depth_p95']:.1f} | "
              f"{s['width_m']:.3f} | {s['width_err_mean']:+.1f} | {'PASS' if s['offset_p95'] < 3.0 else 'FAIL'} |")


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("mode", choices=["preview", "record", "summary"])
ap.add_argument("--out", required=True, help="folder for the recordings")
ap.add_argument("--label", help="name of this recording, e.g. 1.5m_0deg")
ap.add_argument("--range-left", type=float, help="tape range (m), colour lens -> subject's LEFT shoulder sticker")
ap.add_argument("--range-right", type=float, help="tape range (m), colour lens -> subject's RIGHT shoulder sticker")
ap.add_argument("--width", type=float, help="tape distance (m) between the two sticker centres (optional)")
ap.add_argument("--secs", type=float, default=3.0)
ap.add_argument("--color", choices=sorted(COLORS) + ["auto"], default="green",
                help="sticker colour; preview also takes auto (tries them all)")
a = ap.parse_args()
a.out = os.path.abspath(a.out)
if a.mode == "summary":
    print_table(a.out)
elif a.mode == "preview":
    try:
        preview(a)
    except Exception:                                 # realsense's atexit hook would exit 0: fail loudly
        import traceback; traceback.print_exc(); sys.stdout.flush(); os._exit(1)
    os._exit(0)                                       # librealsense segfaults at interpreter exit
else:
    if a.color == "auto":
        ap.error("record needs the sticker colour: pick the one preview found")
    if None in (a.label, a.range_left, a.range_right):
        ap.error("record needs --label, --range-left and --range-right (--width optional)")
    try:
        record(a)
    except BaseException:                         # SystemExit too: realsense's atexit would exit 0
        import traceback; traceback.print_exc(); sys.stdout.flush(); os._exit(1)
    os._exit(0)
