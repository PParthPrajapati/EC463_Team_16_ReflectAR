"""Shoulder alignment test (Test 2), hand-marked: capture frames, click where each shoulder really
is, and compare against BlazePose. The camera's depth is trusted (D415: within ~2% at these
distances), so no tape measurements are needed.

  1. capture (camera; macOS needs sudo): N frames of the subject, each saved with its aligned
     depth map and BlazePose's landmarks
       sudo .venv/bin/python tools/shoulder_click.py capture --out RUNS --label 1.4m_0deg [--n 10]
  2. mark (no sudo): a window shows each frame; click both shoulder joints, in either order
     (the click further right in the image is the subject's left). BlazePose's points are hidden
     so they can't steer the clicks (B shows them). A zoomed inset follows the cursor.
     Keys: U undo, S skip frame, B show/hide BlazePose, Q quit (progress is saved).
       .venv/bin/python tools/shoulder_click.py mark --out RUNS [--marker arav]
     Mark the same frames twice (or with two people, --marker NAME) to get the marking precision.
  3. report: per-marker table and, with two or more markers, how far apart their clicks are
       .venv/bin/python tools/shoulder_click.py report --out RUNS

Per frame and shoulder: the clicked point and BlazePose's point are each back-projected with the
camera's depth at that pixel (the body surface, picked like server.py does), and the report
gives the 3D distance between them plus its across-image part.
"""
import argparse, glob, json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
L_SH, R_SH = 11, 12
TORSO = (11, 12, 23, 24)
SIDES = ((L_SH, "left"), (R_SH, "right"))


# ------------------------------------------------------------ depth (same picks as realsense.py)
def window(d, scale, u, v, r):
    H, W = d.shape
    x, y = int(np.clip(u, 0, W - 1)), int(np.clip(v, 0, H - 1))
    win = d[max(y - r, 0):y + r + 1, max(x - r, 0):x + r + 1].astype(np.float64) * scale
    return win[win > 0]


def sample(d, scale, u, v, r=6):
    win = window(d, scale, u, v, r)
    win = win[(win > 0.2) & (win < 8.0)]
    return float(np.median(win)) if win.size else None


def surface(d, scale, u, v, guess, band=0.35, r=5):
    """Nearest body surface within `band` of guess: a pixel on the silhouette edge reads the
    shoulder, not the wall behind it (SensorDepth.joint)."""
    win = window(d, scale, u, v, r)
    win = win[np.abs(win - guess) < band]
    return float(np.percentile(win, 25)) if win.size >= 4 else None


def backproject(K, u, v, z):
    return np.array([(u - K["cx"]) * z / K["fx"], (v - K["cy"]) * z / K["fy"], z])


def shoulder_guess(meta, d, scale):
    """BlazePose's shoulder depth guess, as server.py forms it: torso depth + relative z."""
    uv, world, vis = np.array(meta["uv"]), np.array(meta["world"]), np.array(meta["vis"])
    s = [x for x in (sample(d, scale, *uv[i]) for i in TORSO if vis[i] > 0.5) if x]
    if not s:
        return None
    z_root = float(np.median(s))
    return {i: z_root + (world[i, 2] - world[list(TORSO), 2].mean()) for i in (L_SH, R_SH)}


def chown_out(out):
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid:                                          # written as root: hand back to the user
        for f in [out] + [os.path.join(out, x) for x in os.listdir(out)]:
            os.chown(f, int(uid), int(gid))


# ------------------------------------------------------------------------------------ capture
def capture(a):
    import realsense
    from backend import Backend, warmup
    from blazepose import BlazePose, DET_SIZE, LM_SIZE
    os.makedirs(a.out, exist_ok=True)
    cfg = json.load(open(os.path.join(HERE, "calib", "config.json")))
    backend = Backend(cfg.get("compute", {}))
    pose = BlazePose(backend)
    warmup(pose.det, (1, DET_SIZE, DET_SIZE, 3)); warmup(pose.lm, (1, LM_SIZE, LM_SIZE, 3))
    rc = dict(cfg["camera"].get("realsense", {}))
    if os.environ.get("RS_NO_RESET"):
        rc["reset_on_start"] = False
    cam = realsense.RealSenseCamera(rc)
    print("[cam]", cam.desc)
    seq, t_next, got = 0, time.time() + 1.0, 0       # 1 s for exposure to settle
    t_end = time.time() + 1.0 + a.n * a.every + 10
    print(f"[capture] {a.label}: hold the pose; {a.n} frames, one every {a.every:.1f} s")
    while got < a.n and time.time() < t_end:
        frame, stamp, seq = cam.read(seq)
        res = pose(frame)                             # every frame, so BlazePose keeps tracking
        if time.time() < t_next or res is None:
            continue
        t_next = time.time() + a.every
        name = f"{a.label}_{got:02d}"
        cv2.imwrite(os.path.join(a.out, name + ".png"), frame)
        np.save(os.path.join(a.out, name + "_depth.npy"), cam.depth)
        json.dump({"label": a.label, "frame": name, "t": stamp, "uv": np.asarray(res["uv"]).tolist(),
                   "world": np.asarray(res["world"]).tolist(), "vis": np.asarray(res["vis"]).tolist(),
                   "K": cam.intrinsics, "depth_scale": cam.depth_scale, "camera": cam.desc},
                  open(os.path.join(a.out, name + ".json"), "w"))
        got += 1
        print(f"  {name}")
    chown_out(a.out)
    if got < a.n:
        raise RuntimeError(f"only {got}/{a.n} frames had a person in view")
    print(f"[capture] saved {got} frames to {a.out}")


# --------------------------------------------------------------------------------------- mark
def frames(out):
    return sorted(f[:-5] for f in os.listdir(out) if f.endswith(".json") and not f.startswith("marks_"))


def mark(a):
    path = os.path.join(a.out, f"marks_{a.marker}.json")
    marks = json.load(open(path)) if os.path.exists(path) else {}
    todo = [f for f in frames(a.out) if f not in marks]
    if not todo:
        print(f"every frame already marked by {a.marker}: {path}")
        return
    S, Z, R = a.scale, 4, 60                          # display scale, inset zoom, inset half-size (px)
    st = {"clicks": [], "mouse": (0, 0), "show_bp": False}

    def on_mouse(ev, x, y, flags, _):
        st["mouse"] = (x, y)
        if ev == cv2.EVENT_LBUTTONDOWN and len(st["clicks"]) < 2:
            st["clicks"].append((x / S, y / S))

    win = "ReflectAR shoulder marking"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    i = 0
    while i < len(todo):
        name = todo[i]
        img = cv2.imread(os.path.join(a.out, name + ".png"))
        meta = json.load(open(os.path.join(a.out, name + ".json")))
        big = cv2.resize(img, None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
        st["clicks"] = []
        while True:
            show = big.copy()
            prompt = ["click one shoulder joint", "click the other shoulder joint", "done"][len(st["clicks"])]
            cv2.putText(show, f"{name}  ({i + 1}/{len(todo)})   {prompt}   [U undo  S skip  B BlazePose  Q quit]",
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            for k, (u, v) in enumerate(st["clicks"]):
                cv2.drawMarker(show, (int(u * S), int(v * S)), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
            if st["show_bp"]:
                for j in (L_SH, R_SH):
                    u, v = meta["uv"][j]
                    cv2.drawMarker(show, (int(u * S), int(v * S)), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
            # zoomed inset around the cursor, top-right corner
            mx, my = st["mouse"]
            h, w = show.shape[:2]
            x0, y0 = int(np.clip(mx - R, 0, w - 2 * R)), int(np.clip(my - R, 0, h - 2 * R))
            inset = cv2.resize(show[y0:y0 + 2 * R, x0:x0 + 2 * R], None, fx=Z / 2, fy=Z / 2,
                               interpolation=cv2.INTER_NEAREST)
            c = inset.shape[0] // 2
            cv2.line(inset, (c, 0), (c, 2 * c), (0, 255, 0), 1); cv2.line(inset, (0, c), (2 * c, c), (0, 255, 0), 1)
            show[40:40 + inset.shape[0], w - inset.shape[1] - 10:w - 10] = inset
            cv2.imshow(win, show)
            k = cv2.waitKey(20) & 0xFF
            if len(st["clicks"]) == 2:
                # either order: the camera image isn't mirrored, so the subject's left shoulder is
                # the click further right in the image
                rt, lf = sorted(st["clicks"])
                marks[name] = {"left": list(map(float, lf)), "right": list(map(float, rt))}
                json.dump(marks, open(path, "w"), indent=1)
                i += 1
                break
            if k in (ord("u"), ord("U")) and st["clicks"]:
                st["clicks"].pop()
            elif k in (ord("b"), ord("B")):
                st["show_bp"] = not st["show_bp"]
            elif k in (ord("s"), ord("S")):
                i += 1
                break
            elif k in (ord("q"), ord("Q"), 27):
                cv2.destroyAllWindows()
                print(f"saved {len(marks)} marked frames to {path}")
                return
    cv2.destroyAllWindows()
    print(f"saved {len(marks)} marked frames to {path}")


# ------------------------------------------------------------------------------------- report
def offsets(out, name, click):
    """{side: (3D offset cm, across-image cm, pixel offset)} for one frame's clicks."""
    meta = json.load(open(os.path.join(out, name + ".json")))
    d = np.load(os.path.join(out, name + "_depth.npy"))
    K, sc = meta["K"], meta["depth_scale"]
    guess = shoulder_guess(meta, d, sc)
    if guess is None:
        return {}
    res = {}
    for i, side in SIDES:
        (u, v), (cu, cv_) = meta["uv"][i], click[side]
        zb = surface(d, sc, u, v, guess[i])                       # pipeline: BlazePose's pixel
        zc = surface(d, sc, cu, cv_, zb if zb else guess[i])      # truth: the clicked pixel
        if zb is None or zc is None:
            continue
        px = float(np.hypot(u - cu, v - cv_))
        res[side] = (100 * float(np.linalg.norm(backproject(K, u, v, zb) - backproject(K, cu, cv_, zc))),
                     100 * px * zc / K["fx"], px, zc)
    return res


def report(a):
    files = sorted(glob.glob(os.path.join(a.out, "marks_*.json")))
    if not files:
        sys.exit("no marks yet: run mark first")
    allmarks = {os.path.basename(f)[6:-5]: json.load(open(f)) for f in files}
    print("| Marker | Recording | Shoulders | 3D offset mean / 95th / max (cm) | Across-image mean / 95th (cm) "
          "| Mean depth (m) | <3 cm (95th)? |")
    print("| --- | --- | --- | --- | --- | --- | --- |")
    for who, marks in allmarks.items():
        by_label = {}
        for name, click in marks.items():
            label = name.rsplit("_", 1)[0]
            for side, v in offsets(a.out, name, click).items():
                by_label.setdefault(label, []).append(v)
        rows = [(lb, v) for lb, v in sorted(by_label.items())] + ([("**all**", sum(by_label.values(), []))]
                                                                 if len(by_label) > 1 else [])
        for lb, v in rows:
            o, lat, z = (np.array([x[k] for x in v]) for k in (0, 1, 3))
            print(f"| {who} | {lb} | {len(v)} | {o.mean():.1f} / {np.percentile(o, 95):.1f} / {o.max():.1f} | "
                  f"{lat.mean():.1f} / {np.percentile(lat, 95):.1f} | {z.mean():.2f} | "
                  f"{'PASS' if np.percentile(o, 95) < 3.0 else 'FAIL'} |")
    if len(allmarks) > 1:
        names = list(allmarks)
        gaps = []
        for f in set.intersection(*(set(m) for m in allmarks.values())):
            meta = json.load(open(os.path.join(a.out, f + ".json")))
            d = np.load(os.path.join(a.out, f + "_depth.npy"))
            for side in ("left", "right"):
                pts = [np.array(allmarks[n][f][side]) for n in names]
                z = sample(d, meta["depth_scale"], *pts[0]) or 1.5
                for p in pts[1:]:
                    gaps.append(100 * float(np.linalg.norm(p - pts[0])) * z / meta["K"]["fx"])
        if gaps:
            g = np.array(gaps)
            print(f"\nMarking precision ({' vs '.join(names)}, {len(g)} shoulders): clicks differ by "
                  f"{g.mean():.1f} cm on average, {np.percentile(g, 95):.1f} cm at the 95th percentile")


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("mode", choices=["capture", "mark", "report"])
ap.add_argument("--out", required=True, help="folder for the frames and marks")
ap.add_argument("--label", help="capture: name for this position, e.g. 1.4m_0deg")
ap.add_argument("--n", type=int, default=10, help="capture: frames to save")
ap.add_argument("--every", type=float, default=0.4, help="capture: seconds between saved frames")
ap.add_argument("--marker", default="pass1", help="mark: whose marks these are (or pass1 / pass2)")
ap.add_argument("--scale", type=float, default=1.5, help="mark: display scale of the frames")
a = ap.parse_args()
a.out = os.path.abspath(a.out)
if a.mode == "capture":
    if not a.label:
        ap.error("capture needs --label")
    try:
        capture(a)
    except BaseException:                             # realsense's atexit hook would exit 0
        import traceback; traceback.print_exc(); sys.stdout.flush(); os._exit(1)
    os._exit(0)                                       # librealsense segfaults at interpreter exit
elif a.mode == "mark":
    mark(a)
else:
    report(a)
