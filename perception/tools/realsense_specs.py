"""RealSense D415 spec bench: link speed, firmware, offered modes, and measured frame rate,
dropped frames and sensor->host latency for a set of colour + depth mode pairs. Saves a colour
and colourised-depth snapshot for each mode and a results.json.

Run from perception/ (macOS needs sudo):
    sudo .venv/bin/python tools/realsense_specs.py OUT_DIR [seconds_per_mode] [mode,mode,...]
"""
import faulthandler, json, os, sys, time
faulthandler.enable()
import numpy as np
import cv2
import pyrealsense2 as rs

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "realsense_specs_out")
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
os.makedirs(OUT, exist_ok=True)

# (label, colour (w, h, fps) or None, depth (w, h, fps) or None)
MODES = [
    ("max_res",      (1920, 1080, 30), (1280, 720, 30)),
    ("hd_720",       (1280, 720, 30),  (1280, 720, 30)),
    ("reflectar",    (960, 540, 60),   (848, 480, 90)),   # what perception/realsense.py uses on USB 3
    ("low_res_fast", (640, 360, 60),   (480, 270, 90)),
    ("depth_only_90", None,            (848, 480, 90)),
    ("color_only_60", (960, 540, 60),  None),
]


CTX = rs.context()                                    # one context for the whole run: a device
_keep = []                                            # outliving its context segfaults librealsense


def device():
    devs = CTX.query_devices()
    _keep.append(devs)
    if len(devs) == 0:
        sys.exit("No RealSense found (on macOS run with sudo)")
    return devs[0]


def reset_and_wait(timeout=20.0):
    """Power-cycle the camera (clears a stuck/half-claimed state) and wait for it to re-enumerate."""
    print("resetting camera...", flush=True)
    device().hardware_reset()
    t0, gone = time.time(), False
    while time.time() - t0 < timeout:
        time.sleep(0.5)
        devs = CTX.query_devices()
        _keep.append(devs)
        gone = gone or len(devs) == 0
        if gone and len(devs) > 0:
            time.sleep(2.0)
            return device()
    sys.exit("camera did not come back after reset: replug it")


def info(dev):
    g = lambda k: dev.get_info(k) if dev.supports(k) else None
    return {"name": g(rs.camera_info.name), "serial": g(rs.camera_info.serial_number),
            "firmware": g(rs.camera_info.firmware_version),
            "usb_type": g(rs.camera_info.usb_type_descriptor),
            "librealsense": rs.__version__ if hasattr(rs, "__version__") else None}


def offered(dev):
    out = {}
    for s in dev.query_sensors():
        name = s.get_info(rs.camera_info.name)
        modes = set()
        for p in s.get_stream_profiles():
            v = p.as_video_stream_profile()
            modes.add(f"{p.stream_name()} {p.format()} {v.width()}x{v.height()}@{p.fps()}".replace("format.", ""))
        out[name] = sorted(modes)
    return out


def run_mode(label, col, dep):
    reset_and_wait()
    pipe, cfg = rs.pipeline(CTX), rs.config()
    if col:
        cfg.enable_stream(rs.stream.color, col[0], col[1], rs.format.bgr8, col[2])
    if dep:
        cfg.enable_stream(rs.stream.depth, dep[0], dep[1], rs.format.z16, dep[2])
    res = {"label": label, "requested_color": col, "requested_depth": dep}
    try:
        prof = pipe.start(cfg)
    except Exception as e:
        res["error"] = f"refused: {e}"
        return res
    depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
    stats = {k: {"n": 0, "first_num": None, "last_num": None, "host": [], "lag_ms": [], "domain": None}
             for k in ("color", "depth")}
    last = {}
    try:
        t_end = time.time() + 2.0                      # warm-up: auto-exposure settles
        while time.time() < t_end:
            pipe.wait_for_frames(5000)
        t0 = time.time()
        while time.time() - t0 < SECS:
            fs = pipe.wait_for_frames(5000)
            now = time.time()
            for key, f in (("color", fs.get_color_frame()), ("depth", fs.get_depth_frame())):
                if not f:
                    continue
                st = stats[key]
                num = f.get_frame_number()
                if st["last_num"] == num:              # same frame repeated in a frameset
                    continue
                st["n"] += 1
                st["first_num"] = num if st["first_num"] is None else st["first_num"]
                st["last_num"] = num
                st["host"].append(now)
                st["domain"] = str(f.get_frame_timestamp_domain())
                st["lag_ms"].append(now * 1000 - f.get_timestamp())   # global time -> host clock
                last[key] = f
    except Exception as e:
        res["error"] = f"no frames: {e}"
    finally:
        pipe.stop()
    for key, st in stats.items():
        if st["n"] < 2:
            continue
        h = np.array(st["host"])
        span = st["last_num"] - st["first_num"] + 1
        lag = np.array(st["lag_ms"])
        res[key] = {
            "frames": st["n"],
            "fps_measured": round((st["n"] - 1) / (h[-1] - h[0]), 2),
            "frame_number_span": span,
            "dropped": int(span - st["n"]),
            "interval_ms_p50": round(float(np.median(np.diff(h)) * 1000), 2),
            "interval_ms_p99": round(float(np.percentile(np.diff(h), 99) * 1000), 2),
            "timestamp_domain": st["domain"],
            "sensor_to_host_ms_p50": round(float(np.median(lag)), 1),
            "sensor_to_host_ms_p95": round(float(np.percentile(lag, 95)), 1),
        }
    if "depth" in last:
        d = np.asanyarray(last["depth"].get_data())
        H, W = d.shape
        c = d[H // 2 - 5:H // 2 + 5, W // 2 - 5:W // 2 + 5]
        c = c[c > 0]
        res["depth"]["fill_ratio"] = round(float(np.count_nonzero(d) / d.size), 3)
        res["depth"]["centre_m"] = round(float(np.median(c) * depth_scale), 3) if c.size else None
        res["depth"]["depth_scale_m"] = depth_scale
        vis = cv2.applyColorMap(cv2.convertScaleAbs(d, alpha=255 / (4.0 / depth_scale)), cv2.COLORMAP_TURBO)
        vis[d == 0] = 0
        cv2.imwrite(os.path.join(OUT, f"{label}_depth.png"), vis)
        np.save(os.path.join(OUT, f"{label}_depth_raw.npy"), d)
    if "color" in last:
        cv2.imwrite(os.path.join(OUT, f"{label}_color.png"), np.asanyarray(last["color"].get_data()))
    return res


def attempt(fn, *a, tries=5):
    """macOS races librealsense for the camera ("failed to set power state"): reset and retry."""
    for i in range(1, tries + 1):
        try:
            return fn(*a)
        except (RuntimeError, SystemExit) as e:
            print(f"  attempt {i} failed: {e}", flush=True)
            time.sleep(2.0)
    return None


def save(report):
    path = os.path.join(OUT, "results.json")
    old = json.load(open(path)) if os.path.exists(path) else {}
    merged = {m["label"]: m for m in old.get("modes", [])}
    merged.update({m["label"]: m for m in report["modes"]})
    report = {**old, **report, "modes": list(merged.values())}
    json.dump(report, open(path, "w"), indent=1)
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid:                                           # hand files back to the user, not root
        for f in os.listdir(OUT):
            os.chown(os.path.join(OUT, f), int(uid), int(gid))
        os.chown(OUT, int(uid), int(gid))
    return path


only = set(sys.argv[3].split(",")) if len(sys.argv) > 3 else None   # e.g. hd_720,reflectar
dev = attempt(reset_and_wait)
if dev is None:
    sys.exit("could not open the camera")
report = {"device": info(dev), "offered_modes": offered(dev), "seconds_per_mode": SECS, "modes": []}
print(json.dumps(report["device"], indent=1))
for label, col, dep in MODES:
    if only and label not in only:
        continue
    print(f"--- {label}", flush=True)
    r = None
    for i in range(4):
        r = attempt(run_mode, label, col, dep) or {"label": label, "error": "camera would not open"}
        if "error" not in r:
            break
        print(f"  {r['error']}: retrying", flush=True)
    report["modes"].append(r)
    print(json.dumps({k: v for k, v in r.items() if k not in ("requested_color", "requested_depth")}, indent=1), flush=True)
    save(report)
print("wrote", save(report), flush=True)
os._exit(0)                                           # librealsense segfaults at interpreter exit
