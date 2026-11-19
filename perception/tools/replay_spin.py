"""Turn a record_spin.py recording into the tracking stream the renderer would have received.

Runs BlazePose on every colour frame and builds the joints the way server.py does, with the
distance (and, with --depth sensor, each visible joint's depth) read from the recorded depth
map. Writes into the recording's folder:
    track.txt       line 1: fx fy cx cy w h camx camy camz frames; then per frame:
                    index time tracked, 33 x (x y z visibility) in the Mirror frame
    m_00000.pgm     person mask (256 wide, rows top first), for tracked frames

Run from perception/:  .venv/bin/python tools/replay_spin.py ../test_runs/spin_... [--depth rel]
"""
import argparse, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import cv2
from backend import Backend
from blazepose import BlazePose, frame_mask
from filters import OneEuro, PosePredictor
from geometry import Intrinsics, skeleton_distance, CameraToMirror

TORSO = (11, 12, 23, 24)
OFF = {"torso": 0.10, "limb": 0.04, "face": 0.01}       # measured surface -> joint centre, m


def window(d, scale, u, v, r):
    H, W = d.shape
    x = int(np.clip(u, 0, W - 1)); y = int(np.clip(v, 0, H - 1))
    win = d[max(y - r, 0):y + r + 1, max(x - r, 0):x + r + 1]
    return win[win > 0] * scale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--depth", choices=["sensor", "rel"], default="sensor",
                    help="sensor: each visible joint's depth from the depth map; rel: only the torso's "
                         "distance is measured and the pose model's relative depth does the rest")
    ap.add_argument("--out", default="track.txt")
    args = ap.parse_args()
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = json.load(open(os.path.join(here, "calib", "config.json")))
    meta = json.load(open(os.path.join(args.dir, "meta.json")))
    os.chdir(here)                                       # models are found relative to perception/
    K = Intrinsics(**meta["intrinsics"])
    scale = meta["depth_scale"]
    to_mirror = CameraToMirror(cfg["camera_pose"])
    pose = BlazePose(Backend(cfg.get("compute", {})))
    f_joints, f_dist = PosePredictor(**cfg["filters"]["joints"]), OneEuro(0.5, 0.3)
    lines, last_seen, n_tracked = [], -1e9, 0
    for fr in meta["frames"]:
        i, t = fr["i"], fr["t"]
        frame = cv2.imread(os.path.join(args.dir, f"c_{i:05d}.jpg"))
        d = cv2.imread(os.path.join(args.dir, f"d_{i:05d}.png"), cv2.IMREAD_UNCHANGED)
        H, W = frame.shape[:2]
        res = pose(frame)
        cue = skeleton_distance(res["uv"], res["world"], res["vis"], K) if res else None
        if cue is None:
            if t - last_seen > 1.0:
                f_joints.reset(); f_dist.reset()
            lines.append(f"{i} {t:.4f} 0")
            continue
        uv, world, vis = res["uv"], res["world"], res["vis"]
        samples = []
        for k in TORSO:
            if vis[k] > 0.5:
                w = window(d, scale, *uv[k], 6); w = w[(w > 0.2) & (w < 8.0)]
                if w.size:
                    samples.append(float(np.median(w)))
        z_fused = float(np.median(samples)) + OFF["torso"] if samples else max(cue, 0.3)
        z_root = float(f_dist(np.array([z_fused]), t)[0])
        Z = np.maximum(z_root + (world[:, 2] - world[list(TORSO), 2].mean()), 0.2)
        if args.depth == "sensor":
            for k in range(33):
                if vis[k] > 0.5:
                    w = window(d, scale, *uv[k], 5); w = w[np.abs(w - Z[k]) < 0.35]
                    if w.size >= 4:
                        Z[k] = float(np.percentile(w, 25)) + (OFF["face"] if k < 11 else OFF["torso"] if k in TORSO else OFF["limb"])
        P_c = np.stack([(uv[:, 0] - K.cx) * Z / K.fx, (uv[:, 1] - K.cy) * Z / K.fy, Z], axis=1)
        P_m, _ = f_joints(to_mirror(P_c), t, vis)
        last_seen = t
        n_tracked += 1
        cv2.imwrite(os.path.join(args.dir, f"m_{i:05d}.pgm"), frame_mask(res, W, H, 256))
        lines.append(f"{i} {t:.4f} 1 " + " ".join(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {v:.3f}" for p, v in zip(P_m, vis)))
    pos = cfg["camera_pose"]["pos_m"]
    head = f"{K.fx:.3f} {K.fy:.3f} {K.cx:.3f} {K.cy:.3f} {K.w} {K.h} {pos[0]} {pos[1]} {pos[2]} {len(lines)}"
    open(os.path.join(args.dir, args.out), "w").write("\n".join([head] + lines) + "\n")
    print(f"{n_tracked}/{len(lines)} frames tracked -> {os.path.join(args.dir, args.out)}")


if __name__ == "__main__":
    main()
