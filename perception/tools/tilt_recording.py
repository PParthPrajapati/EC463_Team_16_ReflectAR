"""Re-shoot a tools/record_spin.py recording as a camera pitched down would have seen it.

The mirror's camera sits on top of the mirror and looks down at the user; the recordings were
made with a level camera. Turning a camera about its own optical centre moves no point relative
to it, so the tilted camera's colour image and depth map follow exactly from the level ones (a
homography of the image; each depth scaled by its ray's z in the level camera). Only the viewing
direction changes, not the viewpoint: this tests that a tilt is handled, not what a camera that
is also higher up would see of the shoulders.

--scale < 1 gives the tilted camera a shorter lens, so the person stays in the picture although
it is turned away from them (the part the level camera did not see is black, with no depth).

Run from perception/:
    .venv/bin/python tools/tilt_recording.py ../test_runs/spin_... ../test_runs/spin_tilt20 --tilt 20 --scale 0.5 --frames 60:330
then play it with a config whose camera_pose.tilt_down_deg is the same angle:
    .venv/bin/python server.py --recording ../test_runs/spin_tilt20 --config <config with the tilt>
"""
import argparse, json, math, os
import numpy as np
import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--tilt", type=float, default=20.0, help="degrees the new camera is pitched down")
    ap.add_argument("--scale", type=float, default=0.5, help="new focal length / recorded focal length")
    ap.add_argument("--frames", metavar="A:B", help="only these frame indices")
    args = ap.parse_args()
    meta = json.load(open(os.path.join(args.src, "meta.json")))
    K = meta["intrinsics"]
    W, H = K["w"], K["h"]
    fx, fy = K["fx"] * args.scale, K["fy"] * args.scale
    th = math.radians(args.tilt)
    c, s = math.cos(th), math.sin(th)
    # a ray of the tilted camera, in the level camera (geometry.CameraToMirror's tilt: y down, z forward)
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    x, y = (u - K["cx"]) / fx, (v - K["cy"]) / fy
    dy, dz = c * y + s, -s * y + c
    ok = dz > 1e-3
    mx = np.where(ok, K["fx"] * x / np.where(ok, dz, 1) + K["cx"], -1).astype(np.float32)
    my = np.where(ok, K["fy"] * dy / np.where(ok, dz, 1) + K["cy"], -1).astype(np.float32)
    os.makedirs(args.dst, exist_ok=True)
    frames = meta["frames"]
    if args.frames:
        a, b = (int(v) if v else None for v in args.frames.split(":"))
        frames = [f for f in frames if (a is None or f["i"] >= a) and (b is None or f["i"] <= b)]
    for f in frames:
        i = f["i"]
        color = cv2.imread(os.path.join(args.src, f"c_{i:05d}.jpg"))
        depth = cv2.imread(os.path.join(args.src, f"d_{i:05d}.png"), cv2.IMREAD_UNCHANGED)
        color = cv2.remap(color, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        d = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT).astype(np.float32)
        d = np.where(ok, d / np.where(ok, dz, 1), 0)     # depth along the tilted camera's axis
        cv2.imwrite(os.path.join(args.dst, f"c_{i:05d}.jpg"), color, [cv2.IMWRITE_JPEG_QUALITY, 94])
        cv2.imwrite(os.path.join(args.dst, f"d_{i:05d}.png"), np.clip(d + 0.5, 0, 65535).astype(np.uint16), [cv2.IMWRITE_PNG_COMPRESSION, 1])
    meta["intrinsics"] = {**K, "fx": fx, "fy": fy}
    meta["frames"] = frames
    meta["desc"] = f"{meta.get('desc', '')} re-shot pitched down {args.tilt:g} deg, lens x{args.scale:g}"
    json.dump(meta, open(os.path.join(args.dst, "meta.json"), "w"))
    print(f"wrote {len(frames)} frames to {args.dst}")


if __name__ == "__main__":
    main()
