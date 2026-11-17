"""How far the camera is pitched down (and rolled), measured from the depth image.

Floors and ceilings are level and walls are upright, so the large flat surfaces in view say which
way is up in the camera's own axes. The three largest planes are fitted to the depth points
(RANSAC); a plane facing up or down is a floor or ceiling, one facing sideways a wall. A floor or
ceiling gives pitch and roll; a wall gives pitch only (it may be turned about the vertical).

    server.py --measure-tilt     prints a line every 2 s:  [tilt] floor: 14.8 deg down, roll 0.3 ...

camera_pose.tilt_down_deg is the pitch relative to the glass: add how far the glass leans back.
"""
import math
import numpy as np


def planes(depth, scale, K, step=6, n_planes=3, iters=250, rng=np.random.default_rng(0)):
    """[(unit normal facing the camera, share of the points, mean distance m)], largest first."""
    d = depth[::step, ::step].astype(np.float32) * scale
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    ok = (d > 0.4) & (d < 6.0)
    z = d[ok]
    P = np.stack([(u[ok] - K.cx) * z / K.fx, (v[ok] - K.cy) * z / K.fy, z], axis=1)
    total, out = len(P), []
    for _ in range(n_planes):
        if len(P) < 200:
            break
        tol = 0.015 + 0.004 * P[:, 2] ** 2               # stereo depth noise grows with distance squared
        best, best_in = None, None
        for _ in range(iters):
            a, b, c = P[rng.choice(len(P), 3, replace=False)]
            n = np.cross(b - a, c - a)
            L = np.linalg.norm(n)
            if L < 1e-6:
                continue
            n /= L
            inl = np.abs((P - a) @ n) < tol
            if best is None or inl.sum() > best:
                best, best_in = inl.sum(), inl
        Q = P[best_in]
        c = Q.mean(0)
        n = np.linalg.svd(Q - c, full_matrices=False)[2][2]      # least-squares normal of the inliers
        if n @ c > 0:
            n = -n                                               # facing the camera
        out.append((n, best / total, float(np.linalg.norm(c))))
        P = P[~best_in]
    return out


def measure(depth, scale, K):
    """[(kind, pitch down deg, roll deg or None, share)] for the planes that are a floor, ceiling or wall.
    Camera axes: x right, y down, z forward. Pitched down by t, up is (0, -cos t, -sin t)."""
    res = []
    for n, share, dist in planes(depth, scale, K):
        if share < 0.06:
            continue
        if n[1] < -0.7:                                          # faces up: floor
            res.append(("floor", math.degrees(math.atan2(-n[2], -n[1])), math.degrees(math.asin(np.clip(n[0], -1, 1))), share))
        elif n[1] > 0.7:                                         # faces down: ceiling
            res.append(("ceiling", math.degrees(math.atan2(n[2], n[1])), math.degrees(math.asin(np.clip(-n[0], -1, 1))), share))
        elif abs(n[1]) < 0.5:                                    # upright: wall
            res.append(("wall", math.degrees(math.atan2(n[1], -n[2])), None, share))
    return res


def report(depth, scale, K):
    r = measure(depth, scale, K)
    if not r:
        return "[tilt] no floor, ceiling or wall found in the depth image"
    return "[tilt] " + "; ".join(f"{k}: {p:+.1f} deg down" + (f", roll {ro:+.1f}" if ro is not None else "") + f" ({s * 100:.0f}% of view)"
                                 for k, p, ro, s in r)
