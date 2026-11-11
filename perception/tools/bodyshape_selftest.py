"""Checks bodyshape.py without a camera: renders a depth image of each synthetic body
(synthbody.py), measures it as the server would, and prints how far the mould, the widths and
the shoulder tops are from the body that was rendered.

    .venv/bin/python tools/bodyshape_selftest.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from bodyshape import BodyShape, GRID_T, GRID_S
import synthbody

W, H, FX, DIST = 848, 480, 425.0, 1.8
worst = 0.0
for build in synthbody.BUILDS:
    for shoulders in (("normal", "sloped", "square") if build == "average" else ("normal",)):
        body, shape, rng = synthbody.Body(build, shoulders), BodyShape(), np.random.default_rng(1)
        for i in range(45):
            depth, px = body.depth_image(W, H, FX, DIST, noise=0.003, rng=rng)
            uv, P_c, vis = np.zeros((33, 2)), np.zeros((33, 3)), np.ones(33)
            for j, (x, y) in body.JOINTS.items():
                uv[j] = px(x, y); P_c[j] = (x, -(y - 0.28), DIST)
            def line(us, vs):
                x, y = np.round(us).astype(int), np.round(vs).astype(int)
                ok = (x >= 0) & (x < W) & (y >= 0) & (y < H)
                out = np.zeros(len(x)); out[ok] = depth[y[ok], x[ok]]
                return out
            msg = shape.update(i / 30, line, uv, P_c, vis, FX, FX)
        want = body.message()
        gz, wz = np.array(msg["gz"]).reshape(len(GRID_T), -1), np.array(want["gz"]).reshape(len(GRID_T), -1)
        ga, wa = np.array(msg["ga"]), np.array(want["ga"])
        got = gz > 0
        ez, ea = np.abs(gz - wz)[got], np.abs(ga - wa)[ga > 0]
        es = np.abs(np.array(msg["sh"]) - np.array(want["sh"]))[np.array(msg["sh"]) > 0]
        worst = max(worst, ez.max() if ez.size else 1.0)
        print(f"{build:8s} {shoulders:7s} cells {got.sum():2d}/{got.size}  front mean {1000 * ez.mean():4.1f} mm max {1000 * ez.max():4.1f} mm"
              f"   width max {1000 * ea.max():4.1f} mm   shoulder tops {np.round(np.array(msg['sh']) * 1000).astype(int).tolist()} "
              f"(true {np.round(np.array(want['sh']) * 1000).astype(int).tolist()})")
print("OK" if worst < 0.012 else "FAILED: the mould is more than 12 mm off somewhere")
