"""Synthetic bodies for testing without a person: the shape message the depth camera would give
for a few builds (fake_server.py --build), and a depth image of the same body to measure
(tools/bodyshape_selftest.py). Body frame: x across, y up from the hips, z forward of the plane
through the hip and shoulder joints; torso length 0.5 m, shoulder joints 0.36 m apart.
"""
import numpy as np
from bodyshape import LEVELS, GRID_T, GRID_S, GRID_EDGE_T, SH_X

L, SW = 0.5, 0.36
_A = {"slender": [0.140, 0.132, 0.122, 0.128, 0.138, 0.142],
      "average": [0.172, 0.166, 0.158, 0.158, 0.160, 0.162],
      "heavy":   [0.215, 0.225, 0.228, 0.215, 0.200, 0.192]}
_D = {"slender": [0.085, 0.080, 0.074, 0.078, 0.084, 0.086],
      "average": [0.105, 0.103, 0.099, 0.100, 0.105, 0.107],
      "heavy":   [0.165, 0.185, 0.190, 0.172, 0.150, 0.140]}
# a pot belly on an otherwise average frame; a full chest; the same build each time otherwise
_A["belly"] = [0.176, 0.176, 0.172, 0.164, 0.160, 0.162]
_A["chest"] = _A["average"]
_D["belly"] = _D["chest"] = _D["average"]
SHOULDERS = {"normal": [0.074, 0.063, 0.053, 0.045], "sloped": [0.090, 0.068, 0.046, 0.034], "square": [0.062, 0.060, 0.057, 0.052]}
BUILDS = tuple(_A)


def _g(v, c, w):
    return np.exp(-((np.asarray(v, float) - c) / w) ** 2)


class Body:
    def __init__(self, build="average", shoulders="normal"):
        self.build, self.sh = build, np.array(SHOULDERS[shoulders])
        # the arms hang clear of an average body and lie against a heavy one
        el, wr = max(0.24, float(self.a(0.44)) + 0.05), max(0.26, float(self.a(-0.05)) + 0.05)
        self.JOINTS = {11: (-0.18, 0.5), 12: (0.18, 0.5), 13: (-el, 0.22), 14: (el, 0.22), 15: (-wr, -0.03), 16: (wr, -0.03),
                       23: (-0.10, 0.0), 24: (0.10, 0.0)}

    def a(self, t):
        return np.interp(t, LEVELS, _A[self.build])

    def back(self, t):
        """How far the back lies behind the joints' plane."""
        return np.interp(t, LEVELS, _D["slender" if self.build == "slender" else "average"]) * (1.15 if self.build == "heavy" else 1.0)

    def front(self, t, s):
        """How far the front stands forward of the joints' plane at height t (of the torso length)
        and s = x / half-width."""
        t, s = np.asarray(t, float), np.clip(np.abs(np.asarray(s, float)), 0, 1)
        f0 = np.interp(t, LEVELS, _D[self.build])
        cen = (f0 - self.back(t)) / 2
        z = cen + (f0 - cen) * np.sqrt(1 - s * s)
        if self.build == "belly":
            z = z + 0.085 * _g(t, 0.17, 0.24) * _g(s, 0, 0.8) * np.sqrt(1 - s ** 4)
        if self.build == "chest":
            z = z + 0.05 * _g(t, 0.68, 0.10) * _g(s, 0.42, 0.26)
        z = z * self.dome(t)                              # closing in under the shoulder line,
        return np.where(t > 0.88, np.maximum(z, 0.06), z)   # which is a bar 6 cm deep

    @staticmethod
    def dome(t):
        return np.sqrt(np.clip(1 - np.clip((np.asarray(t, float) - 0.85) / 0.2, 0, 1) ** 2, 0, 1))

    def message(self):
        """What bodyshape.BodyShape.message() gives for this body."""
        gt = np.array(GRID_T)
        ga = self.a(np.minimum(gt, GRID_T[max(i for i, t in enumerate(GRID_T) if t <= GRID_EDGE_T)]))
        gz = np.array([[self.front(t, s) for s in GRID_S] for t in gt])
        lv = np.array(LEVELS)
        f0 = self.front(lv, 0)
        return {"a": self.a(lv).round(4).tolist(), "d": ((f0 + self.back(lv)) / 2).round(4).tolist(),
                "ga": ga.round(4).tolist(), "gz": gz.round(4).ravel().tolist(), "sh": self.sh.round(4).tolist()}

    def depth_image(self, W=848, H=480, fx=425.0, dist=1.8, wall=3.4, noise=0.0, rng=None):
        """Depth (m) of the body standing at dist in front of a wall, hips at the image centre + 0.25 m
        down; and the pixel of a body point. Camera looks along +z, v down."""
        cx, cy, y0 = W / 2, H / 2, -0.28
        px = lambda x, y: np.array([cx + fx * x / dist, cy - fx * (y + y0) / dist])
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        out = np.full((H, W), wall)
        z = np.full((H, W), dist)
        for _ in range(2):                                # the surface is nearer than the plane: once more
            x, y = (u - cx) * z / fx, -(v - cy) * z / fx - y0
            t = y / L
            a = self.a(np.clip(t, -0.3, 1.0))
            dome = self.dome(t)
            on = (np.abs(x) < a * (0.6 + 0.4 * dome)) & (t > -0.5) & (t < 1.05)
            z = np.where(on, dist - self.front(np.clip(t, -0.05, 1.0), x / np.maximum(a, 1e-3)), dist)
        out[on] = z[on]
        x, y = (u - cx) * dist / fx, -(v - cy) * dist / fx - y0
        # shoulders up to the measured tops, the neck and head above
        top = np.interp(np.abs(x), np.array(SH_X) * SW / 2, self.sh)
        end = np.sqrt(np.clip(0.045 ** 2 - np.clip(np.abs(x) - SW / 2, 0, None) ** 2, 0, None)) - 0.045
        m = (y > L - 0.06) & (y < L + top + end) & (np.abs(x) < SW / 2 + 0.045)
        out[m] = np.minimum(out[m], dist - 0.06)
        m = ((np.abs(x) < 0.055) & (y > L) & (y < L + 0.2)) | (np.hypot(x, y - (L + 0.27)) < 0.1)
        out[m] = np.minimum(out[m], dist - 0.05)
        # arms hanging at the sides
        for sg in (-1, 1):
            for p, q, r in ((self.JOINTS[12], self.JOINTS[14], 0.045), (self.JOINTS[14], self.JOINTS[16], 0.04)):
                p, q = np.array([sg * p[0], p[1]]), np.array([sg * q[0], q[1]])
                ab = q - p
                k = np.clip(((x - p[0]) * ab[0] + (y - p[1]) * ab[1]) / (ab @ ab), 0, 1)
                dd = np.hypot(x - (p[0] + k * ab[0]), y - (p[1] + k * ab[1]))
                m = dd < r
                out[m] = np.minimum(out[m], dist - np.sqrt(r * r - dd[m] ** 2))
        if noise > 0:
            rng = rng or np.random.default_rng(0)
            out = out + rng.normal(0, noise, out.shape) * (out / dist) ** 2
            out[rng.random(out.shape) < 0.01] = 0
        return out, px
