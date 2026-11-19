"""The user's build, measured from the depth camera: how wide and how deep their torso is at a
few heights between the hips and the shoulders. The renderer cuts the garments to it, so the
same shirt hangs differently on a slender person and on a heavy one.

At each level a line of depth samples is read across the torso, perpendicular to the spine in
the image. Along it:

  * the body ends where the depth jumps back to the room, or at the crease between the torso and
    an arm resting against it (a local maximum of depth with a nearer ridge beyond it). That gives
    the half-width a on each side, in metres.
  * the front of the torso curves away from the camera toward its sides. Fitting the front of an
    ellipse, z(x) = zc + k x - d sqrt(1 - (x / a)^2), to the middle of the line gives the
    half-depth d: how far the body's surface stands in front of its centre. k absorbs a turned
    torso. A slender person gives a small, shallow ellipse; a belly gives a wide, deep one.

On top of those six ovals comes a mould of the front of the torso: at GRID_T heights and GRID_S
positions across (fractions of the half-width, both sides averaged, which also cancels a turned
torso), how far the surface stands in front of the plane through the hip and shoulder joints. A
belly, a chest or a flat stomach are in it as they are, not as a deeper oval. And the tops of the
shoulders: at SH_X of the way from the neck out to each shoulder joint, how high the silhouette
rises above the line through the shoulder joints, which is the slope of the shoulders.

Samples are only taken while the user faces the camera, and are averaged over a couple of seconds
per level. Without a depth camera there is nothing to measure and the renderer falls back to the
silhouette of the person mask.
"""
import numpy as np

LEVELS = (-0.05, 0.12, 0.28, 0.44, 0.58, 0.72)        # height / torso length; FBodyShape::Levels
GRID_T = tuple(float(v) for v in np.round(np.linspace(-0.05, 1.0, 16), 4))   # FBodyMould::Rows
GRID_S = (0.0, 0.2, 0.4, 0.6, 0.8)                     # fraction of the half-width; FBodyMould::Cols
GRID_EDGE_T = 0.75                                     # above this the arms join: no silhouette edge to find
SH_X = (0.45, 0.65, 0.85, 1.0)                         # fraction of the half shoulder width; FBodyMould::ShX
L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST, L_HIP, R_HIP = 11, 12, 13, 14, 15, 16, 23, 24


class BodyShape:
    def __init__(self, cfg=None):
        c = {"time_constant_s": 1.5, "max_turn": 0.3, "min_samples": 12, "half_span_m": 0.42,
             "edge_jump_m": 0.10, "crease_m": 0.012, "width_m": [0.09, 0.34], "depth_m": [0.055, 0.26], "arm_radius_m": 0.045}
        c.update(cfg or {})
        self.c = c
        self.reset()

    def reset(self):
        n = len(LEVELS)
        self.a, self.d, self.n = np.zeros(n), np.zeros(n), np.zeros(n, int)
        R, C = len(GRID_T), len(GRID_S)
        self.ga, self.gan = np.zeros(R), np.zeros(R, int)             # half-span of each row's columns
        self.gz, self.gn = np.zeros((R, C)), np.zeros((R, C), int)    # forward offsets (m)
        self.sh, self.shn = np.zeros(len(SH_X)), np.zeros(len(SH_X), int)
        self.last_t = None

    # ------------------------------------------------------------------ one line
    def _edge(self, x, z, z0, sign):
        """Lateral distance (m) from the centre to the body's edge on one side, or None."""
        c = self.c
        idx = np.where(x * sign > 0)[0]
        idx = idx[np.argsort(np.abs(x[idx]))]
        deepest, deepest_x, gap = z0, 0.0, 0
        for k, i in enumerate(idx):
            zi = z[i]
            if zi <= 0:                                   # no depth: typical right at a silhouette edge
                gap += 1
                if gap >= 3:
                    return abs(x[idx[k - gap + 1]]) if k >= gap else None
                continue
            gap = 0
            if zi > deepest + c["edge_jump_m"] or zi < z0 - 0.3:
                return abs(x[i])                          # the room behind / something held in front
            if zi >= deepest:
                deepest, deepest_x = zi, abs(x[i])
            elif deepest - zi > c["crease_m"] and deepest_x > 0.06:
                return deepest_x                          # coming nearer again: an arm beyond a crease
        return None                                       # ran off the line without finding an edge

    def _metres(self, x, z):
        """(lateral position in metres, depth at the centre) of a line of samples, or (None, None)
        if the centre has no depth. x: lateral position per metre of depth (pixels / focal length).
        Each sample is placed at its own depth: the sides of a body are further away than its front."""
        mid = z[(np.abs(x) < 0.03) & (z > 0)]
        if mid.size < 3:
            return None, None
        z0 = float(np.median(mid))
        return x * np.where((z > z0 - 0.3) & (z < z0 + 0.2), z, z0 + 0.08), z0

    def _width(self, x, z, z0, arms):
        """Half-width (m) of the torso, or None. arms: where each arm's centre line crosses this
        line (m from the centre; +, - side), or None. An arm lying against the body hides the
        body's edge behind it: the silhouette then ends at the arm's far side, and the torso is
        taken to end where the arm begins."""
        c = self.c
        ok = []
        for sign, arm in zip((1, -1), arms):
            e = self._edge(x, z, z0, sign)
            if e is None:
                continue
            if arm is not None and e > arm + 0.5 * c["arm_radius_m"]:
                e = arm - c["arm_radius_m"]
            if c["width_m"][0] <= e <= c["width_m"][1]:
                ok.append(e)
        return float(np.mean(ok)) if ok else None

    def _level(self, x, z, arms=(None, None)):
        """(half-width, half-depth) in metres from one line of samples, either may be None."""
        c = self.c
        x, z0 = self._metres(x, z)
        if x is None:
            return None, None
        a = self._width(x, z, z0, arms)
        if a is None:
            return None, None
        # front of an ellipse over the middle 80 %, where no arm or edge noise reaches
        m = (np.abs(x) < 0.8 * a) & (z > 0) & (np.abs(z - z0) < 0.2)
        if m.sum() < 8:
            return a, None
        w = np.sqrt(1.0 - (x[m] / a) ** 2)
        A = np.stack([np.ones(m.sum()), x[m], -w], axis=1)
        sol, *_ = np.linalg.lstsq(A, z[m], rcond=None)
        res = z[m] - A @ sol
        keep = np.abs(res) < max(2.5 * np.std(res), 0.004)   # folds, a pocket, a hand
        if keep.sum() >= 8 and keep.sum() < m.sum():
            sol, *_ = np.linalg.lstsq(A[keep], z[m][keep], rcond=None)
        d = float(sol[2])
        return a, d if c["depth_m"][0] <= d <= c["depth_m"][1] else None

    # ------------------------------------------------------------------ the mould
    @staticmethod
    def _arms(uv, P_c, centre, axis, across, f):
        """Where the left (+) and right (-) arm's centre line crosses the line through centre
        along across: metres from the centre, None if that arm doesn't pass this height."""
        out = []
        for sign, ids in ((1, (L_SHOULDER, L_ELBOW, L_WRIST)), (-1, (R_SHOULDER, R_ELBOW, R_WRIST))):
            hit = None
            for i, j in zip(ids[:-1], ids[1:]):
                ha, hb = (uv[i] - centre) @ axis, (uv[j] - centre) @ axis
                if ha * hb > 0 or ha == hb:
                    continue
                k = ha / (ha - hb)
                lat = sign * float((uv[i] + (uv[j] - uv[i]) * k - centre) @ across)
                zz = float(P_c[i, 2] + (P_c[j, 2] - P_c[i, 2]) * k)
                if lat > 0 and zz > 0.2:
                    hit = lat * zz / f
            out.append(hit)
        return out

    def _shoulder_tops(self, sample_line, hip_px, axis, across, half_sw_px, torso_m, z_sh):
        """Height (m) of the silhouette above the shoulder-joint line at SH_X, nan where not found."""
        ts = np.linspace(0.8, 1.45, 66)
        out = np.full(len(SH_X), np.nan)
        for j, fr in enumerate(SH_X):
            hs = []
            for sg in (-1, 1):
                base = hip_px + across * (sg * fr * half_sw_px)
                z = np.asarray(sample_line(base[0] + axis[0] * ts, base[1] + axis[1] * ts), float)
                surf, last, gap = z_sh, -1, 0
                for i, zi in enumerate(z):
                    if 0 < zi < surf + self.c["edge_jump_m"] and zi > z_sh - 0.4:
                        surf, last, gap = zi, i, 0
                    elif last >= 0:                       # (below the armpit there may be a gap first)
                        gap += 1
                        if gap >= 3:
                            break
                else:
                    last = -1                             # never left the body: hair, a hood, the neck
                if last >= 2:
                    hs.append((ts[last] - 1.0) * torso_m)
            if hs:
                out[j] = float(np.clip(np.mean(hs), 0.0, 0.14))
        return out

    def _update_mould(self, sample_line, hip_px, axis, across, s, f, rate, uv, P_c, fx, fy):
        c = self.c
        # the body's own frame, in the camera's: every depth sample becomes a point in it, so that a
        # belly standing well forward of the joints is entered at the height it really is at, not
        # where its pixel would be on the joints' plane (the camera looks down or up at it)
        torso = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
        cx = float(np.mean(uv[torso, 0] - P_c[torso, 0] * fx / P_c[torso, 2]))
        cy = float(np.mean(uv[torso, 1] - P_c[torso, 1] * fy / P_c[torso, 2]))
        hip, sh = (P_c[L_HIP] + P_c[R_HIP]) / 2, (P_c[L_SHOULDER] + P_c[R_SHOULDER]) / 2
        torso_m = float(np.linalg.norm(sh - hip))
        up = (sh - hip) / torso_m
        right = P_c[L_SHOULDER] - P_c[R_SHOULDER]
        right = right - up * (right @ up)
        right /= np.linalg.norm(right)
        fwd = np.cross(right, up)
        if fwd[2] > 0:
            fwd = -fwd                                    # toward the camera
        T, X, F = [], [], []
        for t in np.linspace(-0.12, 1.08, 49):
            us, vs = hip_px[0] + axis[0] * t + across[0] * s, hip_px[1] + axis[1] * t + across[1] * s
            z = np.asarray(sample_line(us, vs), float)
            ok = z > 0
            P = np.stack([(us[ok] - cx) * z[ok] / fx, (vs[ok] - cy) * z[ok] / fy, z[ok]], axis=1) - hip
            T.append(P @ up / torso_m); X.append(P @ right); F.append(P @ fwd)
        T, X, F = np.concatenate(T), np.concatenate(X), np.concatenate(F)
        body = (F > -0.05) & (F < 0.45)
        T, X, F = T[body], X[body], F[body]

        unit = axis / np.hypot(*axis)
        carried = 0.0                                     # the span of the last row below the armpits, for the rows above
        for k, t in enumerate(GRID_T):
            if t <= GRID_EDGE_T:
                centre = hip_px + axis * t
                z = np.asarray(sample_line(centre[0] + across[0] * s, centre[1] + across[1] * s), float)
                x, z0 = self._metres(s / f, z)
                a = self._width(x, z, z0, self._arms(uv, P_c, centre, unit, across, f)) if x is not None else None
                if a is not None and not (self.gan[k] > 15 and abs(a - self.ga[k]) > 0.25 * self.ga[k]):
                    self.ga[k] = a if self.gan[k] == 0 else self.ga[k] + rate * (a - self.ga[k])
                    self.gan[k] += 1
                if self.gan[k] > 0:
                    carried = self.ga[k]
            elif carried > 0:
                self.ga[k] = carried
                self.gan[k] += 1
            if self.gan[k] == 0:
                continue
            a = self.ga[k]
            row = np.abs(T - t) < 0.03
            x, fw = X[row], F[row]
            for j, sc in enumerate(GRID_S):
                # both sides of a column, so that a turned torso cancels out
                sides = [float(np.median(fw[m])) for m in (np.abs(x - sg * sc * a) < 0.08 * a for sg in ((1,) if sc == 0 else (-1, 1))) if m.sum() >= 4]
                if len(sides) != (1 if sc == 0 else 2):
                    continue
                v = float(np.mean(sides))
                if self.gn[k, j] > 15 and abs(v - self.gz[k, j]) > 0.04:
                    continue                              # a hand, a bag, a phone held in front
                self.gz[k, j] = v if self.gn[k, j] == 0 else self.gz[k, j] + rate * (v - self.gz[k, j])
                self.gn[k, j] += 1
        tops = self._shoulder_tops(sample_line, hip_px, axis, across, 0.5 * float(np.hypot(*(uv[L_SHOULDER] - uv[R_SHOULDER]))), torso_m, float(sh[2]))
        for j, v in enumerate(tops):
            if np.isfinite(v):
                self.sh[j] = v if self.shn[j] == 0 else self.sh[j] + rate * (v - self.sh[j])
                self.shn[j] += 1

    # ------------------------------------------------------------------ frame
    def update(self, t, sample_line, uv, P_c, vis, fx, fy):
        """sample_line(us, vs) -> depths (m, 0 = none) at frame pixels; uv: (33, 2) pixels;
        P_c: (33, 3) joints in the camera frame (m). Returns the message for the renderer."""
        c = self.c
        if min(vis[L_SHOULDER], vis[R_SHOULDER]) < 0.6 or min(vis[L_HIP], vis[R_HIP]) < 0.5:
            return self.message()
        sh_w = float(np.linalg.norm(P_c[L_SHOULDER] - P_c[R_SHOULDER]))
        if sh_w < 0.2 or abs(P_c[L_SHOULDER, 2] - P_c[R_SHOULDER, 2]) / sh_w > c["max_turn"]:
            return self.message()                         # turned: the silhouette is not the width
        hip_px, sh_px = (uv[L_HIP] + uv[R_HIP]) / 2, (uv[L_SHOULDER] + uv[R_SHOULDER]) / 2
        axis = sh_px - hip_px
        if np.hypot(*axis) < 20:
            return self.message()
        across = np.array([-axis[1], axis[0]]) / np.hypot(*axis)
        if across @ (uv[L_SHOULDER] - uv[R_SHOULDER]) < 0:
            across = -across
        z_ref = float(np.median(P_c[[L_SHOULDER, R_SHOULDER, L_HIP, R_HIP], 2]))
        f = 0.5 * (fx + fy)
        px_per_m = f / max(z_ref - 0.1, 0.3)                  # at the torso's front surface, roughly
        half = c["half_span_m"] * px_per_m
        n = int(np.clip(2 * half, 40, 400))
        s = np.linspace(-half, half, n)
        rate = 1.0 if self.last_t is None else 1.0 - np.exp(-min(t - self.last_t, 0.5) / c["time_constant_s"])
        self.last_t = t
        for k, lv in enumerate(LEVELS):
            centre = hip_px + axis * lv
            z = np.asarray(sample_line(centre[0] + across[0] * s, centre[1] + across[1] * s), float)
            a, d = self._level(s / f, z, self._arms(uv, P_c, centre, axis / np.hypot(*axis), across, f))
            if a is None:
                continue
            first = self.n[k] == 0
            if not first and self.n[k] > 15 and abs(a - self.a[k]) > 0.25 * self.a[k]:
                continue                                  # a bag, an elbow out, someone walking past
            self.a[k] = a if first else self.a[k] + rate * (a - self.a[k])
            if d is not None:
                self.d[k] = d if self.d[k] == 0 else self.d[k] + rate * (d - self.d[k])
            self.n[k] += 1
        self._update_mould(sample_line, hip_px, axis, across, s, f, rate, uv, P_c, fx, fy)
        return self.message()

    def message(self):
        """Half-width a and half-depth d (m) per level; the mould: ga, the half-span of each row's
        columns, gz, how far the front stands forward of the joints' plane (m, row by row), and sh,
        the shoulder tops (m above the shoulder joints). 0 where not measured (yet)."""
        m = self.c["min_samples"]
        ok = self.n >= m
        gok = self.gn >= m
        return {"a": np.where(ok, self.a, 0).round(4).tolist(), "d": np.where(ok, self.d, 0).round(4).tolist(),
                "ga": np.where(gok[:, 0], self.ga, 0).round(4).tolist(),
                "gz": np.where(gok, self.gz, 0).round(4).ravel().tolist(),
                "sh": np.where(self.shn >= m, self.sh, 0).round(4).tolist()}

    def hip_offset(self, hip_half_m, default):
        """Depth from the body's surface at the hip joints back to their centre, once measured."""
        if self.n[1] < self.c["min_samples"] or self.d[1] <= 0 or self.a[1] <= 0:
            return default
        off = self.d[1] * np.sqrt(max(1.0 - (hip_half_m / self.a[1]) ** 2, 0.25))
        return float(np.clip(off, default - 0.04, default + 0.07))
