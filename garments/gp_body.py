"""Reference mannequin for draping: the renderer's canonical rest body (body coordinates, metres:
origin between the hip joints, +X wearer's right, +Y up, +Z toward the wearer's front).

Torso: FRestSkeleton::BodyModel (GarmentMesher.cpp) with HipW 0.22, SW 0.36, U 1: an elliptical
cross-section a(t) x d(t) at t = y / 0.5, rounded off above the chest into a dome under the
shoulder line (as the cloth collider in ClothGarment.cpp does, but starting higher, so this
mannequin contains that collider). Capsules as in FClothGarment::BuildColliders: shoulder line
r 0.05, upper arms r 0.043, forearms r 0.036, neck r 0.05; plus a head and hands for the drape.
"""
import numpy as np

L = 0.5
SW = 0.36
HIPW = 0.22
ARM_DROP = np.radians(40.0)
UPPER, FORE = 0.29, 0.26
SH = {+1: np.array([0.18, 0.50, 0.0]), -1: np.array([-0.18, 0.50, 0.0])}


def arm_dir(side):
    return np.array([side * np.cos(ARM_DROP), -np.sin(ARM_DROP), 0.0])


def arm_up(side):
    """Unit normal to the arm pointing up/outward (the top of the arm)."""
    return np.array([side * np.sin(ARM_DROP), np.cos(ARM_DROP), 0.0])


EL, WR = {}, {}
HEAD_C = np.array([0.0, 0.79, 0.01])
HEAD_R = 0.095
NECK = (np.array([0.0, 0.55, 0.0]), np.array([0.0, 0.70, 0.0]), 0.05)


def _pw(t, v):
    k = np.array([-0.3, 0.0, 0.3, 0.7, 1.0])
    return np.interp(t, k, v)


def body_model_raw(t):
    """BodyModel(t, 0.22, 0.36, 1): half-width a and half-depth d of the torso section."""
    hipA = HIPW * 0.68 + 0.02
    chestA = SW * 0.5 * 0.9
    waistA = (hipA + (chestA - hipA) * 0.45) * 0.93
    a = _pw(t, [hipA * 1.04, hipA, waistA, chestA, chestA * 0.92])
    d = _pw(t, [0.11, 0.105, 0.098, 0.108, 0.075])
    return a, d


# The model is piecewise linear in t; its kinks (chest line, waist) would print as creases in
# fabric lying on them, so the mannequin uses it smoothed (Gaussian, sigma 0.06 in t: <= 3 mm).
_T = np.linspace(-1.0, 2.0, 1501)
_K = np.exp(-0.5 * (np.arange(-60, 61) * (_T[1] - _T[0]) / 0.06) ** 2)
_K /= _K.sum()
_A = np.convolve(np.pad(body_model_raw(_T)[0], 60, mode="edge"), _K, mode="valid")
_D = np.convolve(np.pad(body_model_raw(_T)[1], 60, mode="edge"), _K, mode="valid")


def body_model(t):
    return np.interp(t, _T, _A), np.interp(t, _T, _D)


DOME_T0, DOME_T1 = 0.8, 1.05


def torso_radii(y):
    """Mannequin torso half-width/half-depth at height y (0 above the dome)."""
    t = np.asarray(y, dtype=float) / L
    a, d = body_model(t)
    cap = np.clip((t - DOME_T0) / (DOME_T1 - DOME_T0), 0.0, 1.0)
    dome = np.sqrt(np.maximum(1.0 - cap * cap, 0.0))
    a = a * (0.5 + 0.5 * dome)
    d = d * dome
    return a, d


TORSO_Y0 = -0.35


def capsules():
    caps = [(SH[-1], SH[1], 0.05), NECK]
    for s in (1, -1):
        caps.append((SH[s], EL[s], 0.043))
        caps.append((EL[s], WR[s], 0.036))
        # hand: wider than the wrist, so a cuff can't slide off
        caps.append((WR[s] + arm_dir(s) * 0.045, WR[s] + arm_dir(s) * 0.12, 0.045))
    return caps


CAPS = []


def set_arm_drop(deg):
    """Pose the arms (degrees below horizontal). The reference pose is 40; the drape starts
    higher and lowers the arms into it, as a garment is put on."""
    global ARM_DROP, CAPS
    ARM_DROP = np.radians(deg)
    for s in (1, -1):
        EL[s] = SH[s] + arm_dir(s) * UPPER
        WR[s] = EL[s] + arm_dir(s) * FORE
    CAPS[:] = capsules()


set_arm_drop(40.0)


def _seg_closest(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return a + t[:, None] * ab


def sdf(p):
    """Approximate signed distance (m) to the mannequin and the outward unit normal there."""
    p = np.asarray(p, dtype=float)
    n = len(p)
    best = np.full(n, 1e9)
    nrm = np.zeros((n, 3))
    # torso: first-order distance to the ellipse at the point's height
    y = p[:, 1]
    a, d = torso_radii(y)
    ok = (d > 0.004) & (y > TORSO_Y0)
    if ok.any():
        aa, dd = a[ok], d[ok]
        x, z = p[ok, 0], p[ok, 2]
        q = np.sqrt(x * x / (aa * aa) + z * z / (dd * dd)) + 1e-12
        gx, gz = x / (aa * aa) / q, z / (dd * dd) / q
        gl = np.sqrt(gx * gx + gz * gz) + 1e-12
        dist = (q - 1.0) / gl
        # near the axis the gradient vanishes: use a radial estimate
        dist = np.where(q < 0.3, -(np.minimum(aa, dd)) * (1 - q), dist)
        # dome top: points above the section are measured to the top of the dome too
        idx = np.nonzero(ok)[0]
        better = dist < best[idx]
        best[idx[better]] = dist[better]
        nn = np.stack([gx / gl, np.zeros_like(gx), gz / gl], 1)
        nrm[idx[better]] = nn[better]
    for (ca, cb, r) in CAPS:
        c = _seg_closest(p, ca, cb)
        v = p - c
        dl = np.linalg.norm(v, axis=1) + 1e-12
        dist = dl - r
        better = dist < best
        best[better] = dist[better]
        nrm[better] = v[better] / dl[better, None]
    hv = p - HEAD_C
    hl = np.linalg.norm(hv, axis=1) + 1e-12
    dist = hl - HEAD_R
    better = dist < best
    best[better] = dist[better]
    nrm[better] = hv[better] / hl[better, None]
    return best, nrm


def chest_circumference(y=0.35):
    a, d = torso_radii(y)
    return ellipse_perimeter(a, d)


def ellipse_perimeter(a, d):
    h = ((a - d) / (a + d)) ** 2
    return np.pi * (a + d) * (1 + 3 * h / (10 + np.sqrt(4 - 3 * h)))


# ------------------------------------------------------------------ mannequin mesh (previews)
def _ring_mesh(rings):
    """Stack of closed rings (K x M x 3) -> triangles (outward for CCW-from-above rings)."""
    K, M, _ = rings.shape
    V = rings.reshape(-1, 3)
    F = []
    for k in range(K - 1):
        for m in range(M):
            a, b = k * M + m, k * M + (m + 1) % M
            c, d = a + M, b + M
            F += [(a, b, d), (a, d, c)]
    return V, np.array(F)


def _capsule_mesh(a, b, r, M=24, K=24):
    ax = b - a
    ln = np.linalg.norm(ax)
    w = ax / ln
    u = np.cross(w, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(w, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(w, u)
    rings = []
    th = np.linspace(0, 2 * np.pi, M, endpoint=False)
    for k in range(K + 1):
        phi = -np.pi / 2 + np.pi * k / K
        rad = r * np.cos(phi) + 1e-5
        bases = [0.0] if k < K // 2 else ([0.0, ln] if k == K // 2 else [ln])
        for base in bases:
            off = base + r * np.sin(phi)
            rings.append(a + w * off + rad * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v))
    return _ring_mesh(np.array(rings))


def mannequin_mesh():
    parts = []
    ys = np.linspace(-0.3, L * DOME_T1 - 1e-4, 60)
    th = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    rings = []
    for y in ys:
        a, d = torso_radii(y)
        rings.append(np.stack([a * np.cos(th), np.full_like(th, y), d * np.sin(th)], 1))
    parts.append(_ring_mesh(np.array(rings)))
    for (ca, cb, r) in CAPS:
        parts.append(_capsule_mesh(ca, cb, r))
    parts.append(_capsule_mesh(HEAD_C - [0, 0.02, 0], HEAD_C + [0, 0.02, 0], HEAD_R))
    V, F, off = [], [], 0
    for v, f in parts:
        V.append(v)
        F.append(f + off)
        off += len(v)
    return np.concatenate(V), np.concatenate(F)
