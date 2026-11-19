"""ReflectAR garment generator: drafts, sews and drapes the catalogue tops and writes them as FBX.

    perception/.venv/bin/python garments/make_garments.py [names...] [--preview DIR] [--work DIR]

Pipeline (deterministic: fixed seeds, no threads):
 1. Draft (gp_garments.py): flat pattern pieces for a medium adult fitted to the reference
    mannequin (gp_body.py, the renderer's canonical rest body): front/back bodice, set-in or
    dropped sleeves with the cap solved to the armhole length, and strips (neck band, collar
    stand + fall, hem band, cuffs). Pattern coordinates are metres of fabric = UV0, warp along v.
 2. Mesh (gp_pattern.py): every edge resampled at the target edge length (the two sides of a
    seam get the same count, so a short rib band gathers the longer body edge), each piece
    triangulated with Triangle (quality, max area, internal lines for centre front / placket /
    yoke). The edge length is raised until the welded garment has <= MAX_VERTS vertices.
 3. Place: pieces wrapped around the mannequin with its arms raised to 25 degrees (bodice on a
    hanging envelope of the torso, sleeves as tubes around the arms, long sleeves pushed up the
    arm so their extra length stacks above the cuff, strips continuing off the edge they are
    sewn to, the collar fall folded down over its stand).
 4. Sew (gp_sim.py): zero-gravity cloth simulation pulls the seam pairs together; then weld.
 5. Drape ("dressing"): gravity, the pattern's rest lengths (compression a little softer than
    stretch, so ease buckles), flat bending per fabric, mannequin collision with friction (rib
    bands grip), vertex-vertex and vertex-triangle self-collision. The arms come down to the
    40 degree rest pose, a temporary inward pull smooths the garment onto the body (its ease
    buckles into folds sized by the bending stiffness) and is released, and the garment settles.
 6. Finish: Taubin smoothing of mesh-scale noise (boundary loops along themselves, so neck, hem
    and cuff edges stay clean), body cleared, every piece oriented to face outward; vertex
    colours R = pin weight (where the body carries the garment), G = elastic band, B = ribbed
    knit, A = 1; checks (clearance, manifold, duplicates, strain, self-intersection).
 7. Export (gp_fbx.py): ASCII FBX 7.4, ONE mesh, one material. Axis/unit convention: Z-up,
    front -Y, right-handed, centimetres; fbx = (100 z, -100 x, 100 y) for a canonical point
    (x right, y up, z front, metres). Unreal imports it with only its Y flip, so an imported
    vertex equals ReflectAR::BodyToUE(canonical) and GarmentAsset.cpp maps it back with UEToBody.
    (The importer also stores UV V as 1 - V; the loader undoes that.)

Takes about 5 minutes per garment. Re-import into Unreal with Content/Python/reflectar_garments.py.
"""
import argparse
import os
import sys
import time

import numpy as np
from scipy.sparse.csgraph import dijkstra
import scipy.sparse as sp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gp_body  # noqa: E402
import gp_fbx  # noqa: E402
import gp_garments  # noqa: E402
from gp_pattern import assign_counts  # noqa: E402
from gp_sim import Cloth  # noqa: E402

MAX_VERTS = 8900
H0 = 0.0105


# ============================================================================ placement
def _quarter_ellipse(A, D, n=400):
    phi = np.linspace(0, np.pi / 2, n)
    x, z = A * np.sin(phi), D * np.cos(phi)
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(z)))])
    return s, x, z


def _ellipse_A(D, perim):
    lo, hi = 0.0, 1.0
    for _ in range(50):
        m = 0.5 * (lo + hi)
        if gp_body.ellipse_perimeter(m, D) < perim:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def place_bodice(pan, UV, front, clear=0.03):
    hem_y = pan.place["hem_y"]
    ys = np.arange(hem_y - 0.02, 0.66, 0.002)
    a, d = gp_body.torso_radii(ys)
    sh = np.sqrt(np.maximum(0.05 ** 2 - (ys - 0.5) ** 2, 0))
    zb = np.maximum(d, sh)
    zenv = np.maximum.accumulate(zb[::-1])[::-1] + clear      # hangs straight down from above
    # profile in (z, y), then a turn over the top toward the centre plane
    prof = np.stack([zenv, ys], 1)
    top = prof[-1]
    turn = np.stack([np.linspace(top[0], 0.005, 20), np.full(20, top[1])], 1)[1:]
    prof = np.vstack([prof, turn])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(prof, axis=0), axis=1))])
    s0 = np.interp(hem_y, ys, s[:len(ys)])
    u, v = UV[:, 0], UV[:, 1]
    sv = s0 + v
    zprof = np.interp(sv, s, prof[:, 0])
    yprof = np.interp(sv, s, prof[:, 1])
    side = pan.edge("side_r")
    Wv = np.interp(v, side[:, 1], side[:, 0], left=side[0, 0], right=side[-1, 0])
    X = np.zeros((len(UV), 3))
    for i in range(len(UV)):
        D = max(zprof[i], 0.004)
        A = max(_ellipse_A(D, 4 * Wv[i]), gp_body.torso_radii(yprof[i])[0] + clear)
        sq, xq, zq = _quarter_ellipse(A, D, 120)
        au = abs(u[i])
        if au <= sq[-1]:
            x, z = np.interp(au, sq, xq), np.interp(au, sq, zq)
        else:
            x, z = A, -(au - sq[-1])
        X[i] = (np.sign(u[i]) * x, yprof[i], z if front else -z)
    return X


def place_sleeve(pan, UV):
    side = pan.side
    pl = pan.place
    B, Wo, SL, capH = pl["B"], pl["Wo"], pl["SL"], pl["capH"]
    S = gp_body.SH[1]
    a, n1, n2 = gp_body.arm_dir(1), gp_body.arm_up(1), np.array([0, 0, 1.0])
    u, v = UV[:, 0], UV[:, 1]
    l = (SL - v) + 0.005
    if "l_hem" in pl and pl["l_hem"] < SL:
        # start pushed up the arm: the extra length stacks into folds above the cuff
        k0 = 0.22
        l = np.where(l > k0, k0 + (l - k0) * (pl["l_hem"] - k0) / (SL + 0.005 - k0), l)
    vb = SL - capH
    w = np.where(v < vb, Wo + (B - Wo) * np.clip(v / vb, 0, 1), B)
    rarm = np.where(l < gp_body.UPPER, 0.043, 0.036)
    r = np.maximum(w / (2 * np.pi), rarm + 0.022)
    th = u / r
    X = S + a * l[:, None] + r[:, None] * (np.cos(th)[:, None] * n1 + np.sin(th)[:, None] * n2)
    if side < 0:
        X[:, 0] *= -1
    return X


def chain_polyline(chain, edge_pos):
    pts = None
    for i, (pn, en, rev) in enumerate(chain):
        q = edge_pos[(pn, en)]
        if i == 0:
            q = q[::-1] if rev else q
            pts = q
        else:
            if np.linalg.norm(q[-1] - pts[-1]) < np.linalg.norm(q[0] - pts[-1]):
                q = q[::-1]
            pts = np.vstack([pts, q[1:]])
    return pts


def place_strip(pan, UV, chain_pts):
    pl = pan.place
    Lb, Hh = pl["length"], pl["height"]
    seg = np.linalg.norm(np.diff(chain_pts, axis=0), axis=1)
    cs = np.concatenate([[0], np.cumsum(seg)])
    cs /= cs[-1]
    s = np.clip((Lb - UV[:, 0]) / Lb, 0, 1)
    P = np.stack([np.interp(s, cs, chain_pts[:, k]) for k in range(3)], 1)
    t = (Hh - UV[:, 1])[:, None]
    up = np.array([0, 1.0, 0])
    radial = P * np.array([1, 0, 1.0])
    radial /= np.linalg.norm(radial, axis=1, keepdims=True) + 1e-9
    d = pl["dir"]
    if d == "up":
        return P + up * t - radial * 0.25 * t
    if d == "down":
        return P - up * t
    if d == "arm":
        ad = gp_body.arm_dir(pl["side"])
        return P + ad * t
    if d == "fold":
        # turned down over the stand: a tight fold at the top, then down and out over the yoke
        rf = 0.004
        tf = np.pi * rf
        phi = np.clip(t / rf, 0, np.pi)
        X = P + radial * rf * (1 - np.cos(phi)) + up * rf * np.sin(phi)
        beyond = np.maximum(t - tf, 0)
        return X + radial * 0.6 * beyond - up * 0.8 * beyond
    raise ValueError(d)


# ============================================================================ build
class UF:
    def __init__(self, n):
        self.p = np.arange(n)

    def find(self, a):
        r = a
        while self.p[r] != r:
            r = self.p[r]
        while self.p[a] != r:
            self.p[a], a = r, self.p[a]
        return r

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[max(ra, rb)] = min(ra, rb)


ARM_START = 25.0      # the drape starts with the arms raised and lowers them to the rest pose


def build(key, h, log):
    g = gp_garments.GARMENTS[key]
    gp_body.set_arm_drop(ARM_START)
    panels, seams, strips, g2 = gp_garments.draft(g)
    assign_counts(panels, seams, h)
    names = list(panels)
    off, UVs, Fs, pid = {}, [], [], []
    n = 0
    for k, nm in enumerate(names):
        pan = panels[nm]
        V, F = pan.triangulate(h)
        off[nm] = n
        UVs.append(V)
        Fs.append(F + n)
        pid.append(np.full(len(V), k))
        n += len(V)
    UV = np.vstack(UVs)
    F = np.vstack(Fs)
    pid = np.concatenate(pid)
    ftri_pid = pid[F[:, 0]]
    # ---- placement
    X = np.zeros((n, 3))
    edge_pos = {}
    strip_chain = dict(strips)
    for nm in names:
        pan = panels[nm]
        sl = slice(off[nm], off[nm] + len(pan.uv))
        if pan.kind in ("front", "back"):
            X[sl] = place_bodice(pan, pan.uv, pan.kind == "front")
        elif pan.kind == "sleeve":
            X[sl] = place_sleeve(pan, pan.uv)
        else:
            X[sl] = place_strip(pan, pan.uv, chain_polyline(strip_chain[nm], edge_pos))
        for en, ids in pan.edge_ids.items():
            edge_pos[(nm, en)] = X[np.array(ids) + off[nm]]
    # ---- seams
    pairs, fold_pairs = [], []
    uf = UF(n)
    for s in seams:
        (pa, ea), (pb, eb) = s.a, s.b
        A = np.array(panels[pa].edge_ids[ea]) + off[pa]
        B = np.array(panels[pb].edge_ids[eb]) + off[pb]
        assert len(A) == len(B), (s.a, s.b)
        fwd = np.linalg.norm(X[A[0]] - X[B[0]]) + np.linalg.norm(X[A[-1]] - X[B[-1]])
        bwd = np.linalg.norm(X[A[0]] - X[B[-1]]) + np.linalg.norm(X[A[-1]] - X[B[0]])
        if bwd < fwd:
            B = B[::-1]
        P = np.stack([A, B], 1)
        pairs.append(P)
        if s.kind == "fold":
            fold_pairs.append(P)
        for a_, b_ in P:
            uf.union(a_, b_)
    pairs = np.vstack(pairs)
    root = np.array([uf.find(i) for i in range(n)])
    uniq, wid = np.unique(root, return_inverse=True)
    m = len(uniq)
    log("  %s: h=%.4f  pieces=%d  render verts=%d  welded=%d  tris=%d" % (key, h, len(names), n, m, len(F)))
    return dict(g=g, g2=g2, panels=panels, names=names, off=off, UV=UV, F=F, pid=pid, ftri_pid=ftri_pid,
                X0=X, pairs=pairs, fold_pairs=np.vstack(fold_pairs) if fold_pairs else np.zeros((0, 2), int),
                wid=wid, m=m, strips=strips)


def vertex_props(b):
    """Per render-vertex material properties from the piece and its pattern position."""
    panels, names, off = b["panels"], b["names"], b["off"]
    n = len(b["UV"])
    stretch = np.ones(n)
    bend = np.ones(n)
    rib = np.zeros(n)
    elastic = np.zeros(n)
    collar = np.zeros(n)
    fall = np.zeros(n)
    cf = np.zeros(n, bool)
    for nm in names:
        pan = panels[nm]
        sl = slice(off[nm], off[nm] + len(pan.uv))
        u, v = pan.uv[:, 0], pan.uv[:, 1]
        stretch[sl] = pan.stretch
        bend[sl] = pan.bend
        rib[sl] = 1.0 if pan.rib else 0.0
        elastic[sl] = 1.0 if pan.elastic else 0.0
        if pan.kind in ("collar", "fall"):
            collar[sl] = 1
        if pan.kind == "fall":
            fall[sl] = 1
        hf = pan.tags.get("hemfold", 0.0)
        if hf > 0:
            bend[sl] = np.where(v < hf, np.maximum(bend[sl], 3.0), bend[sl])
        if pan.kind == "front":
            cf[sl] = np.abs(u) < 1e-9
            pw = pan.tags.get("placket", 0.0)
            if pw > 0:
                bend[sl] = np.where(np.abs(u) <= pw + 1e-6, 3.0, bend[sl])
        if pan.kind in ("band", "collar") and pan.place.get("closed"):
            pass
    return stretch, bend, rib, elastic, collar, fall, cf


def skip_springs(b, cos_max=-0.9):
    """Pairs (a, b) of neighbours of a vertex v that lie on a straight line through v in the
    pattern: a distance constraint between them resists bending over two edges (larger folds)."""
    out, rest = [], []
    for nm in b["names"]:
        pan = b["panels"][nm]
        o = b["off"][nm]
        V, F = pan.uv, pan.tris
        nb = [set() for _ in range(len(V))]
        for t in F:
            for i in range(3):
                nb[t[i]].update((t[(i + 1) % 3], t[(i + 2) % 3]))
        for v in range(len(V)):
            ns = sorted(nb[v])
            if len(ns) < 2:
                continue
            d = V[ns] - V[v]
            d /= np.linalg.norm(d, axis=1, keepdims=True)
            c = d @ d.T
            ii, jj = np.nonzero(np.triu(c < cos_max, 1))
            for i, j in zip(ii, jj):
                out.append((ns[i] + o, ns[j] + o))
                rest.append(np.linalg.norm(V[ns[i]] - V[ns[j]]))
    P = np.array(out, int).reshape(-1, 2)
    P = np.sort(P, axis=1)
    P, u = np.unique(P, axis=0, return_index=True)
    return P, np.array(rest)[u]


def rest_lengths(UV, F, keymap=None):
    rl = {}
    for a, b in ((0, 1), (1, 2), (2, 0)):
        i, j = F[:, a], F[:, b]
        d = np.linalg.norm(UV[i] - UV[j], axis=1)
        wi, wj = (i, j) if keymap is None else (keymap[i], keymap[j])
        for p, q, l in zip(wi, wj, d):
            k = (int(min(p, q)), int(max(p, q)))
            rl[k] = min(rl.get(k, 1e9), float(l))
    return rl


def drape(b, log, steps_sew=500, steps_drape=900, preview_cb=None):
    g = b["g"]
    fab = g["fabric"]
    UV, F, X0 = b["UV"], b["F"], b["X0"]
    n = len(UV)
    stretch, bend, rib, elastic, collar, fall, cf = vertex_props(b)
    rng = np.random.default_rng(1234)
    # ---------------- phase A: sew (no gravity)
    rl = rest_lengths(UV, F)
    kb = fab["bend"] * bend
    cl = Cloth(X0 + rng.normal(0, 0.0005, X0.shape), F, rl, np.minimum(stretch, 1.0), np.minimum(kb, 1.0),
               np.ones(n), thickness=0.005)
    cl.seams = b["pairs"]
    cl.compress = 0.5
    cl.set_exclusion(b["wid"], rings=3)
    cl.self_r = 0.0
    cl.vt_r = 0.02
    dt = 1 / 200
    t0 = time.time()
    for k in range(steps_sew):
        cl.seam_k = min(1.0, 0.1 + k / 150)
        cl.step(dt, gravity=0.0, iters=8, damping=0.1, seam_cap=0.003)
        if k > 150:
            gap = np.linalg.norm(cl.x[b["pairs"][:, 0]] - cl.x[b["pairs"][:, 1]], axis=1).max()
            if gap < 5e-4:
                break
    gap = np.linalg.norm(cl.x[b["pairs"][:, 0]] - cl.x[b["pairs"][:, 1]], axis=1).max()
    log("  initial self-intersections %d" % self_intersections(X0, F))
    log("  sewn in %d steps (%.0fs), max seam gap %.2f mm" % (k + 1, time.time() - t0, gap * 1000))
    # ---------------- weld
    wid, m = b["wid"], b["m"]
    Xw = np.zeros((m, 3))
    cnt = np.zeros(m)
    np.add.at(Xw, wid, cl.x)
    np.add.at(cnt, wid, 1)
    Xw /= cnt[:, None]
    Fw = wid[F]
    good = (Fw[:, 0] != Fw[:, 1]) & (Fw[:, 1] != Fw[:, 2]) & (Fw[:, 0] != Fw[:, 2])
    assert good.all(), "degenerate after weld: %d" % (~good).sum()

    def wmax(a):
        o = np.zeros(m)
        np.maximum.at(o, wid, a)
        return o

    def wmin(a):
        o = np.full(m, 1e9)
        np.minimum.at(o, wid, a)
        return o

    rlw = rest_lengths(UV, F, keymap=wid)
    st_w, bd_w = wmin(stretch), wmax(bend)
    # no bending stiffness across the collar fold line
    fold_v = np.zeros(m, bool)
    if len(b["fold_pairs"]):
        fold_v[wid[b["fold_pairs"][:, 0]]] = True
    bk = np.minimum(fab["bend"] * bd_w, 1.0)
    bk[fold_v] = 0.0
    cl2 = Cloth(Xw, Fw, rlw, st_w, bk, np.ones(m), thickness=0.005)
    if fab.get("bend2", 0) > 0:
        SP_, SR_ = skip_springs(b)
        SPw = wid[SP_]
        ok = (SPw[:, 0] != SPw[:, 1]) & ~fold_v[SPw[:, 0]] & ~fold_v[SPw[:, 1]]
        SPw, SR_ = SPw[ok], SR_[ok]
        k2 = np.minimum(fab["bend2"] * 0.5 * (bd_w[SPw[:, 0]] + bd_w[SPw[:, 1]]), 1.0)
        cl2.add_bend(SPw, SR_, k2)
    cl2.compress = fab["compress"]
    cl2.set_exclusion(np.arange(m), rings=2)
    cl2.self_r = 0.0075
    cl2.vt_r = 0.02
    # elastic bands grip the skin: they don't slide
    cl2.fric_v = np.where(wmax(elastic) > 0, 1.0, 0.4)
    cfw = np.zeros(m, bool)
    cfw[wid[cf]] = True
    cl2.cf = np.nonzero(cfw)[0]
    no_cling = 1.0 - wmax(collar)          # collars keep their own shape
    fall_w = wmax(fall) > 0
    fall_w[fold_v] = False                  # the fold line itself stays on the stand
    if fall_w.any():
        # the fall is the top layer: it stays outside the stand and the bodice under it
        cl2.layer_top = np.nonzero(fall_w)[0]
        base = ~(wmax(fall) > 0)
        cl2.layer_tris = Fw[base[Fw].all(1)]
    dt = 1 / 600                 # small substeps: the hanging fabric stays (nearly) inextensible
    t0 = time.time()
    N = steps_drape
    cling = fab.get("cling", 6.0)
    for k in range(N):
        f = k / N
        gp_body.set_arm_drop(ARM_START + (40.0 - ARM_START) * min(f / 0.3, 1.0))
        # dressing: a temporary inward pull (hands smoothing the garment onto the body) makes the
        # ease buckle into folds whose size the bending stiffness sets; then it is let go
        c = cling * np.clip(f / 0.25, 0, 1) * np.clip((0.55 - f) / 0.2, 0, 1)
        acc = None
        if c > 0:
            _, nb = gp_body.sdf(cl2.x)
            acc = -nb * (c * no_cling)[:, None]
        damp = fab["damping"] if f < 0.75 else (0.15 if f < 0.9 else 0.45)
        vmax = cl2.step(dt, gravity=9.81, iters=4, damping=damp / 3, accel=acc)
        if preview_cb and k % 900 == 899:
            preview_cb(cl2.x, Fw, k)
    gp_body.set_arm_drop(40.0)
    sp_ = np.linalg.norm(cl2.v, axis=1)
    imax = int(np.argmax(sp_))
    log("  draped %d steps (%.0fs), final max speed %.3f m/s at %s (%d vertices > 0.05 m/s)"
        % (steps_drape, time.time() - t0, vmax, np.round(cl2.x[imax], 3), int((sp_ > 0.05).sum())))
    out = dict(X=cl2.x.copy(), F=Fw, wid=wid, m=m, rib=wmax(rib), elastic=wmax(elastic), collar=wmax(collar),
               rest=rlw, cf=cl2.cf, fold_v=fold_v, cloth=cl2)
    return out


# ============================================================================ post
def fair(d, iters=4, lam=0.5, mu=-0.53):
    """Taubin smoothing of mesh-scale noise (keeps folds, no shrinkage). Boundary vertices are
    smoothed only along their boundary loop, sharp boundary corners (collar points, the V) stay;
    the centre-front line stays at x = 0; then the body is cleared again."""
    X, F = d["X"].copy(), d["F"]
    m = len(X)
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    Eu, cnt = np.unique(E, axis=0, return_counts=True)
    bnd = Eu[cnt == 1]
    isb = np.zeros(m, bool)
    isb[bnd.ravel()] = True
    # interior: all neighbours; boundary: boundary neighbours only
    I = np.concatenate([Eu[:, 0], Eu[:, 1]])
    J = np.concatenate([Eu[:, 1], Eu[:, 0]])
    keep = ~isb[I]
    Ib = np.concatenate([bnd[:, 0], bnd[:, 1]])
    Jb = np.concatenate([bnd[:, 1], bnd[:, 0]])
    I2, J2 = np.concatenate([I[keep], Ib]), np.concatenate([J[keep], Jb])
    deg = np.bincount(I2, minlength=m).astype(float)
    W = sp.coo_matrix((1.0 / deg[I2], (I2, J2)), shape=(m, m)).tocsr()
    fixed = np.zeros(m, bool)
    # sharp boundary corners
    nb = [[] for _ in range(m)]
    for a, b_ in bnd:
        nb[a].append(b_)
        nb[b_].append(a)
    for v in np.nonzero(isb)[0]:
        if len(nb[v]) != 2:
            fixed[v] = True
            continue
        u1, u2 = X[nb[v][0]] - X[v], X[nb[v][1]] - X[v]
        c = (u1 @ u2) / (np.linalg.norm(u1) * np.linalg.norm(u2) + 1e-12)
        if c > -0.82:          # turning by more than ~35 degrees
            fixed[v] = True
    for _ in range(iters):
        for k in (lam, mu):
            L = W @ X - X
            X = np.where(fixed[:, None], X, X + k * L)
            if len(d["cf"]):
                X[d["cf"], 0] = 0.0
    dist, nrm = gp_body.sdf(X)
    pen = dist < 0.005
    X[pen] += nrm[pen] * (0.005 - dist[pen])[:, None]
    d["X"] = X


def orient(b, d):
    """Flip pieces whose triangles face the body."""
    X, F = d["X"], d["F"]
    fpid = b["ftri_pid"]
    c = X[F].mean(1)
    fn = np.cross(X[F[:, 1]] - X[F[:, 0]], X[F[:, 2]] - X[F[:, 0]])
    _, bn = gp_body.sdf(c)
    s = (fn * bn).sum(1)
    flip = np.zeros(len(F), bool)
    for k in np.unique(fpid):
        sel = fpid == k
        if (np.sign(s[sel]) < 0).sum() > sel.sum() / 2:
            flip[sel] = True
    F = F.copy()
    F[flip] = F[flip][:, ::-1]
    Fr = b["F"].copy()
    Fr[flip] = Fr[flip][:, ::-1]
    d["F"], d["Frender"] = F, Fr
    return flip


def colours(b, d):
    X, F, m = d["X"], d["F"], d["m"]
    dist, bn = gp_body.sdf(X)
    seed = np.zeros(m, bool)
    # torso: the yoke resting on the shoulders
    seed |= (dist < 0.02) & (bn[:, 1] > 0.3) & (X[:, 1] > 0.44)
    # sleeve heads: the top of the upper arm near the shoulder
    for s in (1, -1):
        rel = X - gp_body.SH[s]
        along = rel @ gp_body.arm_dir(s)
        up = rel @ gp_body.arm_up(s)
        seed |= (along > 0) & (along < 0.09) & (up > 0.02) & (dist < 0.025)
    # back of the collar
    seed |= (d["collar"] > 0) & (X[:, 2] < 0.0)
    # geodesic falloff over the mesh
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    w = np.linalg.norm(X[E[:, 0]] - X[E[:, 1]], axis=1)
    G = sp.coo_matrix((w, (E[:, 0], E[:, 1])), shape=(m, m)).tocsr()
    G = G.maximum(G.T)
    if seed.any():
        dg = dijkstra(G, directed=False, indices=np.nonzero(seed)[0], min_only=True)
    else:
        dg = np.full(m, np.inf)
    t = np.clip(dg / 0.07, 0, 1)
    R = 1 - (3 * t * t - 2 * t ** 3)
    C = np.stack([R, d["elastic"], d["rib"], np.ones(m)], 1)
    d["C"] = C
    return C


def checks(b, d, log):
    X, F = d["X"], d["F"]
    dist, _ = gp_body.sdf(X)
    c = X[F].mean(1)
    dc, _ = gp_body.sdf(c)
    fn = np.cross(X[F[:, 1]] - X[F[:, 0]], X[F[:, 2]] - X[F[:, 0]])
    area = 0.5 * np.linalg.norm(fn, axis=1)
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    Eu, cnt = np.unique(E, axis=0, return_counts=True)
    dup = len(F) - len(np.unique(np.sort(F, axis=1), axis=0))
    # edge strain vs pattern
    L = np.linalg.norm(X[Eu[:, 0]] - X[Eu[:, 1]], axis=1)
    R = np.array([d["rest"][(int(a), int(b_))] for a, b_ in Eu])
    strain = L / R - 1
    # orientation consistency: an interior edge must appear once in each direction
    De = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    key = np.sort(De, axis=1)
    dirn = (De[:, 0] < De[:, 1]).astype(int)
    order = np.lexsort((key[:, 1], key[:, 0]))
    k2, d2 = key[order], dirn[order]
    same = np.all(k2[1:] == k2[:-1], axis=1)
    incons = int((same & (d2[1:] == d2[:-1])).sum())
    si = self_intersections(X, F)
    log("  min clearance verts %.1f mm, tri centres %.1f mm; non-manifold edges %d; dup tris %d; "
        "min area %.2e; strain p50 %.3f p99 %.3f max %.3f (min %.3f); inconsistent edges %d; "
        "self-intersecting edge/tri pairs %d" % (dist.min() * 1000, dc.min() * 1000, int((cnt > 2).sum()), dup,
                                                area.min(), np.median(strain), np.percentile(strain, 99),
                                                strain.max(), strain.min(), incons, si))
    return dict(clear=float(dist.min()), nonmanifold=int((cnt > 2).sum()), dup=dup, incons=incons, selfx=si)


def untangle(d, log, gap=0.002, rounds=30):
    """Resolve remaining crossings locally. For each crossing edge/triangle pair the element
    farther from the body is the outer layer: the edge end below the triangle is lifted above
    it, or the triangle is lifted above the edge end that pokes through it (along the
    triangle's outward normal), with a small gap."""
    X, F = d["X"].copy(), d["F"]
    X0 = X.copy()
    best = (self_intersections(X, F), X.copy())
    r = -1
    for r in range(rounds):
        E, ei, ti = self_intersections(X, F, detail=True)
        if len(ei) == 0:
            break
        p, q = X[E[ei, 0]], X[E[ei, 1]]
        a, b_, c = X[F[ti, 0]], X[F[ti, 1]], X[F[ti, 2]]
        n = np.cross(b_ - a, c - a)
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        tc = (a + b_ + c) / 3
        dt, nb = gp_body.sdf(tc)
        n *= np.where((n * nb).sum(1) < 0, -1.0, 1.0)[:, None]
        de, _ = gp_body.sdf(0.5 * (p + q))
        sp_, sq_ = ((p - a) * n).sum(1), ((q - a) * n).sum(1)
        acc = np.zeros_like(X)
        cnt = np.zeros(len(X))
        edge_out = de >= dt
        # outer edge: lift its end below the triangle
        lo_p = sp_ < sq_
        v_lo = np.where(lo_p, E[ei, 0], E[ei, 1])
        s_lo = np.minimum(sp_, sq_)
        k = edge_out
        np.add.at(acc, v_lo[k], n[k] * (gap - s_lo[k])[:, None])
        np.add.at(cnt, v_lo[k], 1)
        # outer triangle: lift it above the edge end that pokes through
        s_hi = np.maximum(sp_, sq_)
        k = ~edge_out
        for j in range(3):
            np.add.at(acc, F[ti[k], j], n[k] * (s_hi[k] + gap)[:, None])
            np.add.at(cnt, F[ti[k], j], 1)
        mv = cnt > 0
        step = acc[mv] / cnt[mv, None]
        ln = np.linalg.norm(step, axis=1, keepdims=True)
        X[mv] += step * np.minimum(1.0, 0.002 / (ln + 1e-12))       # at most 2 mm per round
        n_now = self_intersections(X, F)
        if n_now < best[0]:
            best = (n_now, X.copy())
        elif n_now > best[0] * 1.2 + 2:
            break                                                    # not converging: stop
    X = best[1]
    d["X"] = X
    log("  untangle: %d rounds, %d vertices moved (max %.1f mm), %d crossings left"
        % (r + 1, int((np.abs(X - X0).max(1) > 0).sum()), np.abs(X - X0).max() * 1000, best[0]))


def self_intersections(X, F, detail=False):
    """Count edge-triangle crossings between non-adjacent elements (segment/triangle test)."""
    from scipy.spatial import cKDTree
    E = np.unique(np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1), axis=0)
    tc = X[F].mean(1)
    em = 0.5 * (X[E[:, 0]] + X[E[:, 1]])
    tree = cKDTree(tc)
    cand = tree.query_ball_point(em, 0.03)
    ei = np.concatenate([np.full(len(c), i) for i, c in enumerate(cand)])
    ti = np.concatenate([np.array(c, int) for c in cand])
    Ft = F[ti]
    share = (Ft == E[ei, 0][:, None]).any(1) | (Ft == E[ei, 1][:, None]).any(1)
    ei, ti = ei[~share], ti[~share]
    p, q = X[E[ei, 0]], X[E[ei, 1]]
    a, b, c = X[F[ti, 0]], X[F[ti, 1]], X[F[ti, 2]]
    n = np.cross(b - a, c - a)
    dp, dq = ((p - a) * n).sum(1), ((q - a) * n).sum(1)
    cross = dp * dq < 0
    t = dp / (dp - dq + 1e-30)
    x = p + (q - p) * t[:, None]

    def inside(x, a, b, c, n):
        return ((np.cross(b - a, x - a) * n).sum(1) >= 0) & ((np.cross(c - b, x - b) * n).sum(1) >= 0) & \
            ((np.cross(a - c, x - c) * n).sum(1) >= 0)

    hit = cross & inside(x, a, b, c, n)
    if detail:
        return E, ei[hit], ti[hit]
    return int(hit.sum())


def measure(d, y):
    """Perimeter of the garment's torso cross-section at height y (convex hull of the slice and
    the actual loop length)."""
    X, F = d["X"], d["F"]
    pts = []
    for a, b_ in ((0, 1), (1, 2), (2, 0)):
        p, q = X[F[:, a]], X[F[:, b_]]
        s = (p[:, 1] - y) * (q[:, 1] - y) < 0
        t = (y - p[s, 1]) / (q[s, 1] - p[s, 1])
        pts.append(p[s] + (q[s] - p[s]) * t[:, None])
    P = np.vstack(pts)
    # keep the torso's loop: points within reach of the body section, not the sleeves
    a, dd = gp_body.torso_radii(y)
    P = P[(P[:, 0] / (a + 0.12)) ** 2 + (P[:, 2] / (dd + 0.12)) ** 2 < 1]
    if len(P) < 3:
        return 0.0, 0.0
    ang = np.arctan2(P[:, 2], P[:, 0])
    o = np.argsort(ang)
    Q = P[o][:, [0, 2]]
    loop = np.linalg.norm(np.diff(np.vstack([Q, Q[:1]]), axis=0), axis=1).sum()
    from scipy.spatial import ConvexHull
    hull = ConvexHull(Q)
    hp = Q[hull.vertices]
    hper = np.linalg.norm(np.diff(np.vstack([hp, hp[:1]]), axis=0), axis=1).sum()
    return float(hper), float(loop)


def render_preview(b, d, path, mannequin, colour=None, mode="shade"):
    import gp_render
    X, F = d["X"], d["F"]
    N = gp_render.vertex_normals(X, F)
    if mode == "pins":
        C = d["C"]
        col = np.stack([0.3 + 0.7 * C[:, 0], 0.35 + 0.5 * C[:, 1], 0.35 + 0.6 * C[:, 2]], 1)
    else:
        col = np.asarray(colour if colour is not None else b["g"]["color"])
    mv, mf = mannequin
    views = []
    for yaw in (0, 35, 180):
        views.append(gp_render.render([dict(V=mv, F=mf, color=(0.62, 0.66, 0.72)),
                                       dict(V=X, F=F, N=N, color=col)], yaw=yaw))
    gp_render.save(path, views, ["front", "3/4", "back"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*", default=list(gp_garments.GARMENTS))
    ap.add_argument("--preview", default=None, help="write preview PNGs to this directory")
    ap.add_argument("--work", default=None, help="write intermediate .npz here")
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args()
    log = lambda s: print(s, flush=True)  # noqa: E731
    mann = gp_body.mannequin_mesh() if a.preview else None
    for key in a.names:
        h = H0
        while True:
            b = build(key, h, log)
            if b["m"] <= MAX_VERTS:
                break
            h *= np.sqrt(b["m"] / (MAX_VERTS * 0.985))
        steps = (150, 900) if a.fast else (500, 3300)
        d = drape(b, log, *steps)
        fair(d)
        untangle(d, log)
        orient(b, d)
        colours(b, d)
        chk = checks(b, d, log)
        X, F = d["X"], d["F"]
        N = vertex_normals_w(X, F)
        UV = layout_uv(b)
        name = key
        gp_fbx.write(os.path.join(a.out, name + ".fbx"), name, X, F, N, UV, d["Frender"], d["C"])
        ch_body = gp_body.chest_circumference(0.30)
        hull, loop = measure(d, 0.30)
        hem = b["g2"]["hem_y"]
        hy = hem_height(b, d)
        hh, hl = measure(d, hy + 0.02)
        log("  %s: verts %d tris %d  chest hull %.3f (body %.3f, ease %.3f) loop %.3f  hem y %.3f (target %.3f) "
            "hem hull %.3f" % (key, len(X), len(F), hull, ch_body, hull - ch_body, loop, hy, hem, hh))
        if a.work:
            np.savez(os.path.join(a.work, key + ".npz"), X=X, F=F, UV=UV, Fr=d["Frender"], C=d["C"], wid=d["wid"])
        if a.preview:
            render_preview(b, d, os.path.join(a.preview, key + ".png"), mann)
            render_preview(b, d, os.path.join(a.preview, key + "_colours.png"), mann, mode="pins")


def vertex_normals_w(X, F):
    fn = np.cross(X[F[:, 1]] - X[F[:, 0]], X[F[:, 2]] - X[F[:, 0]])
    n = np.zeros_like(X)
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def hem_height(b, d):
    """Mean height of the bottom opening (lowest boundary loop) at the centre front/back."""
    X, F = d["X"], d["F"]
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    Eu, cnt = np.unique(E, axis=0, return_counts=True)
    bv = np.unique(Eu[cnt == 1])
    torso = bv[np.abs(X[bv, 0]) < 0.05]
    return float(X[torso, 1].min()) if len(torso) else float("nan")


def layout_uv(b):
    """Pattern coordinates (m of fabric) laid out side by side, hems aligned at v = 0."""
    UV = b["UV"].copy()
    x = 0.0
    for nm in b["names"]:
        pan = b["panels"][nm]
        sl = slice(b["off"][nm], b["off"][nm] + len(pan.uv))
        u = UV[sl, 0]
        UV[sl, 0] = u - u.min() + x
        if pan.kind in ("collar", "fall", "band", "cuff"):
            UV[sl, 1] = UV[sl, 1] - UV[sl, 1].min()
        x = UV[sl, 0].max() + 0.05
    return UV


if __name__ == "__main__":
    main()
