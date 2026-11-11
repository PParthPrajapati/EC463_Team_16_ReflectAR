"""Offline cloth solver (numpy): position-based dynamics with graph-coloured Gauss-Seidel
constraint projection.

Constraints: edge lengths (rest = pattern lengths; stiff in stretch, a little softer in
compression so ease buckles into folds), bending (distance across each pair of adjacent
triangles, rest = the flat pattern), seams (pull sewn edges together, then weld), collision with
the mannequin (signed distance, thickness and friction) and self-collision (vertex-vertex
repulsion between mesh-distant vertices).
"""
import numpy as np
from scipy.spatial import cKDTree
import scipy.sparse as sp

import gp_body


def color_pairs(P, n):
    """Greedy colouring of index pairs so that no vertex appears twice in one colour."""
    order = np.arange(len(P))
    used = [set() for _ in range(n)]
    col = np.empty(len(P), np.int32)
    for k in order:
        a, b = P[k]
        c = 0
        ua, ub = used[a], used[b]
        while c in ua or c in ub:
            c += 1
        col[k] = c
        ua.add(c)
        ub.add(c)
    groups = []
    for c in range(col.max() + 1 if len(col) else 0):
        groups.append(np.nonzero(col == c)[0])
    return groups


def edges_of(F):
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    E = np.sort(E, axis=1)
    return np.unique(E, axis=0)


def bend_pairs(F, rest_len):
    """Opposite vertices across every interior edge, with the flat (unfolded) rest distance
    computed from rest edge lengths. rest_len: dict (i, j) sorted -> length."""
    E = np.concatenate([F[:, [0, 1, 2]], F[:, [1, 2, 0]], F[:, [2, 0, 1]]])  # (a, b, opposite)
    key = np.sort(E[:, :2], axis=1)
    order = np.lexsort((key[:, 1], key[:, 0]))
    key, E = key[order], E[order]
    same = np.all(key[1:] == key[:-1], axis=1)
    idx = np.nonzero(same)[0]
    A, B, C, D = [], [], [], []
    for i in idx:
        a, b = key[i]
        A.append(a); B.append(b); C.append(E[i, 2]); D.append(E[i + 1, 2])
    A, B, C, D = map(np.array, (A, B, C, D))

    def L(x, y):
        k = np.sort(np.stack([x, y], 1), axis=1)
        return np.array([rest_len[(int(p), int(q))] for p, q in k])

    lab, lac, lbc, lad, lbd = L(A, B), L(A, C), L(B, C), L(A, D), L(B, D)
    # put a at the origin, b on +x; c above, d below (unfolded flat)
    xc = (lab ** 2 + lac ** 2 - lbc ** 2) / (2 * lab)
    yc = np.sqrt(np.maximum(lac ** 2 - xc ** 2, 0))
    xd = (lab ** 2 + lad ** 2 - lbd ** 2) / (2 * lab)
    yd = -np.sqrt(np.maximum(lad ** 2 - xd ** 2, 0))
    rest = np.hypot(xc - xd, yc - yd)
    return np.stack([C, D], 1), rest, np.stack([A, B], 1)


class Cloth:
    def __init__(self, X, F, rest_len, stretch_k, bend_k, inv_mass, thickness=0.005):
        """X (n,3) start positions; F (m,3) triangles; rest_len dict for every edge; stretch_k,
        bend_k: per-vertex multipliers (an edge uses the mean of its ends)."""
        self.x = X.astype(np.float64).copy()
        self.v = np.zeros_like(self.x)
        self.F = F
        self.n = len(X)
        E = edges_of(F)
        self.E = E
        self.Erest = np.array([rest_len[(int(a), int(b))] for a, b in E])
        self.Ek = 0.5 * (stretch_k[E[:, 0]] + stretch_k[E[:, 1]])
        self.Ecol = color_pairs(E, self.n)
        Bp, Br, Bedge = bend_pairs(F, rest_len)
        self.B, self.Brest = Bp, Br
        self.Bk = 0.5 * (bend_k[Bedge[:, 0]] + bend_k[Bedge[:, 1]])
        self.Bcol = color_pairs(Bp, self.n) if len(Bp) else []
        self.w = inv_mass.astype(np.float64)
        self.thick = thickness
        self.seams = None
        self.seam_k = 0.0
        self.cf = np.zeros(0, int)
        self.excl = None
        self.self_r = 0.0
        self.compress = 1.0
        self.friction = 0.4
        self.pinned = np.zeros(self.n, bool)
        self.vt_r = 0.0
        self.layer_top = np.zeros(0, int)
        self.layer_tris = np.zeros((0, 3), int)
        self.layer_gap = 0.003
        self.thick_self = 0.004
        self.fric_v = None

    def add_bend(self, pairs, rest, k):
        """Extra bending constraints (e.g. skip springs across two collinear edges)."""
        if len(pairs) == 0:
            return
        self.B = np.vstack([self.B, pairs])
        self.Brest = np.concatenate([self.Brest, rest])
        self.Bk = np.concatenate([self.Bk, k])
        self.Bcol = color_pairs(self.B, self.n)

    # ------------------------------------------------------------ helpers
    def set_exclusion(self, ids, rings=2):
        """Vertex pairs closer than `rings` edges apart (in the welded topology `ids`) never
        repel each other."""
        E = self.E
        m = ids.max() + 1
        A = sp.coo_matrix((np.ones(len(E)), (ids[E[:, 0]], ids[E[:, 1]])), shape=(m, m)).tocsr()
        A = ((A + A.T + sp.eye(m)) > 0).astype(np.int8)
        R = A.copy()
        for _ in range(rings - 1):
            R = ((R @ A) > 0).astype(np.int8)
        R = R.tocoo()
        self.excl_keys = np.unique(R.row.astype(np.int64) * m + R.col)
        self.excl_ids = ids
        self.excl_m = m

    def _project_edges(self, p, cols, E, rest, k, compress=1.0):
        w = self.w
        for g in cols:
            a, b = E[g, 0], E[g, 1]
            d = p[a] - p[b]
            ln = np.sqrt((d * d).sum(1)) + 1e-12
            C = ln - rest[g]
            kk = k[g] * np.where(C < 0, compress, 1.0)
            wa, wb = w[a], w[b]
            s = kk * C / ((wa + wb + 1e-12) * ln)
            corr = d * s[:, None]
            p[a] -= corr * wa[:, None]
            p[b] += corr * wb[:, None]

    def _collide_body(self, p, prev=None):
        dist, nrm = gp_body.sdf(p)
        pen = dist < self.thick
        if pen.any():
            p[pen] += nrm[pen] * (self.thick - dist[pen])[:, None]
        return pen, nrm

    def _self_collide(self, p):
        if self.self_r <= 0:
            return
        tree = cKDTree(p)
        P = tree.query_pairs(self.self_r, output_type="ndarray")
        if len(P) == 0:
            return
        P = P[~self._excluded(P[:, 0], P[:, 1])]
        if len(P) == 0:
            return
        i, j = P[:, 0], P[:, 1]
        d = p[i] - p[j]
        ln = np.sqrt((d * d).sum(1)) + 1e-12
        # which side of each other the two vertices are on comes from the normals: keep the
        # side they were on at the start of the step (no crossing through)
        corr = d * ((self.self_r - ln) / ln * 0.5)[:, None]
        acc = np.zeros_like(p)
        cnt = np.zeros(len(p))
        np.add.at(acc, i, corr)
        np.add.at(acc, j, -corr)
        np.add.at(cnt, i, 1)
        np.add.at(cnt, j, 1)
        nz = cnt > 0
        p[nz] += acc[nz] / cnt[nz, None]

    def _excluded(self, i, j):
        ids, m, keys = self.excl_ids, self.excl_m, self.excl_keys
        a, b = ids[i], ids[j]
        key = np.minimum(a, b).astype(np.int64) * m + np.maximum(a, b)
        k = np.minimum(np.searchsorted(keys, key), len(keys) - 1)
        return (keys[k] == key) | (a == b)

    def _self_collide_vt(self, p, x0):
        """Vertex-triangle: keep every vertex on the side of nearby triangles it started the step
        on, at least `thick_self` away (pushes vertex and triangle apart)."""
        F = self.F
        tc = (p[F[:, 0]] + p[F[:, 1]] + p[F[:, 2]]) / 3
        dd, ti = cKDTree(tc).query(p, k=10, distance_upper_bound=self.vt_r)
        ok = np.isfinite(dd)
        vi = np.broadcast_to(np.arange(len(p))[:, None], ti.shape)[ok]
        ti = ti[ok]
        T = F[ti]
        a, b, c = p[T[:, 0]], p[T[:, 1]], p[T[:, 2]]
        n = np.cross(b - a, c - a)
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        s = ((p[vi] - a) * n).sum(1)
        pre = np.abs(s) < self.thick_self * 3
        if not pre.any():
            return
        vi, T, a, b, c, n, s = vi[pre], T[pre], a[pre], b[pre], c[pre], n[pre], s[pre]
        a0, b0, c0 = x0[T[:, 0]], x0[T[:, 1]], x0[T[:, 2]]
        s0 = ((x0[vi] - a0) * np.cross(b0 - a0, c0 - a0)).sum(1)
        side = np.where(s0 >= 0, 1.0, -1.0)
        q = p[vi] - s[:, None] * n

        def area(u, v, w):
            return (np.cross(v - u, w - u) * n).sum(1)
        A = area(a, b, c) + 1e-18
        l0, l1 = area(q, b, c) / A, area(a, q, c) / A
        l2 = 1 - l0 - l1
        ins = (l0 > -0.05) & (l1 > -0.05) & (l2 > -0.05)
        depth = self.thick_self - side * s
        hit = ins & (depth > 0)
        if not hit.any():
            return
        idx = np.nonzero(hit)[0]
        excl = np.zeros(len(idx), bool)
        for k in range(3):
            excl |= self._excluded(vi[idx], T[idx, k])
        idx = idx[~excl]
        if len(idx) == 0:
            return
        vi, T, n, side, depth = vi[idx], T[idx], n[idx], side[idx], depth[idx]
        l0, l1, l2 = l0[idx], l1[idx], l2[idx]
        corr = n * (side * depth)[:, None]
        acc = np.zeros_like(p)
        cnt = np.zeros(len(p))
        np.add.at(acc, vi, corr * 0.5)
        np.add.at(cnt, vi, 1)
        for k, l in ((0, l0), (1, l1), (2, l2)):
            np.add.at(acc, T[:, k], -corr * 0.5 * np.clip(l, 0, 1)[:, None])
            np.add.at(cnt, T[:, k], 1)
        nz = cnt > 0
        p[nz] += acc[nz] / cnt[nz, None] * 1.5

    def _layer(self, p):
        """Layering: `layer_top` vertices (a collar fall) stay at least `layer_gap` outside the
        surface of the `layer_base` triangles beneath them (outside = away from the body)."""
        top, Fb = self.layer_top, self.layer_tris
        if len(top) == 0:
            return
        a, b, c = p[Fb[:, 0]], p[Fb[:, 1]], p[Fb[:, 2]]
        tc = (a + b + c) / 3
        dd, ti = cKDTree(tc).query(p[top], k=6, distance_upper_bound=0.025)
        ok = np.isfinite(dd)
        vi = np.broadcast_to(top[:, None], ti.shape)[ok]
        ti = ti[ok]
        if len(ti) == 0:
            return
        a, b, c = a[ti], b[ti], c[ti]
        n = np.cross(b - a, c - a)
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        _, nb = gp_body.sdf((a + b + c) / 3)
        n *= np.where((n * nb).sum(1) < 0, -1.0, 1.0)[:, None]
        s = ((p[vi] - a) * n).sum(1)
        q = p[vi] - s[:, None] * n

        def area(u, v, w):
            return (np.cross(v - u, w - u) * n).sum(1)
        A = area(a, b, c) + 1e-18
        l0, l1 = area(q, b, c) / A, area(a, q, c) / A
        ins = (l0 > -0.1) & (l1 > -0.1) & (1 - l0 - l1 > -0.1)
        need = ins & (s < self.layer_gap) & (s > -0.02)
        if not need.any():
            return
        push = np.zeros_like(p)
        cnt = np.zeros(len(p))
        np.add.at(push, vi[need], n[need] * (self.layer_gap - s[need])[:, None])
        np.add.at(cnt, vi[need], 1)
        m = cnt > 0
        p[m] += push[m] / cnt[m, None]

    def normals(self, p=None):
        p = self.x if p is None else p
        F = self.F
        fn = np.cross(p[F[:, 1]] - p[F[:, 0]], p[F[:, 2]] - p[F[:, 0]])
        n = np.zeros_like(p)
        for k in range(3):
            np.add.at(n, F[:, k], fn)
        return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)

    # ------------------------------------------------------------ step
    def step(self, dt, gravity=9.81, iters=12, damping=0.02, seam_cap=0.002, body=True, accel=None):
        x, v = self.x, self.v
        v *= (1.0 - damping)
        v[:, 1] -= gravity * dt
        if accel is not None:
            v += accel * dt
        v[self.pinned] = 0
        p = x + v * dt
        for it in range(iters):
            self._project_edges(p, self.Ecol, self.E, self.Erest, self.Ek, self.compress)
            if len(self.B):
                self._project_edges(p, self.Bcol, self.B, self.Brest, self.Bk, 1.0)
            if self.seams is not None and self.seam_k > 0:
                a, b = self.seams[:, 0], self.seams[:, 1]
                d = p[b] - p[a]
                ln = np.sqrt((d * d).sum(1)) + 1e-12
                mv = np.minimum(ln * 0.5 * self.seam_k, seam_cap)
                corr = d * (mv / ln)[:, None]
                np.add.at(p, a, corr)
                np.add.at(p, b, -corr)
            if len(self.cf):
                p[self.cf, 0] = 0.0
            if body and (it % 2 == 1 or it == iters - 1):
                self._collide_body(p)
        if self.self_r > 0:
            self._self_collide(p)
        if self.vt_r > 0:
            self._self_collide_vt(p, x)
        if len(self.layer_top):
            self._layer(p)
        if self.self_r > 0 or self.vt_r > 0:
            if body:
                self._collide_body(p)
        p[self.pinned] = x[self.pinned]
        nv = (p - x) / dt
        if body:
            dist, nrm = gp_body.sdf(p)
            touch = dist < self.thick * 1.5
            if touch.any():
                vn = (nv[touch] * nrm[touch]).sum(1, keepdims=True) * nrm[touch]
                fr = self.friction if self.fric_v is None else self.fric_v[touch][:, None]
                nv[touch] = vn + (nv[touch] - vn) * (1 - fr)
        self.v = nv
        self.x = p
        return float(np.abs(nv).max())
