"""2D pattern pieces, their triangulation and the seams that join them.

A Panel is a closed outline of named edges (polylines, metres, counter-clockwise) plus optional
internal constraint lines (centre front, placket, yoke) that the triangulation must follow. Seams
pair two edges; both are sampled with the same number of points so the seam vertices weld 1:1
(a shorter edge gathers the longer one, as a rib band does). Triangulation: Shewchuk's Triangle
(quality, max-area, no Steiner points on any segment).
"""
import numpy as np
import triangle as tr


def bezier(p0, p1, p2, p3, n=96):
    t = np.linspace(0, 1, n)[:, None]
    p0, p1, p2, p3 = map(np.asarray, (p0, p1, p2, p3))
    return ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + 3 * (1 - t) * t * t * p2 + t ** 3 * p3


def line(p0, p1, n=2):
    t = np.linspace(0, 1, n)[:, None]
    return np.asarray(p0) * (1 - t) + np.asarray(p1) * t


def polylen(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def resample(p, n):
    """n segments (n + 1 points) evenly spaced by arc length along polyline p."""
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(0, s[-1], n + 1)
    return np.stack([np.interp(t, s, p[:, k]) for k in range(p.shape[1])], 1)


def split_at(p, fracs):
    """Split polyline at arc-length fractions (sorted, in (0,1)) -> list of polylines."""
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    s /= s[-1]
    cuts = [0.0] + list(fracs) + [1.0]
    out = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        inner = (s > a + 1e-9) & (s < b - 1e-9)
        pa = np.array([np.interp(a, s, p[:, k]) for k in range(2)])
        pb = np.array([np.interp(b, s, p[:, k]) for k in range(2)])
        out.append(np.vstack([pa, p[inner], pb]))
    return out


class Panel:
    def __init__(self, name, edges, kind, side=0, internal=(), rib=False, elastic=False,
                 bend=1.0, stretch=1.0, place=None, tags=None):
        """edges: list of (name, polyline Nx2) counter-clockwise, each starting where the
        previous ended. kind: placement family. side: +1 right / -1 left / 0 centre."""
        self.name = name
        self.edges = [(n, np.asarray(p, float)) for n, p in edges]
        for (n0, a), (n1, b) in zip(self.edges, self.edges[1:] + self.edges[:1]):
            assert np.linalg.norm(a[-1] - b[0]) < 1e-6, (name, n0, n1, a[-1], b[0])
        self.kind = kind
        self.side = side
        self.internal = [np.asarray(p, float) for p in internal]
        self.rib = rib
        self.elastic = elastic
        self.bend = bend          # bending stiffness multiplier (interfacing, double layers)
        self.stretch = stretch    # stretch stiffness multiplier (rib knit is soft)
        self.place = place or {}
        self.tags = tags or {}
        self.counts = {}

    def edge(self, n):
        for k, p in self.edges:
            if k == n:
                return p
        raise KeyError((self.name, n))

    def area(self):
        p = np.vstack([e[:-1] for _, e in self.edges])
        x, y = p[:, 0], p[:, 1]
        return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def triangulate(self, h):
        pts, segs, eidx = [], [], {}
        for n, p in self.edges:
            q = resample(p, self.counts[n])
            start = len(pts)
            pts.extend(q[:-1])
            eidx[n] = list(range(start, start + len(q) - 1))
        nb = len(pts)
        for n, _ in self.edges:
            eidx[n].append(eidx[n][-1] + 1 if eidx[n][-1] + 1 < nb else 0)
        segs = [(i, (i + 1) % nb) for i in range(nb)]
        P = np.array(pts)
        for line_ in self.internal:
            n = max(1, int(np.ceil(polylen(line_) / h)))
            q = resample(line_, n)
            ids = []
            for pt in q:
                d = np.linalg.norm(P - pt, axis=1)
                j = int(np.argmin(d))
                if d[j] < 1e-7:
                    ids.append(j)
                else:
                    P = np.vstack([P, pt])
                    ids.append(len(P) - 1)
            segs += list(zip(ids[:-1], ids[1:]))
        area = np.sqrt(3) / 4 * h * h * 1.5
        T = tr.triangulate({"vertices": P, "segments": np.array(segs)}, "pq30a%.10fYYQ" % area)
        V, F = T["vertices"], T["triangles"]
        assert np.allclose(V[:len(P)], P), self.name
        fixed = np.zeros(len(V), bool)
        fixed[:len(P)] = True
        used = np.zeros(len(V), bool)
        used[F.ravel()] = True
        assert used[:nb].all(), "%s: boundary vertex outside the mesh" % self.name
        if not used.all():                      # e.g. a guide line poking outside the outline
            remap = np.cumsum(used) - 1
            V, F, fixed = V[used], remap[F], fixed[used]
            P = P[used[:len(P)]]
        V = _relax(V, F, fixed, 6)
        # counter-clockwise in pattern space
        e1, e2 = V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]
        cr = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        F[cr < 0] = F[cr < 0][:, ::-1]
        self.uv, self.tris, self.edge_ids, self.n_fixed = V, F, eidx, int(fixed.sum())
        return V, F


def _relax(V, F, fixed, iters):
    """Laplacian smoothing of the free interior vertices: a more even triangle size."""
    n = len(V)
    I = np.concatenate([F[:, 0], F[:, 1], F[:, 2], F[:, 1], F[:, 2], F[:, 0]])
    J = np.concatenate([F[:, 1], F[:, 2], F[:, 0], F[:, 0], F[:, 1], F[:, 2]])
    for _ in range(iters):
        acc = np.zeros_like(V)
        cnt = np.zeros(n)
        np.add.at(acc, I, V[J])
        np.add.at(cnt, I, 1)
        Vn = acc / np.maximum(cnt, 1)[:, None]
        V = np.where(fixed[:, None], V, 0.5 * V + 0.5 * Vn)
    return V


class Seam:
    def __init__(self, a, b, kind="seam", rev=None):
        """a, b: (panel, edge name). kind 'seam' | 'fold' (no bending stiffness across)."""
        self.a, self.b, self.kind, self.rev = a, b, kind, rev


def assign_counts(panels, seams, h):
    for p in panels.values():
        for n, e in p.edges:
            p.counts[n] = max(1, int(np.ceil(polylen(e) / h)))
    for s in seams:
        pa, na = s.a
        pb, nb = s.b
        la, lb = polylen(panels[pa].edge(na)), polylen(panels[pb].edge(nb))
        n = max(1, int(np.ceil(max(la, lb) / h)))
        panels[pa].counts[na] = n
        panels[pb].counts[nb] = n


def chain_split(length, parts):
    """Fractions at which an edge of `length` is split to sew onto chain `parts` (lengths)."""
    c = np.cumsum(parts)[:-1] / np.sum(parts)
    return list(c)
