"""Pattern drafting for the five catalogue tops (medium adult, fitted to the reference mannequin).

Every top is drafted the same way a flat pattern is: a front and a back bodice (one piece each,
symmetric about the centre line), two sleeves whose cap length is solved to match the armhole,
and strips (neck band, collar stand + fall, hem band, cuffs) sewn onto a chain of edges. Pattern
coordinates are metres of fabric: u across, v along the grain (up the body, up the sleeve).
"""
import numpy as np

from gp_pattern import Panel, Seam, bezier, line, polylen, split_at
import gp_body

WRIST_L = 0.565       # cuff edge: distance along the arm from the shoulder joint (wrist at 0.55)
HPS_Y = 0.562          # high point shoulder (side neck point) on the mannequin, plus thickness


def mirror_edges(edges):
    """Right-half edges (u >= 0, in order) -> the left half, mirrored and reversed."""
    out = []
    for n, p in reversed(edges):
        q = p[::-1].copy()
        q[:, 0] *= -1
        out.append((n[:-2] + "_l", q))
    return out


def bodice(name, g, back=False):
    L = g["L"] - (g.get("back_short", 0.015) if back else 0.0)   # the back is flatter than the front
    Wc, Wh, vU = g["Wc"], g["Wh"], L - g["AD"]
    Sx, Sd, Nw = g["Sx"] + (0.004 if back else 0), g["Sd"], g["Nw"]
    rise = g.get("tail", 0.0)
    if rise > 0:
        hem = bezier((0, 0), (Wh * 0.45, 0), (Wh * 0.8, rise * 0.15), (Wh, rise))
    else:
        hem = line((0, 0), (Wh, 0))
    side = line((Wh, rise), (Wc, vU))
    top = L - Sd
    sc = max(Wc - Sx, 0.012)
    arm = bezier((Wc, vU), (Wc - 0.75 * sc, vU), (Sx - 0.002, vU + 0.3 * (top - vU)), (Sx, top))
    shoulder = line((Sx, top), (Nw, L))
    if back:
        d = g["Bd"]
        neck = bezier((Nw, L), (Nw * 0.92, L - d * 0.8), (Nw * 0.45, L - d), (0, L - d))
    elif g["neck"] == "v":
        d = g["Vd"]
        neck = bezier((Nw, L), (Nw * 0.78, L - d * 0.28), (Nw * 0.22, L - d * 0.82), (0, L - d))
    else:
        d = g["Nd"]
        neck = bezier((Nw, L), (Nw * 0.98, L - d * 0.62), (Nw * 0.58, L - d), (0, L - d))
    right = [("hem_r", hem), ("side_r", side), ("arm_r", arm), ("shoulder_r", shoulder), ("neck_r", neck)]
    edges = right + mirror_edges(right)
    internal = []
    if not back:
        internal.append(line((0, 0), (0, L - d), 2))
        pw = g.get("placket", 0.0)
        if pw > 0:
            internal += [line((s * pw, 0.0), (s * pw, L - d - 0.006), 2) for s in (1, -1)]
    elif g.get("yoke"):
        # yoke seam line across the back, stopping 12 mm short of the armholes
        yv = L - g["yoke"]
        xa = np.interp(yv, arm[:, 1], arm[:, 0]) - 0.012
        internal.append(line((-xa, yv), (xa, yv), 2))
    tags = {"hemfold": g.get("hemfold", 0.0), "placket": g.get("placket", 0.0),
            "yoke": g.get("yoke", 0.0), "L": L}
    return Panel(name, edges, "back" if back else "front", 0, internal, bend=1.0, tags=tags)


def sleeve(name, side, g, armhole_len):
    B, Wo, SL = g["B"], g["Wo"], g["SL"]
    target = armhole_len * (1 + g.get("cap_ease", 0.0))

    def cap(h):
        yb = SL - h
        return bezier((B / 2, yb), (B / 2 * 0.6, yb + 0.1 * h), (B / 2 * 0.36, SL), (0, SL))

    lo, hi = 0.0, 0.3
    for _ in range(60):
        h = 0.5 * (lo + hi)
        if polylen(cap(h)) < target:
            lo = h
        else:
            hi = h
    h = 0.5 * (lo + hi)
    if h < 0.03:
        print("  warning: %s cap height %.3f (sleeve too wide for the armhole)" % (name, h))
    cf = cap(h)
    cb = cf[::-1].copy()
    cb[:, 0] *= -1
    edges = [("hem", line((-Wo / 2, 0), (Wo / 2, 0))), ("under_f", line((Wo / 2, 0), (B / 2, SL - h))),
             ("cap_f", cf), ("cap_b", cb), ("under_b", line((-B / 2, SL - h), (-Wo / 2, 0)))]
    p = Panel(name, edges, "sleeve", side, bend=1.0, tags={"hemfold": g.get("sleevefold", 0.0)})
    p.place = {"capH": h, "B": B, "Wo": Wo, "SL": SL}
    return p


def strip(name, length, height, chain_lens, kind, rib=False, elastic=False, closed=True,
          points=0.0, bend=1.0, stretch=1.0, place=None):
    """Rectangle (or a collar fall with points) whose top edge (v = height) is sewn onto a chain."""
    p_ = points
    bottom = line((-p_, 0), (length + p_, 0))
    end_r = line((length + p_, 0), (length, height))
    top = line((length, height), (0, height), 64)
    subs = split_at(top, np.cumsum(chain_lens)[:-1] / np.sum(chain_lens)) if len(chain_lens) > 1 else [top]
    end_l = line((0, height), (-p_, 0))
    edges = [("free", bottom), ("end_r", end_r)] + [("top%d" % i, s) for i, s in enumerate(subs)] + [("end_l", end_l)]
    pan = Panel(name, edges, kind, 0, rib=rib, elastic=elastic, bend=bend, stretch=stretch,
                place=dict(place or {}, length=length, height=height, closed=closed))
    return pan


def draft(g):
    """Returns (panels dict, seams list, strips list of (panel name, chain))."""
    hem_top = g["hem_y"] + (g["band"]["h"] if g.get("band") else 0.0)
    g = dict(g)
    g["L"] = HPS_Y - hem_top + g.get("extra", 0.03)
    P = {}
    P["front"] = bodice("front", g, back=False)
    P["back"] = bodice("back", g, back=True)
    P["front"].place["hem_y"] = hem_top
    P["back"].place["hem_y"] = hem_top
    arm = polylen(P["front"].edge("arm_r"))
    for s, nm in ((1, "sleeve_r"), (-1, "sleeve_l")):
        P[nm] = sleeve(nm, s, g, arm)
        # long sleeves end with the cuff at the wrist; what is longer stacks above it
        if g.get("cuff"):
            P[nm].place["l_hem"] = WRIST_L - g["cuff"]["h"]
    S = []
    for sd in ("r", "l"):
        S.append(Seam(("front", "side_" + sd), ("back", "side_" + sd)))
        S.append(Seam(("front", "shoulder_" + sd), ("back", "shoulder_" + sd)))
        S.append(Seam(("front", "arm_" + sd), ("sleeve_" + sd, "cap_f")))
        S.append(Seam(("back", "arm_" + sd), ("sleeve_" + sd, "cap_b")))
        S.append(Seam(("sleeve_" + sd, "under_f"), ("sleeve_" + sd, "under_b")))
    strips = []

    def add_strip(pan, chain, closed=True, fold=False):
        P[pan.name] = pan
        for i, (pn, en, _) in enumerate(chain):
            S.append(Seam((pan.name, "top%d" % i), (pn, en), kind="fold" if fold else "seam"))
        if closed:
            S.append(Seam((pan.name, "end_l"), (pan.name, "end_r")))
        strips.append((pan.name, chain))

    def chain_len(chain):
        return [polylen(P[pn].edge(en)) for pn, en, _ in chain]

    # ---- neck
    if g["neck"] == "crew":
        ch = [("back", "neck_l", False), ("front", "neck_l", True), ("front", "neck_r", True), ("back", "neck_r", True)]
        cl = chain_len(ch)
        nb = g["neckband"]
        add_strip(strip("neckband", sum(cl) * nb["ratio"], nb["h"], cl, "collar", rib=True,
                        stretch=0.5, place={"dir": "up"}), ch)
    elif g["neck"] in ("shirt", "bomber"):
        ch = [("front", "neck_r", True), ("back", "neck_r", False), ("back", "neck_l", False), ("front", "neck_l", False)]
        cl = chain_len(ch)
        c = g["collar"]
        rib = g["neck"] == "bomber"
        stand = strip("stand", sum(cl) * c.get("ratio", 1.0), c["stand"], cl, "collar", rib=rib,
                      stretch=0.5 if rib else 1.0, bend=c.get("bend", 2.0), place={"dir": "up"})
        add_strip(stand, ch)
        if c.get("fall"):
            ln = stand.place["length"]
            fall = strip("fall", ln, c["fall"], [ln], "fall", closed=False, points=c["points"],
                         bend=c.get("bend", 2.0), place={"dir": "fold"})
            add_strip(fall, [("stand", "free", False)], closed=False, fold=True)
    # ---- hem band
    if g.get("band"):
        ch = [("front", "hem_r", False), ("back", "hem_r", True), ("back", "hem_l", True), ("front", "hem_l", True)]
        b = g["band"]
        add_strip(strip("hemband", b["circ"], b["h"], chain_len(ch), "band", rib=True, elastic=True,
                        stretch=0.25, place={"dir": "down"}), ch)
    # ---- cuffs
    if g.get("cuff"):
        c = g["cuff"]
        for sd in ("r", "l"):
            ch = [("sleeve_" + sd, "hem", False)]
            add_strip(strip("cuff_" + sd, c["circ"], c["h"], chain_len(ch), "cuff", rib=c["rib"],
                            elastic=c["rib"], stretch=0.25 if c["rib"] else 1.0, bend=c.get("bend", 1.0),
                            place={"dir": "arm", "side": 1 if sd == "r" else -1}), ch)
    return P, S, strips, g


# ---------------------------------------------------------------------------- the catalogue
# Fabric: bend = bending stiffness per Gauss-Seidel pass (0..1), compress = compression stiffness
# relative to stretch, density only scales gravity's share against damping (PBD is mass-free).
GARMENTS = {
    "heavyweight_tee": dict(
        title="Heavyweight Tee", hem_y=-0.09, Wc=0.25, Wh=0.25, Sx=0.236, Sd=0.032, AD=0.235,
        Nw=0.085, Nd=0.07, Bd=0.02, neck="crew", neckband=dict(h=0.02, ratio=0.9),
        B=0.38, Wo=0.35, SL=0.20, cap_ease=0.0, hemfold=0.025, sleevefold=0.022, extra=0.036,
        fabric=dict(bend=0.30, bend2=0.12, compress=0.6, damping=0.03), color=(0.90, 0.89, 0.86)),
    "crewneck_sweatshirt": dict(
        title="Crewneck Sweatshirt", hem_y=-0.04, Wc=0.265, Wh=0.255, Sx=0.25, Sd=0.032, AD=0.25, extra=0.048,
        Nw=0.09, Nd=0.075, Bd=0.02, neck="crew", neckband=dict(h=0.025, ratio=0.9),
        B=0.38, Wo=0.28, SL=0.555, cap_ease=0.0, band=dict(h=0.06, circ=0.84),
        cuff=dict(h=0.065, circ=0.21, rib=True),
        fabric=dict(bend=0.45, bend2=0.25, compress=0.6, damping=0.03), color=(0.52, 0.54, 0.57)),
    "satin_blouse": dict(
        title="Satin Blouse", hem_y=-0.10, Wc=0.245, Wh=0.275, Sx=0.212, Sd=0.045, AD=0.215, extra=0.044,
        Nw=0.08, Vd=0.16, Bd=0.02, neck="v", B=0.32, Wo=0.30, SL=0.53, cap_ease=0.02,
        hemfold=0.008, cuff=dict(h=0.045, circ=0.235, rib=False, bend=2.0),
        fabric=dict(bend=0.03, compress=0.4, damping=0.02, cling=8.0), color=(0.43, 0.12, 0.20)),
    "flannel_overshirt": dict(
        title="Flannel Overshirt", hem_y=-0.12, tail=0.05, Wc=0.25, Wh=0.255, Sx=0.236, Sd=0.038,
        AD=0.235, Nw=0.072, Nd=0.065, Bd=0.018, neck="shirt", placket=0.018, yoke=0.10, extra=0.052,
        collar=dict(stand=0.03, fall=0.045, points=0.03, ratio=1.0, bend=3.0),
        B=0.37, Wo=0.29, SL=0.52, cap_ease=0.01, hemfold=0.012,
        cuff=dict(h=0.06, circ=0.255, rib=False, bend=3.0),
        fabric=dict(bend=0.40, bend2=0.25, compress=0.6, damping=0.03), color=(0.62, 0.15, 0.16)),
    "bomber_jacket": dict(
        title="Bomber Jacket", hem_y=-0.05, Wc=0.275, Wh=0.27, Sx=0.256, Sd=0.032, AD=0.27, extra=0.05,
        Nw=0.09, Nd=0.08, Bd=0.025, neck="bomber", collar=dict(stand=0.06, ratio=0.9, bend=1.0),
        B=0.40, Wo=0.32, SL=0.55, cap_ease=0.0, band=dict(h=0.065, circ=0.84),
        cuff=dict(h=0.07, circ=0.21, rib=True),
        fabric=dict(bend=0.80, bend2=0.5, compress=0.7, damping=0.03), color=(0.29, 0.36, 0.23)),
}
