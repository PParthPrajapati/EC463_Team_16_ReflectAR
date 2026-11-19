"""Small z-buffer software renderer for the preview images (numpy + OpenCV for PNG output)."""
import numpy as np
import cv2


def _look(yaw_deg, pitch_deg, target, dist):
    yaw, pitch = np.radians(yaw_deg), np.radians(pitch_deg)
    eye = target + dist * np.array([np.sin(yaw) * np.cos(pitch), np.sin(pitch), np.cos(yaw) * np.cos(pitch)])
    f = target - eye
    f /= np.linalg.norm(f)
    # canonical body space is left-handed (x right, y up, z front): camera right = up x f
    r = np.cross([0, 1.0, 0], f)
    r /= np.linalg.norm(r)
    u = np.cross(f, r)
    return eye, r, u, f


def vertex_normals(V, F):
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    n = np.zeros_like(V)
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def render(meshes, yaw=0.0, pitch=8.0, size=(600, 720), target=(0, 0.26, 0), dist=2.5, fov=30.0):
    """meshes: list of dict(V, F, color (3,) or per-vertex (n,3), two_sided_dim)."""
    W, H = size
    eye, r, u, f = _look(yaw, pitch, np.asarray(target, float), dist)
    foc = 0.5 * H / np.tan(np.radians(fov) / 2)
    zbuf = np.full((H, W), np.inf)
    img = np.zeros((H, W, 3))
    img[:] = (0.97, 0.97, 0.96)
    # key light from the upper left of the camera, a weak fill from the right
    light1 = 0.8 * u - 0.55 * r - 0.6 * f; light1 /= np.linalg.norm(light1)
    light2 = 0.2 * u + 0.8 * r - 0.3 * f; light2 /= np.linalg.norm(light2)
    for m in meshes:
        V, F = m["V"], m["F"]
        N = m.get("N")
        if N is None:
            N = vertex_normals(V, F)
        col = np.asarray(m["color"], float)
        if col.ndim == 1:
            col = np.tile(col, (len(V), 1))
        d = V - eye
        zc = d @ f
        xs = W / 2 + foc * (d @ r) / zc
        ys = H / 2 - foc * (d @ u) / zc
        # shade per vertex (Gouraud); back faces of the cloth (seen from inside) darker
        to_eye = eye - V
        to_eye /= np.linalg.norm(to_eye, axis=1, keepdims=True)
        facing = (N * to_eye).sum(1)
        Ns = np.where(facing[:, None] < 0, -N, N)
        lam = 0.16 + 0.8 * np.clip(Ns @ light1, 0, 1) ** 1.3 + 0.18 * np.clip(Ns @ light2, 0, 1)
        rim = 0.08 * (1 - np.abs(facing)) ** 2
        shade = col * lam[:, None] + rim[:, None]
        shade = np.where(facing[:, None] < 0, shade * m.get("inside", 0.55), shade)
        for t in F:
            x0, x1, x2 = xs[t]
            y0, y1, y2 = ys[t]
            minx, maxx = int(max(np.floor(min(x0, x1, x2)), 0)), int(min(np.ceil(max(x0, x1, x2)), W - 1))
            miny, maxy = int(max(np.floor(min(y0, y1, y2)), 0)), int(min(np.ceil(max(y0, y1, y2)), H - 1))
            if minx > maxx or miny > maxy:
                continue
            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-12:
                continue
            gx, gy = np.meshgrid(np.arange(minx, maxx + 1) + 0.5, np.arange(miny, maxy + 1) + 0.5)
            l0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / den
            l1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / den
            l2 = 1 - l0 - l1
            ins = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
            if not ins.any():
                continue
            z = l0 * zc[t[0]] + l1 * zc[t[1]] + l2 * zc[t[2]]
            zb = zbuf[miny:maxy + 1, minx:maxx + 1]
            upd = ins & (z < zb)
            if not upd.any():
                continue
            zb[upd] = z[upd]
            c = (l0[upd, None] * shade[t[0]] + l1[upd, None] * shade[t[1]] + l2[upd, None] * shade[t[2]])
            img[miny:maxy + 1, minx:maxx + 1][upd] = c
    return np.clip(img, 0, 1)


def save(path, views, labels=None):
    row = np.concatenate(views, axis=1)
    out = (row[:, :, ::-1] * 255).astype(np.uint8)
    if labels:
        w = views[0].shape[1]
        for i, s in enumerate(labels):
            cv2.putText(out, s, (10 + i * w, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2, cv2.LINE_AA)
    cv2.imwrite(path, out)
