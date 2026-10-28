#!/usr/bin/env python3
"""Akina Downhill centerline, traced from the real route map photo.

Usage: python3 make_akina_from_map.py <route-map-image.png> [akina.txt]

Pipeline: extract the orange route line from the map image -> Zhang-Suen
skeleton -> ordered walk from the Ikaho (green pin) end to the goal
(blue pin) end -> light Gaussian denoise -> scale to world meters sized so
the tightest drawn hairpins keep radius >= ~13.5 m (total ~9.86 km) ->
resample at 4 m -> designed elevation profile, camber from curvature,
corner segmentation/naming -> write the track file.
"""
import sys, math
import numpy as np
from PIL import Image

TARGET_LENGTH = 9880.0
START_WORLD = (1550.0, 1050.0)   # x, z of the start sample
START_PX = (568, 116)            # route end by the green pin in the image
GOAL_PX = (122, 588)             # route end by the blue pin in the image

def load_mask(path):
    a = np.asarray(Image.open(path).convert('RGB')).astype(int)
    R, G, B = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    return (R > 180) & (G > 60) & (G < 175) & (B < 100) & ((R - B) > 130)

def zhang_suen(img):
    img = img.astype(np.uint8)
    def nb(P):
        return (P[2:, 1:-1], P[2:, 2:], P[1:-1, 2:], P[:-2, 2:],
                P[:-2, 1:-1], P[:-2, :-2], P[1:-1, :-2], P[2:, :-2])
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            P = np.pad(img, 1)
            P2, P3, P4, P5, P6, P7, P8, P9 = nb(P)
            nbrs = [P2, P3, P4, P5, P6, P7, P8, P9]
            B = sum(nbrs)
            seq = nbrs if step == 0 else [P2, P9, P8, P7, P6, P5, P4, P3]
            A = np.zeros_like(img)
            for i in range(8):
                A += ((seq[i] == 0) & (seq[(i + 1) % 8] == 1))
            m, m2 = (P2 * P4 * P6, P4 * P6 * P8) if step == 0 else (P2 * P4 * P8, P2 * P6 * P8)
            delete = (P[1:-1, 1:-1] == 1) & (B >= 2) & (B <= 6) & (A == 1) & (m == 0) & (m2 == 0)
            if delete.any():
                img[delete] = 0; changed = True
    return img.astype(bool)

def trace(mask):
    skel = zhang_suen(mask)
    ys, xs = np.nonzero(skel)
    S = set(zip(xs.tolist(), ys.tolist()))
    def nbrs(p):
        x, y = p; out = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                q = (x + dx, y + dy)
                if q in S: out.append(q)
        return out
    ends = [p for p in S if len(nbrs(p)) == 1]
    start = min(ends, key=lambda p: (p[0] - START_PX[0]) ** 2 + (p[1] - START_PX[1]) ** 2)
    visited = {start}; chain = [start]; cur = start
    prev = np.array([-1.0, 0.3]); prev /= np.linalg.norm(prev)
    for _ in range(20000):
        cands = [q for q in nbrs(cur) if q not in visited]
        if not cands:
            best, bs = None, -1e18
            for q in S:
                if q in visited: continue
                dd = np.hypot(q[0] - cur[0], q[1] - cur[1])
                if dd > 30: continue
                v = np.array([q[0] - cur[0], q[1] - cur[1]], float); v /= (np.linalg.norm(v) + 1e-9)
                sc = float(np.dot(v, prev)) * 20 - dd
                if sc > bs: bs, best = sc, q
            if best is None: break
            cands = [best]
        nxt = max(cands, key=lambda q: float(np.dot(
            np.array([q[0] - cur[0], q[1] - cur[1]], float) /
            (np.hypot(q[0] - cur[0], q[1] - cur[1]) + 1e-9), prev)))
        v = np.array([nxt[0] - cur[0], nxt[1] - cur[1]], float)
        prev = 0.6 * prev + 0.4 * (v / (np.linalg.norm(v) + 1e-9)); prev /= np.linalg.norm(prev)
        cur = nxt; visited.add(cur); chain.append(cur)
        if np.hypot(cur[0] - GOAL_PX[0], cur[1] - GOAL_PX[1]) < 6: break
    return np.array(chain, dtype=float)

def gauss(pts, sigma, passes):
    r = max(1, int(sigma * 3))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2); k /= k.sum()
    out = pts.copy(); src = pts.copy()
    for _ in range(passes):
        new = out.copy()
        for c in range(2):
            new[:, c] = np.convolve(np.pad(out[:, c], r, mode='edge'), k, mode='valid')
        out = new; out[0] = src[0]; out[-1] = src[-1]
    return out

def cumlen(p):
    return np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])

def main():
    img_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else 'akina.txt'
    chain = gauss(trace(load_mask(img_path)), 0.8, 1)
    seg = np.linalg.norm(np.diff(chain, axis=0), axis=1)
    scale = TARGET_LENGTH / float(seg.sum())
    world = np.column_stack([START_WORLD[0] + (chain[:, 0] - START_PX[0]) * scale,
                             START_WORLD[1] + (START_PX[1] - chain[:, 1]) * scale])
    cum = cumlen(world); total = cum[-1]
    n = int(round(total / 4.0)); s_new = np.linspace(0, total, n + 1)
    pts = np.column_stack([np.interp(s_new, cum, world[:, 0]), np.interp(s_new, cum, world[:, 1])])
    cum = cumlen(pts); length = float(cum[-1]); n = len(pts)
    # headings / curvature
    yaw = np.zeros(n)
    for i in range(n):
        a = pts[max(0, i - 1)]; b = pts[min(n - 1, i + 1)]
        yaw[i] = math.atan2(b[0] - a[0], b[1] - a[1])
    theta = np.unwrap(yaw)
    hd = (theta + math.pi) % (2 * math.pi) - math.pi
    raw = np.zeros(n)
    for i in range(1, n - 1):
        ds = cum[i + 1] - cum[i - 1]
        raw[i] = (theta[i + 1] - theta[i - 1]) / ds if ds > 0.1 else 0.0
    ks = np.array([raw[max(0, i - 6):min(n, i + 7)].mean() for i in range(n)])
    cross = np.clip(-ks * 1.15, -0.07, 0.07)
    # corner detection uses the test-style curvature (raw over +-2 samples,
    # box-smoothed +-3): sharper peaks than ks, so hairpins stay separate
    rawk = np.zeros(n)
    for i in range(1, n - 1):
        dd = (theta[i + 1] - theta[i - 1] + math.pi) % (2 * math.pi) - math.pi
        ds = cum[i + 1] - cum[i - 1]
        rawk[i] = abs((yaw[i + 1] - yaw[i - 1] + math.pi) % (2 * math.pi) - math.pi) / ds if ds > 0.1 else 0.0
    kap = np.array([rawk[max(0, i - 3):min(n, i + 4)].mean() for i in range(n)])
    ak = kap
    # elevation
    FR = [(0.0, -0.015), (0.042, -0.040), (0.125, -0.055), (0.25, -0.065),
          (0.39, -0.085), (0.53, -0.095), (0.67, -0.100), (0.81, -0.085),
          (0.92, -0.070), (1.0, -0.055)]
    KNOTS = [(f * length, g) for f, g in FR]
    def grade_at(s):
        for (s0, g0), (s1, g1) in zip(KNOTS, KNOTS[1:]):
            if s <= s1:
                t = (s - s0) / max(1.0, s1 - s0)
                return g0 + (g1 - g0) * (0.5 - 0.5 * math.cos(math.pi * t))
        return KNOTS[-1][1]
    ys = [0.0]
    for i in range(1, n):
        ys.append(ys[-1] + grade_at((cum[i] + cum[i - 1]) * 0.5) * (cum[i] - cum[i - 1]))
    ys = np.array(ys); ys = 18.0 + (ys - ys[-1])
    # corners
    in_c = ak > (1.0 / 75.0)
    regions = []; i = 0
    while i < n:
        if in_c[i]:
            j = i
            while j + 1 < n and (in_c[j + 1] or (j + 2 < n and in_c[j + 2])): j += 1
            regions.append([i, j]); i = j + 1
        else: i += 1
    merged = []
    for r in regions:
        if merged and (r[0] - merged[-1][1]) * 4.0 < 30.0: merged[-1][1] = r[1]
        else: merged.append(r)
    def split_region(a, b):
        peaks = []
        for i in range(a, b + 1):
            if ak[i] > (1.0 / 45.0) and (i == a or ak[i] >= ak[i - 1]) and (i == b or ak[i] >= ak[i + 1]):
                if not peaks or (i - peaks[-1]) * 4.0 >= 40.0: peaks.append(i)
                elif ak[i] > ak[peaks[-1]]: peaks[-1] = i
        if len(peaks) < 2: return [[a, b]]
        cuts = []
        for p, q in zip(peaks, peaks[1:]):
            v = p + int(np.argmin(ak[p:q + 1]))
            if ak[v] < 0.55 * min(ak[p], ak[q]): cuts.append(v)
        if not cuts: return [[a, b]]
        out = []; prev = a
        for c in cuts: out.append([prev, c]); prev = c + 1
        out.append([prev, b])
        return out
    final = []
    for a, b in merged:
        for r in split_region(a, b):
            if (r[1] - r[0]) * 4.0 >= 12.0 or ak[r[0]:r[1] + 1].max() > (1.0 / 30.0):
                final.append(r)
    regions = final
    corner_of = np.zeros(n, dtype=int); infos = []
    for ci, (a, b) in enumerate(regions, start=1):
        infos.append(dict(idx=ci, a=a, b=b, s_mid=float(cum[(a + b) // 2]), dh=float(theta[b] - theta[a])))
        corner_of[a:b + 1] = ci
    def zone(s):
        f = s / length
        return ('ikaho' if f < 0.06 else 'nagamine' if f < 0.24 else 'takane' if f < 0.45
                else 'sweep' if f < 0.65 else 'lower' if f < 0.80 else 'final')
    names = []; name_ids = {}
    def nid(nm):
        if nm not in name_ids: name_ids[nm] = len(names); names.append(nm)
        return name_ids[nm]
    for nm in ["Akina Start", "Check Point 3", "Check Point 2", "Check Point 1", "Akina Goal"]: nid(nm)
    corner_name = {}; thp = fhp = nhp = 0
    for info in infos:
        z = zone(info['s_mid']); adh = abs(info['dh'])
        if adh > 2.0 and z == 'takane': thp += 1; corner_name[info['idx']] = f"Takane Hairpin {thp}"
        elif adh > 2.0 and z == 'final': fhp += 1; corner_name[info['idx']] = f"Final Hairpin {fhp}"
        elif adh > 2.0 and z == 'nagamine': nhp += 1; corner_name[info['idx']] = f"Nagamine Hairpin {nhp}"
        elif adh > 2.0: corner_name[info['idx']] = "Hairpin"
        elif z == 'ikaho': corner_name[info['idx']] = "Ikaho Bend"
        elif z == 'nagamine': corner_name[info['idx']] = None
        elif z == 'takane': corner_name[info['idx']] = "Takane Bend"
        elif z == 'lower': corner_name[info['idx']] = "Lower Bend"
        elif z == 'final': corner_name[info['idx']] = "Final Bend"
        else: corner_name[info['idx']] = "Sweep Bend"
    sweep_c = [e for e in infos if zone(e['s_mid']) == 'sweep']
    if sweep_c:
        corner_name[max(sweep_c, key=lambda e: abs(e['dh']))['idx']] = "Great Sweep"
    nag = [e for e in infos if zone(e['s_mid']) == 'nagamine']
    if nag:
        big = max(nag, key=lambda e: abs(e['dh']))
        for e in nag:
            if corner_name[e['idx']] is None:
                corner_name[e['idx']] = "Nagamine Loop" if (e is big and abs(e['dh']) > 3.6) else "Nagamine Bend"
    zone_road = {'ikaho': "Ikaho Straight", 'nagamine': "Nagamine Road", 'takane': "Takane Switchbacks",
                 'sweep': "South Ridge Road", 'lower': "Lower Touge", 'final': "Final Descent"}
    name_of = [corner_name[corner_of[i]] if corner_of[i] > 0 else zone_road[zone(float(cum[i]))] for i in range(n)]
    for frac, cpn in [(0.28, "Check Point 3"), (0.52, "Check Point 2"), (0.76, "Check Point 1")]:
        for i in range(n):
            if abs(cum[i] - frac * length) < 30.0: name_of[i] = cpn
    for i in range(n):
        if cum[i] < 4.0: name_of[i] = "Akina Start"
        if cum[i] > length - 30.0: name_of[i] = "Akina Goal"
    sample_nid = [nid(nm) for nm in name_of]
    with open(out_path, 'w') as f:
        f.write("# Akina Downhill - real-route trace (Ikaho -> Nagamine -> Takane -> south ridge)\n")
        f.write("# Point-to-point downhill: NOT a loop. Requires the 'closed 0' support in track.cpp.\n")
        f.write("# Centerline traced from the user-supplied route map photo (orange route line),\n")
        f.write("# scaled so the tightest hairpins keep radius >= ~13.5 m. Elevation is a designed\n")
        f.write("# profile in the spirit of the pass (game physics has no slope gravity).\n")
        f.write("origin 36.471300 138.873000\n")
        f.write("title Akina Downhill\nclosed 0\nstyle touge\nrunoff 2.40\n")
        f.write("half_width 5.00\ncurb_width 0.70\n")
        f.write(f"length {length:.3f}\n")
        f.write(f"spawn {pts[2][0]:.3f} {ys[2]:.3f} {pts[2][1]:.3f} {hd[2]:.6f}\n")
        f.write(f"names {len(names)}\n")
        for i, nm in enumerate(names): f.write(f"{i} {nm}\n")
        f.write(f"samples {n}\n")
        for i in range(n):
            f.write(f"{pts[i][0]:.3f} {pts[i][1]:.3f} {ys[i]:.3f} {cross[i]:.5f} 0 {corner_of[i]} {sample_nid[i]}\n")
    print(f"wrote {out_path}: {n} samples, length {length:.1f} m, corners {len(regions)}, "
          f"elevation {ys[0]:.1f} -> {ys[-1]:.1f}")

if __name__ == '__main__':
    main()
