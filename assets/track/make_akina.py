#!/usr/bin/env python3
"""
Akina (Mt. Haruna) downhill course generator for the racing game.

Produces akina.txt in exactly the same format as suzuka.txt:
  header keys (origin / closed / style / runoff / half_width / curb_width /
               length / spawn / names / samples)
  sample rows:  x z y cross bridge corner nameId

REBUILT 2026-10-29 from the REAL road, after the user questioned whether
the previous course (an analytic design loosely following the Initial D
course diagram) was really Akina. Investigation summary:
  * The fictional Akina downhill is Mt. Haruna's Gunma Prefectural
    Route 33 (Shibukawa-Matsuida line), from the water tower / old toll
    gate at Yaseone Pass by Lake Haruna down toward Ikaho Onsen.
  * SEGA's Arcade Stage data (7,552 m, sections 1,680 / 1,962 / 1,884 /
    2,026 m) states the arcade layout reproduces the real road as-is.
  * The full real road from the lake junction to Ikaho is ~10.5 km;
    this course takes the first 7,552 m from the Yaseone start, i.e.
    the same segment the games use, with real corner radii.
  * The famous consecutive hairpins sit in the upper mountain,
    roughly 2.0-3.3 km into the run (in reality 4 hairpins; the
    fiction counts 5).
  * Real elevation drop over the segment is ~370 m (start ~1,111 m),
    gentler than the old fictional profile's 543 m.

Data provenance (embedded below, no network needed):
  * Centreline: OpenStreetMap Route 33 ways (ODbL), lake->Ikaho chain,
    extracted 2026-10-29. Coordinates are metres east / -north of the
    origin (36.4713, 138.873), quantized to 0.05 m, zlib+base64.
  * Elevation: GSI (Geospatial Information Authority of Japan)
    elevation API, sampled every 150 m along the route.

Geometry is smoothed lightly (OSM nodes are noisy), cropped to
7,552 m, resampled at 4 m. Cross slope (camber) is derived from
smoothed signed curvature, banked into the corner, clamped to 0.07.
Elevation is made monotonically non-increasing (local DEM noise and
tiny rises are flattened), with the goal set to y=18 like before.
"""
import base64
import math
import sys
import zlib

PATH_BLOB = (
    "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"
)
ELEV_BLOB = (
    "eNolkkuWBCEIBC9UC0Dk8+b+95oMe5OmlE1I2vb5te+Yfam1tLbW1eom44E7uIsr3OBWLlwuApe4i2vc/JpKXnv6HwAHwgFxYCSM"
    "hJEwEkbCSBgJ48K4MC6MC+PCuPO7sARGwag3BIyCUTAaRsNoGA2jYTSMhjEwBsbAGBgDY+YXhgTGwlgYC2NfVL+sXlj20rIXlwr2"
    "526en/Q8vYH287tST3z4lR7nawaVe0pazpkaKm3x9FDZp+GcLH6rpNDT9Hn9f93CHsUHouu8beiMDZ2tj+pWl/otNJuvZ4pZRhRz"
    "7mm2fDVvjRv3W8Wk6U8oh1SN6yoWbU/KRa7yKpKr+GZHQa7oMwpj9BTflAafi+Q93+gfofwFHx+JacBebVsP8LUeSmKuZ9NUCl21"
    "VL8+GqzjaBuxvHF+7bpLW60eX61qPf8B7PyQVw=="
)

COURSE_LENGTH = 7552.0
START_LAT, START_LON = 36.4744319, 138.8871979  # water tower / old toll gate
ORIGIN_LAT, ORIGIN_LON = 36.4713, 138.8730
HALF_WIDTH = 6.00
CURB_WIDTH = 0.70
RUNOFF = 2.40
SECTIONS = [1680.0, 1962.0, 1884.0, 2026.0]  # arcade figures, travel order


def decode(blob):
    raw = zlib.decompress(base64.b64decode(blob)).decode()
    a, b = raw.split(";")
    return [int(v) for v in a.split(",")], [int(v) for v in b.split(",")]


def main():
    px, pz = decode("".join(PATH_BLOB.split()))
    pts = [(x / 20.0, z / 20.0) for x, z in zip(px, pz)]
    es, ee = decode("".join(ELEV_BLOB.split()))
    elev_s = [v for v in es]
    elev_v = [v / 10.0 for v in ee]

    # cumulative distance along the OSM chain
    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))

    # start offset: chain point nearest the Yaseone start
    sx = (START_LON - ORIGIN_LON) * 89455.0
    sz = -(START_LAT - ORIGIN_LAT) * 110574.0
    i0 = min(range(len(pts)), key=lambda i: (pts[i][0] - sx) ** 2 + (pts[i][1] - sz) ** 2)
    s_start = s[i0]
    print(f"start chain point {i0} at s={s_start:.0f} m (elev {interp(elev_s, elev_v, s_start):.0f} m)")

    # crop and densify to 2 m
    dense = []
    target = s_start
    i = i0
    while target <= s_start + COURSE_LENGTH + 200.0 and i + 1 < len(pts):
        while i + 1 < len(pts) and s[i + 1] < target:
            i += 1
        if i + 1 >= len(pts):
            break
        t = (target - s[i]) / max(s[i + 1] - s[i], 1e-6)
        dense.append((pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t,
                      pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t,
                      target - s_start))
        target += 2.0
    # smooth plan geometry: two moving-average passes, 7-pt window (14 m)
    for _ in range(3):
        dense = smooth_dense(dense, 9)

    # resample at 4 m with arc-length recomputed after smoothing
    samples = resample(dense, 4.0)
    # trim to exactly COURSE_LENGTH (smoothing shortens the chain a little)
    samples = [p for p in samples if p[2] <= COURSE_LENGTH - 0.5]
    a = samples[-1]
    b = next(p for p in resample(dense, 4.0) if p[2] > COURSE_LENGTH)
    t = (COURSE_LENGTH - a[2]) / max(b[2] - a[2], 1e-9)
    samples.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, COURSE_LENGTH))
    total = samples[-1][2]
    print(f"resampled: {len(samples)} samples, length {total:.1f} m")

    # elevation: interpolate GSI profile, shift so goal = 18, monotone
    ys = []
    for (_, _, ss) in samples:
        ys.append(interp(elev_s, elev_v, s_start + ss))
    y_goal = ys[-1]
    ys = [y - y_goal + 18.0 for y in ys]
    for i in range(1, len(ys)):
        ys[i] = min(ys[i], ys[i - 1])
    print(f"elevation: start {ys[0]:.1f} -> goal {ys[-1]:.1f} (drop {ys[0] - ys[-1]:.1f} m)")

    # headings + smoothed signed curvature + camber
    n = len(samples)
    heads = []
    for i in range(n):
        a = samples[max(i - 1, 0)]
        b = samples[min(i + 1, n - 1)]
        heads.append(math.atan2(b[0] - a[0], b[1] - a[1]))
    kappa = [0.0] * n
    win = 4  # +/-16 m
    for i in range(n):
        a = heads[max(i - win, 0)]
        b = heads[min(i + win, n - 1)]
        d = wrap_angle(b - a)
        dist = samples[min(i + win, n - 1)][2] - samples[max(i - win, 0)][2]
        kappa[i] = d / max(dist, 1.0)
    kappa = smooth_list(kappa, 9)
    min_r = 1.0 / max(abs(k) for k in kappa)
    print(f"tightest radius after smoothing: {min_r:.1f} m")

    # corner peaks: local maxima of |kappa| with R < 65 m, merged < 120 m
    peaks = []
    for i in range(2, n - 2):
        if abs(kappa[i]) >= abs(kappa[i - 1]) and abs(kappa[i]) >= abs(kappa[i + 1]) and abs(kappa[i]) > 1.0 / 80.0:
            if peaks and samples[i][2] - peaks[-1][0] < 90.0:
                if abs(kappa[i]) > abs(kappa[peaks[-1][1]]):
                    peaks[-1] = (samples[i][2], i)
            else:
                peaks.append((samples[i][2], i))
    print(f"corners detected: {len(peaks)}")

    names = ["Akina Start"]
    name_at = {0: 0}
    cp_s = []
    acc = 0.0
    for sec in SECTIONS[:-1]:
        acc += sec
        cp_s.append(acc)
    for k, cs in enumerate(cp_s):
        names.append(f"Check Point {k + 1}")
        idx = min(range(n), key=lambda i: abs(samples[i][2] - cs))
        name_at[idx] = len(names) - 1
    hairpin_no = 0
    corner_no = 0
    corner_of = {}
    for (ps, pi) in peaks:
        corner_no += 1
        corner_of[pi] = corner_no
        r = 1.0 / abs(kappa[pi])
        if 1800.0 <= ps <= 3700.0 and r < 30.0:
            hairpin_no += 1
            names.append(f"Five Hairpins {hairpin_no}" if hairpin_no > 1 else "Five Hairpins 1")
        elif r < 22.0:
            names.append(f"Hairpin C{corner_no}")
        elif r < 40.0:
            names.append(f"Corner C{corner_no}")
        else:
            names.append(f"Sweep C{corner_no}")
        name_at[pi] = len(names) - 1
    names.append("Akina Goal")
    name_at[n - 1] = len(names) - 1
    print(f"five-hairpin cluster corners named: {hairpin_no}")

    spawn = samples[2]
    spawn_yaw = heads[2]
    out = sys.argv[1] if len(sys.argv) > 1 else "akina.txt"
    with open(out, "w") as f:
        f.write("# Akina Downhill - REAL Mt. Haruna (Gunma Prefectural Route 33),\n")
        f.write("# traced from OpenStreetMap centreline + GSI elevation (2026-10-29).\n")
        f.write("# Point-to-point downhill: NOT a loop. Requires the 'closed 0' support in track.cpp.\n")
        f.write("# Start: Yaseone Pass water tower (old toll gate) by Lake Haruna; 7,552 m segment\n")
        f.write("# of the real road toward Ikaho (full real road to Ikaho is ~10.5 km).\n")
        f.write("# Sections (arcade figures): 1,680 + 1,962 + 1,884 + 2,026 = 7,552 m.\n")
        f.write("# Real elevation drop ~370 m. Game physics has no slope gravity.\n")
        f.write(f"origin {ORIGIN_LAT:.6f} {ORIGIN_LON:.6f}\n")
        f.write("title Akina Downhill\n")
        f.write("closed 0\n")
        f.write("style touge\n")
        f.write(f"runoff {RUNOFF:.2f}\n")
        f.write(f"half_width {HALF_WIDTH:.2f}\n")
        f.write(f"curb_width {CURB_WIDTH:.2f}\n")
        f.write(f"length {total:.3f}\n")
        f.write(f"spawn {spawn[0]:.3f} {ys[2]:.3f} {spawn[1]:.3f} {spawn_yaw:.6f}\n")
        f.write(f"names {len(names)}\n")
        for i, nm in enumerate(names):
            f.write(f"{i} {nm}\n")
        f.write(f"samples {n}\n")
        for i in range(n):
            cross = max(-0.07, min(0.07, kappa[i] * 1.15))
            f.write(f"{samples[i][0]:.3f} {samples[i][1]:.3f} {ys[i]:.3f} {cross:.5f} "
                    f"0 {corner_of.get(i, 0)} {name_at.get(i, -1)}\n")
    print(f"wrote {out}")


def interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / max(xs[hi] - xs[lo], 1e-9)
    return ys[lo] + (ys[hi] - ys[lo]) * t


def smooth_dense(dense, w):
    h = w // 2
    out = []
    for i in range(len(dense)):
        sx = sz = 0.0
        cnt = 0
        for j in range(max(0, i - h), min(len(dense), i + h + 1)):
            sx += dense[j][0]
            sz += dense[j][1]
            cnt += 1
        out.append((sx / cnt, sz / cnt, dense[i][2]))
    return out


def smooth_list(v, w):
    h = w // 2
    return [sum(v[max(0, i - h):i + h + 1]) / len(v[max(0, i - h):i + h + 1]) for i in range(len(v))]


def resample(dense, step):
    s = [0.0]
    for i in range(1, len(dense)):
        s.append(s[-1] + math.hypot(dense[i][0] - dense[i - 1][0], dense[i][1] - dense[i - 1][1]))
    out = []
    target = 0.0
    i = 0
    while target <= s[-1] and i + 1 < len(dense):
        while i + 1 < len(dense) and s[i + 1] < target:
            i += 1
        if i + 1 >= len(dense):
            break
        t = (target - s[i]) / max(s[i + 1] - s[i], 1e-9)
        out.append((dense[i][0] + (dense[i + 1][0] - dense[i][0]) * t,
                    dense[i][1] + (dense[i + 1][1] - dense[i][1]) * t,
                    target))
        target += step
    out.append((dense[-1][0], dense[-1][1], s[-1]))
    return out


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


if __name__ == "__main__":
    main()
