#!/usr/bin/env python3
"""
Akina (Mt. Haruna) downhill course generator for the racing game.

Produces akina.txt in exactly the same format as suzuka.txt:
  header keys (origin / half_width / curb_width / length / spawn / names / samples)
  sample rows:  x z y cross bridge corner nameId

The course is an OPEN downhill (point-to-point), 7,552 m like the
Initial D Akina map the user supplied:
  Section 4 (start side) 2,026 m / Section 3 1,884 m /
  Section 2 1,962 m (composite S) / Section 1 1,680 m (five hairpins)

Geometry is built from straights + constant-radius arcs (analytic),
then resampled at ~4 m. Elevation is a smooth grade profile integrated
along s (all downhill). Cross slope (camber) is derived from smoothed
curvature, tilted into the corner.
"""
import math
import sys

# ---------------------------------------------------------------- elements
# Each element: ("line", length, name) or ("arc", radius, signed_deg, corner_no, name)
# signed_deg > 0 = right turn (heading increases, compass style), < 0 = left.
# heading: degrees, 0 = north (+z), 90 = east (+x). forward = (sin, cos).

SECTIONS = [
    # (section title, target length, [elements])   -- flex lines absorb the remainder
    ("Section 4", 2026.0, [
        ("line", 120.0, "Shoreline Straight"),
        ("arc", 150.0, 38.0, "First Bend"),
        ("line", 80.0, "Shoreline Straight"),
        ("arc", 90.0, -55.0, "Lakeside Bend"),
        ("line", 140.0, "Ridge Road"),
        ("arc", 70.0, 85.0, "Pine Bend"),
        ("line", 60.0, "Ridge Road"),
        ("arc", 120.0, -40.0, "Ridge S Curves"),
        ("line", 180.0, "Ridge S Curves"),
        ("arc", 48.0, -115.0, "Pine Hairpin"),
        ("line", 70.0, "West Ridge"),
        ("arc", 65.0, 95.0, "West Ridge Bend"),
        ("line", 130.0, "West Ridge"),
        ("arc", 110.0, -35.0, "Section 4 Exit"),
        ("line", 140.0, "West Ridge Descent"),
        ("arc", 60.0, 50.0, "Ridge Hairpin West"),
        ("line", 150.0, "North Ridge"),
        ("arc", 70.0, -80.0, "Ridge Drop Bend"),
        ("arc", 80.0, 30.0, "Section 4 Exit"),
        ("flex", 0.0, "Section 4 Exit"),
    ]),
    ("Section 3", 1884.0, [
        ("line", 100.0, "Upper Touge"),
        ("arc", 42.0, -135.0, "Long Hairpin"),
        ("line", 150.0, "Upper Touge"),
        ("arc", 50.0, 115.0, "Switchback Bend"),
        ("line", 120.0, "Upper Touge"),
        ("arc", 60.0, -70.0, "Cliff Bend"),
        ("line", 90.0, "Cliff Straight"),
        ("arc", 38.0, 120.0, "Rock Face Hairpin"),
        ("line", 100.0, "Cliff Straight"),
        ("arc", 70.0, -45.0, "Mid Touge Bend"),
        ("line", 220.0, "Mid Touge"),
        ("arc", 95.0, 42.0, "Mid Touge Bend"),
        ("line", 200.0, "Mid Touge"),
        ("arc", 80.0, -25.0, "Section 3 Exit"),
        ("line", 180.0, "South Descent"),
        ("arc", 55.0, -88.0, "South Hairpin"),
        ("flex", 0.0, "Section 3 Exit"),
    ]),
    ("Section 2", 1962.0, [
        ("line", 80.0, "Composite S Entrance"),
        ("arc", 55.0, -140.0, "Composite S 1"),
        ("line", 200.0, "Composite S Curves"),
        ("arc", 48.0, 165.0, "Composite S 2"),
        ("line", 160.0, "Composite S Curves"),
        ("arc", 60.0, -115.0, "Composite S 3"),
        ("line", 140.0, "Composite S Exit"),
        ("arc", 58.0, 105.0, "Valley Bend"),
        ("line", 80.0, "Valley Road"),
        ("arc", 66.0, -60.0, "Valley Bend"),
        ("line", 100.0, "Valley Road"),
        ("arc", 52.0, 80.0, "Gutter Bend"),
        ("line", 120.0, "Lower Touge"),
        ("arc", 85.0, -38.0, "Lower Touge Bend"),
        ("line", 120.0, "Lower Touge"),
        ("arc", 78.0, 36.0, "Section 2 Exit"),
        ("flex", 0.0, "Section 2 Exit"),
    ]),
    ("Section 1", 1680.0, [
        ("line", 60.0, "Hairpin Entrance"),
        ("arc", 45.0, -85.0, "Hairpin Approach"),
        ("line", 150.0, "Five Hairpins"),
        ("arc", 16.0, 160.0, "Hairpin 1"),
        ("line", 170.0, "Five Hairpins"),
        ("arc", 16.0, -160.0, "Hairpin 2"),
        ("line", 170.0, "Five Hairpins"),
        ("arc", 16.0, 160.0, "Hairpin 3"),
        ("line", 170.0, "Five Hairpins"),
        ("arc", 16.0, -160.0, "Hairpin 4"),
        ("line", 170.0, "Five Hairpins"),
        ("arc", 16.0, 160.0, "Hairpin 5"),
        ("line", 140.0, "Final Straight"),
        ("arc", 50.0, -70.0, "Final Corner"),
        ("flex", 0.0, "Goal Straight"),
    ]),
]

START_POS = (1180.0, 780.0)     # (x, z) of s = 0
START_HEADING_DEG = 268.0       # almost due west, a touch south

# grade knots: (s, grade) grade < 0 = downhill. Cosine-smoothed between knots.
GRADE_KNOTS = [
    (0.0, -0.015), (200.0, -0.045), (600.0, -0.065), (1100.0, -0.055),
    (1600.0, -0.075), (2026.0, -0.070), (2500.0, -0.085), (3100.0, -0.075),
    (3600.0, -0.090), (3910.0, -0.080), (4400.0, -0.065), (5000.0, -0.080),
    (5500.0, -0.070), (5872.0, -0.075), (6200.0, -0.095), (6800.0, -0.100),
    (7200.0, -0.085), (7552.0, -0.060),
]
FINISH_Y = 18.0

NAMES_EXTRA = ["Akina Start", "Check Point 3", "Check Point 2", "Check Point 1", "Akina Goal"]


def element_length(el):
    if el[0] == "arc":
        return el[1] * math.radians(abs(el[2]))
    return el[1]


def build_points():
    """Integrate elements at 0.5 m steps. Returns list of dicts."""
    # resolve flex lengths per section first
    sections = []
    for title, target, elements in SECTIONS:
        fixed = sum(element_length(e) for e in elements if e[0] != "flex")
        flex = target - fixed
        if flex < 40.0:
            raise SystemExit(f"{title}: flex too small ({flex:.1f})")
        sections.append((title, target, [
            ("line", flex, e[2]) if e[0] == "flex" else e for e in elements]))

    pts = []
    corner_counter = [0]
    x, z = START_POS
    heading = math.radians(START_HEADING_DEG)
    s = 0.0
    name_ids = {}
    names = []

    def name_id(name):
        if name not in name_ids:
            name_ids[name] = len(names)
            names.append(name)
        return name_ids[name]

    for name in NAMES_EXTRA:
        name_id(name)

    ds = 0.5
    section_bounds = []  # (s_start, s_end, title)
    for title, target, elements in sections:
        sec_start = s
        for el in elements:
            if el[0] == "line":
                length, nm = el[1], el[2]
                corner = 0
                nid = name_id(nm)
                steps = max(1, int(round(length / ds)))
                step = length / steps
                for _ in range(steps):
                    x += math.sin(heading) * step
                    z += math.cos(heading) * step
                    s += step
                    pts.append(dict(x=x, z=z, s=s, heading=heading, kappa=0.0,
                                    corner=corner, nameId=nid))
            else:
                _, radius, adeg, nm = el
                corner_counter[0] += 1
                corner = corner_counter[0]
                nid = name_id(nm)
                length = radius * math.radians(abs(adeg))
                kappa = (1.0 / radius) * (1.0 if adeg > 0 else -1.0)
                steps = max(2, int(round(length / ds)))
                step = length / steps
                dh = math.radians(adeg) / steps
                for _ in range(steps):
                    heading += dh * 0.5
                    x += math.sin(heading) * step
                    z += math.cos(heading) * step
                    heading += dh * 0.5
                    s += step
                    pts.append(dict(x=x, z=z, s=s, heading=heading, kappa=kappa,
                                    corner=corner, nameId=nid))
        section_bounds.append((sec_start, s, title))
    return pts, names, name_ids, section_bounds


def grade_at(s):
    knots = GRADE_KNOTS
    if s <= knots[0][0]:
        return knots[0][1]
    for (s0, g0), (s1, g1) in zip(knots, knots[1:]):
        if s <= s1:
            t = (s - s0) / (s1 - s0)
            w = 0.5 - 0.5 * math.cos(math.pi * t)
            return g0 + (g1 - g0) * w
    return knots[-1][1]


def smooth(values, radius):
    out = []
    n = len(values)
    for i in range(n):
        lo = max(0, i - radius)
        hi = min(n, i + radius + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "akina.txt"
    pts, names, name_ids, section_bounds = build_points()
    total = pts[-1]["s"]

    # resample at ~4 m along the dense polyline (linear in index space is fine:
    # dense steps are uniform 0.5 m in s by construction, last step of each
    # element may differ slightly -- acceptable, s is recomputed from x/z anyway)
    target_spacing = 4.0
    count = int(round(total / target_spacing))  # segments
    samples = []
    for i in range(count + 1):
        t = i / count * (len(pts) - 1)
        i0 = int(math.floor(t))
        i1 = min(i0 + 1, len(pts) - 1)
        f = t - i0
        a, b = pts[i0], pts[i1]
        samples.append(dict(
            x=a["x"] + (b["x"] - a["x"]) * f,
            z=a["z"] + (b["z"] - a["z"]) * f,
            heading=a["heading"] + (b["heading"] - a["heading"]) * f,
            kappa=a["kappa"] + (b["kappa"] - a["kappa"]) * f,
            corner=a["corner"] if f < 0.5 else b["corner"],
            nameId=a["nameId"] if f < 0.5 else b["nameId"],
        ))

    # true cumulative s from x/z
    cum = [0.0]
    for a, b in zip(samples, samples[1:]):
        cum.append(cum[-1] + math.hypot(b["x"] - a["x"], b["z"] - a["z"]))
    length = cum[-1]

    # elevation: integrate grade over true s, then place finish at FINISH_Y
    ys = [0.0]
    for i in range(1, len(samples)):
        ds = cum[i] - cum[i - 1]
        ys.append(ys[-1] + grade_at((cum[i] + cum[i - 1]) * 0.5) * ds)
    drop = -ys[-1]
    for i in range(len(samples)):
        samples[i]["y"] = FINISH_Y + (ys[i] - ys[-1])
        samples[i]["s"] = cum[i]

    # camber from smoothed curvature: tilt into the corner.
    kappas = smooth([p["kappa"] for p in samples], 6)
    for p, k in zip(samples, kappas):
        p["cross"] = max(-0.07, min(0.07, -k * 1.15))

    # section boundary checkpoints: name the first sample of each later
    # section as its check point for a few samples so the HUD shows it.
    bounds = [b[0] for b in section_bounds]  # dense-s boundaries ~ section starts
    # recompute section starts in true s by proportion (dense s == true s closely)
    sec_start_s = [0.0, 2026.0, 3910.0, 5872.0]
    cp_names = [None, "Check Point 3", "Check Point 2", "Check Point 1"]
    for start_s, cp in zip(sec_start_s, cp_names):
        if cp is None:
            continue
        for p in samples:
            if abs(p["s"] - start_s) < 30.0:
                p["nameId"] = name_ids[cp]
    samples[0]["nameId"] = name_ids["Akina Start"]
    for p in samples:
        if p["s"] > length - 30.0:
            p["nameId"] = name_ids["Akina Goal"]

    # spawn: 10 m into the course, on sample ~2-3
    spawn = samples[2]
    spawn_yaw = samples[2]["heading"]

    with open(out_path, "w") as f:
        f.write("# Akina Downhill - Mt. Haruna (Harunasan) touge, after the Initial D course map\n")
        f.write("# Point-to-point downhill: NOT a loop. Requires the 'closed 0' support in track.cpp.\n")
        f.write("# Sections (from the map): total 7552 m = Sec.4 2026 m + Sec.3 1884 m + Sec.2 1962 m + Sec.1 1680 m\n")
        f.write("# Elevation drop is a designed profile in the spirit of the pass (game physics has no slope gravity).\n")
        f.write("origin 36.471300 138.873000\n")
        f.write("title Akina Downhill\n")
        f.write("closed 0\n")
        f.write("half_width 5.00\n")
        f.write("curb_width 0.70\n")
        f.write(f"length {length:.3f}\n")
        f.write(f"spawn {spawn['x']:.3f} {spawn['y']:.3f} {spawn['z']:.3f} {spawn_yaw:.6f}\n")
        f.write(f"names {len(names)}\n")
        for i, nm in enumerate(names):
            f.write(f"{i} {nm}\n")
        f.write(f"samples {len(samples)}\n")
        for p in samples:
            f.write(f"{p['x']:.3f} {p['z']:.3f} {p['y']:.3f} {p['cross']:.5f} "
                    f"0 {p['corner']} {p['nameId']}\n")

    print(f"wrote {out_path}: samples={len(samples)} length={length:.1f} "
          f"drop={drop:.1f} y {samples[0]['y']:.1f} -> {samples[-1]['y']:.1f}")
    print(f"bounds x [{min(p['x'] for p in samples):.0f}, {max(p['x'] for p in samples):.0f}] "
          f"z [{min(p['z'] for p in samples):.0f}, {max(p['z'] for p in samples):.0f}]")
    for (s0, s1, title) in section_bounds:
        print(f"  {title}: dense s {s0:.0f}..{s1:.0f} ({s1 - s0:.0f} m)")
    # minimum distance between far-apart parts of the course (self-proximity)
    worst = (1e9, None)
    n = len(samples)
    for i in range(n):
        for j in range(i + 60, n):
            d = math.hypot(samples[i]["x"] - samples[j]["x"], samples[i]["z"] - samples[j]["z"])
            if d < worst[0]:
                worst = (d, (i, j, samples[i]["s"], samples[j]["s"]))
    print(f"min plan distance between samples >240 m apart in s: {worst[0]:.1f} m at {worst[1]}")


if __name__ == "__main__":
    main()
