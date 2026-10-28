#!/usr/bin/env python3
"""
Akina (Mt. Haruna) downhill course generator for the racing game.

Produces akina.txt in exactly the same format as suzuka.txt:
  header keys (origin / half_width / curb_width / length / spawn / names / samples)
  sample rows:  x z y cross bridge corner nameId

The course is an OPEN downhill (point-to-point) laid out after the
Initial D style Akina course map the user supplied (2026-10-28):
total 7,552 m with sections 1,680 / 1,692 / 1,884 / 2,026 m, starting
at the BOTTOM-LEFT (south) end of the map and finishing at the
top-right (north-east) end, with three check points in between.
The map's section lengths sum to 7,282 m, so the design is built at
x1.037 to match the printed total of 7,552 m.

(A pixel trace of this diagram was attempted first: the drawn line is
a white core inside a green glow, and the glow merges the tightest
switchback clusters at this resolution, so the layout is instead
built analytically from straights + constant-radius arcs following
the map's shape section by section.)

Geometry is built from straights + constant-radius arcs (analytic),
then resampled at ~4 m. Elevation is a smooth grade profile integrated
along s (all downhill). Cross slope (camber) is derived from smoothed
curvature, tilted into the corner.
"""
import math
import sys

# ---------------------------------------------------------------- elements
# Each element: ("line", length, name) or ("arc", radius, signed_deg, name)
# signed_deg > 0 = right turn (heading increases, compass style), < 0 = left.
# heading: degrees, 0 = north (+z), 90 = east (+x). forward = (sin, cos).
# Section targets are the map's printed lengths x1.037 (see docstring).

SECTIONS = [
    ("Section 1", 1742.0, [
        ("line", 100.0, "Akina South"),
        ("arc", 140.0, 20.0, "South Bend"),
        ("line", 85.0, "Akina South"),
        ("arc", 105.0, -32.0, "South S 1"),
        ("line", 95.0, "South Climb"),
        ("arc", 95.0, 36.0, "South S 2"),
        ("line", 80.0, "South Climb"),
        ("arc", 120.0, -14.0, "Ridge Approach"),
        ("line", 70.0, "Goren Approach"),
        ("arc", 16.0, 158.0, "Goren Hairpin 1"),
        ("line", 30.0, "Goren Hairpins"),
        ("arc", 16.0, -158.0, "Goren Hairpin 2"),
        ("line", 96.0, "Goren Hairpins"),
        ("arc", 16.0, 158.0, "Goren Hairpin 3"),
        ("line", 30.0, "Goren Hairpins"),
        ("arc", 16.0, -158.0, "Goren Hairpin 4"),
        ("line", 96.0, "Goren Hairpins"),
        ("arc", 16.0, 158.0, "Goren Hairpin 5"),
        ("line", 34.0, "Goren Hairpins"),
        ("arc", 38.0, -120.0, "Goren Exit Bend"),
        ("line", 90.0, "Goren Exit"),
        ("arc", 65.0, -95.0, "West Jog"),
        ("line", 130.0, "West Climb"),
        ("arc", 90.0, 55.0, "Section 1 Exit Bend"),
        ("flex", 0.0, "Section 1 Exit"),
    ]),
    ("Section 2", 1755.0, [
        ("line", 60.0, "Section 2 Climb"),
        ("arc", 100.0, 78.0, "A Swing"),
        ("line", 660.0, "Strand A"),
        ("arc", 40.0, -172.0, "Curl"),
        ("line", 580.0, "Strand M"),
        ("arc", 60.0, 80.0, "Rise Bend"),
        ("flex", 0.0, "Section 2 Exit"),
    ]),
    ("Section 3", 1954.0, [
        ("arc", 45.0, 100.0, "B Swing"),
        ("line", 390.0, "Strand B"),
        ("arc", 110.0, 30.0, "B Dip"),
        ("line", 250.0, "Strand B Low"),
        ("arc", 50.0, -150.0, "East Curl"),
        ("line", 150.0, "NW Climb"),
        ("arc", 85.0, 52.0, "Climb Bend"),
        ("arc", 60.0, 75.0, "Traverse Entry"),
        ("line", 100.0, "Traverse"),
        ("arc", 60.0, -75.0, "Traverse Exit"),
        ("line", 60.0, "North Climb"),
        ("arc", 90.0, -28.0, "Climb S"),
        ("line", 90.0, "North Climb"),
        ("arc", 95.0, -48.0, "Summit Traverse"),
        ("line", 90.0, "Summit Approach"),
        ("arc", 85.0, 70.0, "Summit Base Bend"),
        ("flex", 0.0, "Section 3 Exit"),
    ]),
    ("Section 4", 2101.0, [
        ("line", 60.0, "Summit Entry"),
        ("arc", 17.0, 158.0, "Summit Hairpin 1"),
        ("line", 84.0, "Summit Switchbacks"),
        ("arc", 17.0, -160.0, "Summit Hairpin 2"),
        ("line", 78.0, "Summit Switchbacks"),
        ("arc", 18.0, 155.0, "Summit Hairpin 3"),
        ("line", 64.0, "Summit Switchbacks"),
        ("arc", 17.0, -150.0, "Summit Hairpin 4"),
        ("line", 60.0, "Summit Top"),
        ("arc", 80.0, 95.0, "Top Sweep"),
        ("line", 230.0, "North Ridge"),
        ("arc", 100.0, -25.0, "Ridge Bend"),
        ("line", 130.0, "Final Run"),
        ("arc", 75.0, 105.0, "Hook Entry"),
        ("line", 230.0, "Hook Drop"),
        ("arc", 75.0, 90.0, "Hook Turn"),
        ("line", 190.0, "West Strand"),
        ("arc", 75.0, 95.0, "Hook Curl"),
        ("arc", 90.0, 80.0, "Goal Swing"),
        ("flex", 0.0, "Goal Straight"),
    ]),
]

START_POS = (-1000.0, -1250.0)  # (x, z) of s = 0: bottom-left (south) end
START_HEADING_DEG = -4.0        # just west of north up the start tail

# grade knots: (s, grade) grade < 0 = downhill. Cosine-smoothed between knots.
GRADE_KNOTS = [
    (0.0, -0.015), (800.0, -0.045), (2000.0, -0.065), (3500.0, -0.085),
    (5000.0, -0.095), (6200.0, -0.085), (7552.0, -0.060),
]
FINISH_Y = 18.0

NAMES_EXTRA = ["Akina Start", "Check Point 3", "Check Point 2", "Check Point 1", "Akina Goal"]

# Check point s positions, from the red dots on the map (fractions of the
# traced map line: ~0.218 / ~0.386 / ~0.797 of the total length).
CHECKPOINTS = [(1629.0, "Check Point 3"), (3571.0, "Check Point 2"), (5338.0, "Check Point 1")]


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

    # resample at ~4 m along the dense polyline
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

    # check points at the map's red-dot positions (name shown +-30 m around).
    for cp_s, cp in CHECKPOINTS:
        for p in samples:
            if abs(p["s"] - cp_s) < 30.0:
                p["nameId"] = name_ids[cp]
    samples[0]["nameId"] = name_ids["Akina Start"]
    for p in samples:
        if p["s"] > length - 30.0:
            p["nameId"] = name_ids["Akina Goal"]

    # spawn: 10 m into the course, on sample ~2-3
    spawn = samples[2]
    spawn_yaw = samples[2]["heading"]

    with open(out_path, "w") as f:
        f.write("# Akina Downhill - Mt. Haruna touge, after the Initial D course map (7,552 m diagram)\\n")
        f.write("# Point-to-point downhill: NOT a loop. Requires the 'closed 0' support in track.cpp.\\n")
        f.write("# Sections (map figures, built x1.037 to match the printed total):\\n")
        f.write("# Sec.1 1680 m + Sec.2 1692 m + Sec.3 1884 m + Sec.4 2026 m = 7552 m.\\n")
        f.write("# Start at the map's bottom-left (south) end, finish at the top-right;\\n")
        f.write("# check points at the map's three red dots.\\n")
        f.write("# Elevation drop is a designed profile in the spirit of the pass (game physics has no slope gravity).\\n")
        f.write("origin 36.471300 138.873000\n")
        f.write("title Akina Downhill\n")
        f.write("closed 0\n")
        f.write("style touge\n")
        f.write("runoff 2.40\n")
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
