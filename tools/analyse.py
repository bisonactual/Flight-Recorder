"""Analyse flight recorder evidence runs.

For each interrupted run, against the uninterrupted reference:
  - coverage: the farthest any point of the reference toolpath inside the
    stock (below work Z 0) lies from where the interrupted run went inside the
    stock (all boots, re-entry included). Near zero means the paths match: no
    part of the cut was left out. It says nothing of order or spindle state;
  - jumps: consecutive samples farther apart than one sample can move. The
    path isn't interpolated across them, and there should be none;
  - for a power cut: where the machine really was when the power went (last
    stepper sample) against the line the record names. On that line means
    exact; on a later line means the record lagged; on an earlier line would
    mean it ran ahead, which must never happen;
  - a plot of machine Z over time around the interruption.

Usage: analyse.py <runs-dir> <program.nc> <out-dir> reference <run> [<run> ...]
"""

import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

from stream import load_program, WORD

STEPS_PER_MM = 250.0
G54 = np.array([-300.0, -300.0, -60.0])
MAX_STEP = 1.5      # mm: farther than one 10 ms sample moves (6000 mm/min is 1 mm)


def read_steps(path):
    """Stepper samples: (sim time s, machine x, y, z mm)."""
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        t, rest = line.split(None, 1)
        x, y, z = (int(v) for v in rest.split(","))
        rows.append((float(t), x / STEPS_PER_MM, y / STEPS_PER_MM, z / STEPS_PER_MM))
    return np.array(rows) if rows else np.zeros((0, 4))


def boots(run):
    out = []
    n = 1
    while os.path.exists(os.path.join(run, f"boot{n}.steps.log")):
        out.append(read_steps(os.path.join(run, f"boot{n}.steps.log")))
        n += 1
    return out


def densify(points, step=0.02):
    """Polyline -> points every `step` mm (for nearest-distance queries), except across
    a jump: interpolating there would invent motion nobody sampled."""
    out = [points[:1]]
    for a, b in zip(points[:-1], points[1:]):
        d = float(np.linalg.norm(b - a))
        n = int(math.ceil(d / step))
        if n > 1 and d <= MAX_STEP:
            t = np.linspace(0, 1, n + 1)[1:, None]
            out.append(a + (b - a) * t)
        else:
            out.append(b[None, :])
    return np.vstack(out)


def jumps(points):
    return int((np.linalg.norm(np.diff(points, axis=0), axis=1) > MAX_STEP).sum()) if len(points) > 1 else 0


def in_stock(points):
    """Only below work Z 0 does the tool cut: rapids and retracts above it can't."""
    return points[points[:, 2] < G54[2] - 1e-6]


def program_segments(path):
    """Per program line with motion: (line, polyline of machine points)."""
    _, lines = load_program(path)
    pos = None
    motion = "G0"
    segs = []
    for number, text in lines:
        words = WORD.findall(text)
        for letter, value in words:
            if letter == "G" and value.split(".")[0] in ("0", "1", "2", "3", "00", "01", "02", "03"):
                motion = "G" + str(int(float(value)))
        target = dict(zip("XYZ", pos)) if pos is not None else {}
        moved = False
        for letter, value in words:
            if letter in "XYZ":
                target[letter] = float(value)
                moved = True
        if not moved or "G53" in text:
            continue
        if pos is None or len(target) < 3:
            pos = [target.get(a, 0.0) for a in "XYZ"]
            continue
        end = np.array([target[a] for a in "XYZ"])
        start = np.array(pos)
        if motion in ("G2", "G3"):
            ij = dict(words)
            cx, cy = start[0] + float(ij.get("I", 0)), start[1] + float(ij.get("J", 0))
            a0 = math.atan2(start[1] - cy, start[0] - cx)
            a1 = math.atan2(end[1] - cy, end[0] - cx)
            if motion == "G3":
                while a1 <= a0:
                    a1 += 2 * math.pi
            else:
                while a1 >= a0:
                    a1 -= 2 * math.pi
            r = math.hypot(start[0] - cx, start[1] - cy)
            n = max(8, int(abs(a1 - a0) * r / 0.05))
            t = np.linspace(0, 1, n + 1)
            ang = a0 + (a1 - a0) * t
            pts = np.column_stack([cx + r * np.cos(ang), cy + r * np.sin(ang), start[2] + (end[2] - start[2]) * t])
        else:
            pts = np.vstack([start, end])
        segs.append((number, pts + G54))
        pos = list(end)
    return segs


def point_to_polyline(p, pts):
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        L = float(ab @ ab)
        t = 0.0 if L == 0 else max(0.0, min(1.0, float((p - a) @ ab) / L))
        best = min(best, float(np.linalg.norm(a + t * ab - p)))
    return best


def where_on_program(p, segs, line, window=60):
    """Which program line the machine point p lies on, near the recorded line."""
    idx = next(i for i, (n, _) in enumerate(segs) if n >= line)
    cands = []
    for i in range(max(0, idx - window), min(len(segs), idx + window)):
        cands.append((point_to_polyline(p, segs[i][1]), segs[i][0]))
    return min(cands)


def main():
    runs_dir, program, out_dir = sys.argv[1:4]
    ref_name, names = sys.argv[4], sys.argv[5:]
    os.makedirs(out_dir, exist_ok=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ref = np.vstack(boots(os.path.join(runs_dir, ref_name)))
    ref_pts = in_stock(densify(ref[:, 1:4]))
    segs = program_segments(program)
    report = {ref_name: {"jumps": jumps(ref[:, 1:4])}}

    for name in names:
        run = os.path.join(runs_dir, name)
        s = json.load(open(os.path.join(run, "summary.json")))
        bs = boots(run)
        cut = in_stock(np.vstack([densify(b[:, 1:4]) for b in bs if len(b)]))
        dist, _ = cKDTree(cut).query(ref_pts)
        r = {
            "jumps": sum(jumps(b[:, 1:4]) for b in bs),
            "scenario": s["scenario"], "via": s["via"],
            "record": s.get("record_before_resume"),
            "resumed_from": s.get("resume_first_line", "")[:40],
            "resume": s.get("resume"),
            "final_record": s.get("record_after_resume"),
            "coverage_max_mm": round(float(dist.max()), 4),
            "coverage_p99_mm": round(float(np.percentile(dist, 99)), 4),
        }
        if s["scenario"] == "power" and len(bs[0]):
            last = bs[0][-1]
            d, line = where_on_program(last[1:4], segs, s["record_before_resume"]["ln"])
            r["power_cut"] = {"last_position": [round(v, 3) for v in last[1:4]], "on_line": line,
                              "distance_mm": round(d, 4), "recorded_line": s["record_before_resume"]["ln"],
                              "verdict": "exact" if line == s["record_before_resume"]["ln"] else
                                         ("lagged" if line > s["record_before_resume"]["ln"] else "AHEAD")}
        report[name] = r

        # Z over time, all boots back to back.
        fig, ax = plt.subplots(figsize=(9, 3.2))
        offset = 0.0
        for k, b in enumerate(bs):
            if len(b):
                # No samples while the machine is idle: break the line there
                # rather than draw motion that didn't happen.
                t, z = b[:, 0] + offset, b[:, 3].copy()
                gaps = np.where(np.diff(t) > 0.05)[0] + 1
                t, z = np.insert(t, gaps, np.nan), np.insert(z, gaps, np.nan)
                ax.plot(t, z, lw=0.8, label=f"boot {k + 1} (gaps: idle, no motion)")
                offset += b[-1, 0] + 2.0
        ax.axhline(-20.0, color="tab:red", lw=0.8, ls="--", label="retract height (machine Z -20)")
        ax.set_xlabel("simulated time, s (boots placed end to end)")
        ax.set_ylabel("machine Z, mm")
        ax.set_title(f"{name}: {s['scenario']} via {s['via']}, record {r['record']['state'] if r['record'] else '-'}"
                     f" / {r['record']['why'] if r['record'] else '-'} at line {r['record']['ln'] if r['record'] else '-'}")
        ax.legend(fontsize=7, loc="lower right")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"{name}-z.png"), dpi=120)
        plt.close(fig)

    with open(os.path.join(out_dir, "analysis.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
