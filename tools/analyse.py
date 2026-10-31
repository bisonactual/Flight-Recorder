"""Analyses the flight recorder evidence runs.

Each interrupted run is compared with the uninterrupted reference:
  - coverage, the farthest any point of the reference toolpath inside the stock
    (below work Z 0) is from anywhere the interrupted run went inside the stock,
    counting all boots and the move back in. Close to zero means the paths
    match and no part of the cut was left out. It says nothing about the order
    or whether the spindle was on.
  - jumps, samples further apart than one sample interval allows. The path is
    not interpolated across them, and there shouldn't be any.
  - for a power cut, where the machine really was when the power went (the last
    stepper sample) compared with the line the record names. On that line is
    exact, on a later line means the record lagged behind, and an earlier line
    would mean it ran ahead, which must never happen.
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
MAX_STEP = 1.5      # mm, more than the machine moves in one 10 ms sample (6000 mm/min is 1 mm)


def read_steps(path):
    """The stepper samples as rows of (simulated time in s, machine x, y, z in mm)."""
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        time, position = line.split(None, 1)
        x, y, z = (int(value) for value in position.split(","))
        rows.append((float(time), x / STEPS_PER_MM, y / STEPS_PER_MM, z / STEPS_PER_MM))
    return np.array(rows) if rows else np.zeros((0, 4))


def boots(run):
    """The step samples of each boot in a run."""
    samples = []
    number = 1
    while os.path.exists(os.path.join(run, f"boot{number}.steps.log")):
        samples.append(read_steps(os.path.join(run, f"boot{number}.steps.log")))
        number += 1
    return samples


def densify(points, step=0.02):
    """Fill in a polyline with a point every `step` mm, for nearest distance queries.

    Gaps longer than MAX_STEP are left alone, since filling them in would invent motion
    nobody sampled.
    """
    filled = [points[:1]]
    for start, end in zip(points[:-1], points[1:]):
        distance = float(np.linalg.norm(end - start))
        count = int(math.ceil(distance / step))
        if count > 1 and distance <= MAX_STEP:
            t = np.linspace(0, 1, count + 1)[1:, None]
            filled.append(start + (end - start) * t)
        else:
            filled.append(end[None, :])
    return np.vstack(filled)


def jumps(points):
    if len(points) < 2:
        return 0
    return int((np.linalg.norm(np.diff(points, axis=0), axis=1) > MAX_STEP).sum())


def in_stock(points):
    """Only the points below work Z 0, where the tool can cut. Rapids and retracts above it can't."""
    return points[points[:, 2] < G54[2] - 1e-6]


def program_segments(path):
    """For each program line that moves, the line number and its path in machine coordinates."""
    _, lines = load_program(path)
    position = None
    motion = "G0"
    segments = []
    for number, text in lines:
        words = WORD.findall(text)
        for letter, value in words:
            if letter == "G" and value.split(".")[0] in ("0", "1", "2", "3", "00", "01", "02", "03"):
                motion = "G" + str(int(float(value)))
        target = dict(zip("XYZ", position)) if position is not None else {}
        moved = False
        for letter, value in words:
            if letter in "XYZ":
                target[letter] = float(value)
                moved = True
        if not moved or "G53" in text:
            continue
        if position is None or len(target) < 3:
            position = [target.get(axis, 0.0) for axis in "XYZ"]
            continue
        end = np.array([target[axis] for axis in "XYZ"])
        start = np.array(position)
        if motion in ("G2", "G3"):
            offsets = dict(words)
            cx, cy = start[0] + float(offsets.get("I", 0)), start[1] + float(offsets.get("J", 0))
            start_angle = math.atan2(start[1] - cy, start[0] - cx)
            end_angle = math.atan2(end[1] - cy, end[0] - cx)
            if motion == "G3":
                while end_angle <= start_angle:
                    end_angle += 2 * math.pi
            else:
                while end_angle >= start_angle:
                    end_angle -= 2 * math.pi
            radius = math.hypot(start[0] - cx, start[1] - cy)
            count = max(8, int(abs(end_angle - start_angle) * radius / 0.05))
            t = np.linspace(0, 1, count + 1)
            angles = start_angle + (end_angle - start_angle) * t
            path = np.column_stack([cx + radius * np.cos(angles), cy + radius * np.sin(angles),
                                    start[2] + (end[2] - start[2]) * t])
        else:
            path = np.vstack([start, end])
        segments.append((number, path + G54))
        position = list(end)
    return segments


def distance_to_path(point, path):
    best = math.inf
    for start, end in zip(path[:-1], path[1:]):
        segment = end - start
        length_squared = float(segment @ segment)
        t = 0.0 if length_squared == 0 else max(0.0, min(1.0, float((point - start) @ segment) / length_squared))
        best = min(best, float(np.linalg.norm(start + t * segment - point)))
    return best


def line_at(point, segments, near_line, window=60):
    """Which program line the machine point is on, looking around the recorded line."""
    index = next(i for i, (number, _) in enumerate(segments) if number >= near_line)
    candidates = []
    for i in range(max(0, index - window), min(len(segments), index + window)):
        candidates.append((distance_to_path(point, segments[i][1]), segments[i][0]))
    return min(candidates)


def main():
    runs_dir, program, out_dir = sys.argv[1:4]
    reference_name, names = sys.argv[4], sys.argv[5:]
    os.makedirs(out_dir, exist_ok=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    reference = np.vstack(boots(os.path.join(runs_dir, reference_name)))
    reference_points = in_stock(densify(reference[:, 1:4]))
    segments = program_segments(program)
    report = {reference_name: {"jumps": jumps(reference[:, 1:4])}}

    for name in names:
        run = os.path.join(runs_dir, name)
        summary = json.load(open(os.path.join(run, "summary.json")))
        run_boots = boots(run)
        cut = in_stock(np.vstack([densify(samples[:, 1:4]) for samples in run_boots if len(samples)]))
        distances, _ = cKDTree(cut).query(reference_points)
        result = {
            "jumps": sum(jumps(samples[:, 1:4]) for samples in run_boots),
            "scenario": summary["scenario"], "via": summary["via"],
            "record": summary.get("record_before_resume"),
            "resumed_from": summary.get("resume_first_line", "")[:40],
            "resume": summary.get("resume"),
            "final_record": summary.get("record_after_resume"),
            "coverage_max_mm": round(float(distances.max()), 4),
            "coverage_p99_mm": round(float(np.percentile(distances, 99)), 4),
        }
        if summary["scenario"] == "power" and len(run_boots[0]):
            last = run_boots[0][-1]
            recorded = summary["record_before_resume"]["ln"]
            distance, line = line_at(last[1:4], segments, recorded)
            if line == recorded:
                verdict = "exact"
            else:
                verdict = "lagged" if line > recorded else "AHEAD"
            result["power_cut"] = {"last_position": [round(value, 3) for value in last[1:4]], "on_line": line,
                                   "distance_mm": round(distance, 4), "recorded_line": recorded, "verdict": verdict}
        report[name] = result

        # Machine Z over time, with the boots one after the other.
        fig, ax = plt.subplots(figsize=(9, 3.2))
        offset = 0.0
        for number, samples in enumerate(run_boots, 1):
            if len(samples):
                # There are no samples while the machine is idle, so break the line there
                # rather than draw motion that didn't happen.
                t, z = samples[:, 0] + offset, samples[:, 3].copy()
                gaps = np.where(np.diff(t) > 0.05)[0] + 1
                t, z = np.insert(t, gaps, np.nan), np.insert(z, gaps, np.nan)
                ax.plot(t, z, lw=0.8, label=f"boot {number} (gaps: idle, no motion)")
                offset += samples[-1, 0] + 2.0
        ax.axhline(-20.0, color="tab:red", lw=0.8, ls="--", label="retract height (machine Z -20)")
        ax.set_xlabel("simulated time, s (boots placed end to end)")
        ax.set_ylabel("machine Z, mm")
        record = result["record"]
        ax.set_title(f"{name}: {summary['scenario']} via {summary['via']}, record {record['state'] if record else '-'}"
                     f" / {record['why'] if record else '-'} at line {record['ln'] if record else '-'}")
        ax.legend(fontsize=7, loc="lower right")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"{name}-z.png"), dpi=120)
        plt.close(fig)

    with open(os.path.join(out_dir, "analysis.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
