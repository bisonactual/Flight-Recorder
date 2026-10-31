"""Test sender for the flight recorder: numbered streaming, job stamp, resume.

The smallest version of what FlexiSender has to do:
  1. number every line N<file line> (replacing any N-words in the file);
  2. stamp the job: $FLIGHT=<CRC-32 of the file, hex>[,<flags>];
  3. stream with character counting;
  4. resume: read $FLIGHT, check the job id, rebuild the modal state up to the
     recorded line, re-enter straight down onto its start point, and stream
     from that line on.

The resume refuses, before anything moves, what it can't rebuild: incremental
moves, inverse-time or per-revolution feed, cutter compensation, canned
cycles, probing, G10/G28/G30/G92, axes other than XYZ, and a spindle turning
with no speed set. One tool.
"""

import json
import re
import time
import zlib

WORD = re.compile(r"([A-Z])([-+]?[0-9]*\.?[0-9]+)")


def clean(raw):
    """Strip comments, N-words and spaces; upper case."""
    text = re.sub(r"\([^)]*\)", "", raw).split(";")[0].upper()
    text = re.sub(r"N[0-9]+", "", text.replace(" ", "").replace("\t", ""))
    return text.strip()


def load_program(path):
    data = open(path, "rb").read()
    job = zlib.crc32(data) & 0xFFFFFFFF
    lines = []
    for number, raw in enumerate(data.decode("ascii", "replace").splitlines(), 1):
        text = clean(raw)
        if text and text != "%":
            lines.append((number, text))
    return job, lines


# ── Streaming ────────────────────────────────────────────────────────────────

class Streamer:
    """Character-counting streamer (as ioSender/FlexiSender do)."""

    def __init__(self, link, rx_size=1000, log=None, status_interval=0.25):
        self.link = link
        self.rx_size = rx_size
        self.log = log
        self.status_interval = status_interval
        self.inflight = []
        self.errors = []
        self.alarms = []
        self.messages = []
        self.t0 = time.monotonic()
        self.next_status = 0.0

    def _note(self, kind, **fields):
        if self.log:
            fields.update(t=round(time.monotonic() - self.t0, 3), kind=kind)
            self.log.write(json.dumps(fields) + "\n")

    def _handle(self, line):
        if line == "ok" or line.startswith("error"):
            sent = self.inflight.pop(0) if self.inflight else None
            if line.startswith("error"):
                sent = sent.decode("ascii").strip() if sent else None
                self.errors.append((sent, line))
                self._note("error", line=sent, reply=line)
        elif line.startswith("<"):
            self._note("status", report=line)
        elif line.startswith("ALARM"):
            self.alarms.append(line)
            self._note("alarm", reply=line)
        else:
            self.messages.append(line)
            self._note("message", reply=line)

    def _pump(self, timeout):
        line = self.link.readline(timeout)
        while line is not None:
            self._handle(line)
            line = self.link.readline(0)
        now = time.monotonic() - self.t0
        if self.status_interval and now >= self.next_status:
            self.next_status = now + self.status_interval
            self.link.send(b"?")

    def send_lines(self, texts, stop=None):
        """Stream texts; stop(i, elapsed) may return 'hang' or 'close' to cut it."""
        for i, text in enumerate(texts):
            data = (text + "\n").encode("ascii")
            while sum(len(d) for d in self.inflight) + len(data) > self.rx_size:
                self._pump(0.01)
                if self.alarms:
                    return "alarm", i
            if stop:
                action = stop(i, time.monotonic() - self.t0)
                if action:
                    self._note("interrupt", action=action, index=i, sent=text)
                    return action, i
            self.link.send(data)
            self.inflight.append(data)
            self._pump(0)
        return "done", len(texts)

    def drain(self, timeout=120):
        """Wait for all replies and the machine to go idle."""
        deadline = time.monotonic() + timeout
        idle_seen = 0
        while time.monotonic() < deadline:
            self._pump(0.05)
            if self.alarms:
                return False
            if not self.inflight:
                self.link.send(b"?")
                line = self.link.readline(0.5)
                while line is not None and not line.startswith("<"):
                    self._handle(line)
                    line = self.link.readline(0.5)
                if line:
                    self._handle(line)
                    idle_seen = idle_seen + 1 if line.startswith("<Idle") else 0
                    if idle_seen >= 2:
                        return True
        return False


def numbered(lines):
    return [f"N{number}{text}" for number, text in lines]


# ── Flight record ────────────────────────────────────────────────────────────

def parse_flight(line):
    """[FLIGHT:Retracted|why=timeout|job=...|ln=123|...] -> dict"""
    body = line.strip()[len("[FLIGHT:"):-1]
    parts = body.split("|")
    record = {"state": parts[0]}
    for part in parts[1:]:
        key, _, value = part.partition("=")
        record[key] = value
    record["job"] = int(record["job"], 16)
    record["ln"] = int(record["ln"])
    record["mpos"] = [float(v) for v in record["mpos"].split(",")]
    return record


def query_flight(link):
    for line in link.command("$FLIGHT"):
        if line.startswith("[FLIGHT:"):
            return parse_flight(line)
    return None


# ── Resume ───────────────────────────────────────────────────────────────────

def gcodes(words):
    """G numbers of a line, normalised: G01 -> '1', G38.20 -> '38.2'."""
    return {(v.rstrip("0").rstrip(".") if "." in v else str(int(float(v)))) for letter, v in words if letter == "G"}


MOTION = {"0", "1", "2", "3"}
REFUSED = {"93": "inverse-time feed (G93)", "95": "feed per revolution (G95)", "41": "cutter compensation",
           "42": "cutter compensation", "10": "offsets set by the program (G10)", "92": "G92 offsets"}
CYCLES = {"73", "76", "81", "82", "83", "84", "85", "86", "87", "88", "89", "38.2", "38.3", "38.4", "38.5"}


def modal_scan(lines, upto):
    """Modal state in force at the start of line `upto`, and where each axis is then:
    (mm, frame), the frame being the work offset it was programmed in, or G53."""
    st = {"units": "G21", "plane": "G17", "wcs": "G54", "distance": "G90", "motion": "G0",
          "F": None, "S": None, "spindle": "M5", "coolant": "M9",
          "pos": {"X": None, "Y": None, "Z": None}}
    for number, text in lines:
        if number > upto:
            break
        words = WORD.findall(text)
        g = gcodes(words)
        if g & CYCLES:                                  # also on the resume line itself
            raise ValueError(f"line {number}: canned cycles and probing are not supported by this resume")
        if number == upto:
            break
        if g & REFUSED.keys():
            raise ValueError(f"line {number}: {REFUSED[min(g & REFUSED.keys())]} is not supported by this resume")
        if any(letter in "ABCUVW" for letter, _ in words):
            raise ValueError(f"line {number}: axes other than XYZ are not supported by this resume")
        for c in g:
            if c in MOTION:
                st["motion"] = "G" + c
            elif c in ("20", "21"):
                st["units"] = "G" + c
            elif c in ("17", "18", "19"):
                st["plane"] = "G" + c
            elif c in ("90", "91"):
                st["distance"] = "G" + c
            elif c in ("54", "55", "56", "57", "58", "59"):
                st["wcs"] = "G" + c
        for letter, value in words:
            if letter == "M":
                m = str(int(float(value)))
                if m in ("3", "4", "5"):
                    st["spindle"] = "M" + m
                elif m in ("7", "8", "9"):
                    st["coolant"] = "M" + m
            elif letter in ("F", "S"):
                st[letter] = float(value)
        axes = [(letter, float(value)) for letter, value in words if letter in "XYZ"]
        if not axes:
            continue
        if g & {"28", "30"}:                            # to a stored position: unknown here
            for letter, _ in axes:
                st["pos"][letter] = None
            continue
        if st["distance"] == "G91" and "53" not in g:
            raise ValueError(f"line {number}: incremental moves are not supported by this resume")
        unit = 25.4 if st["units"] == "G20" else 1.0      # this line's own G20/G21 applies to it
        frame = "G53" if "53" in g else st["wcs"]
        for letter, value in axes:
            st["pos"][letter] = (value * unit, frame)
    return st


def resume_preamble(job, st, safe_machine_z=-20.0, spinup=2.0):
    """Lines that put the machine back onto the start of the resume line. Re-entry in mm,
    absolute, feed per minute, each axis in the frame it was programmed in; then the
    program's own units, offset and feed back."""
    pos = st["pos"]
    if None in pos.values():
        raise ValueError("no position before the resume line: restart the job instead")
    if st["spindle"] in ("M3", "M4") and not st["S"]:
        raise ValueError("the spindle is on with no speed set before the resume line")
    feed = st["F"] * (25.4 if st["units"] == "G20" else 1.0) if st["F"] else 100.0   # mm/min

    def move(axes, motion, f=""):
        out = []
        for frame in sorted({pos[a][1] for a in axes}):
            words = "".join(f"{a}{pos[a][0]:.4f}" for a in axes if pos[a][1] == frame)
            out.append(f"{frame}{motion}{words}{f}")
        return out

    pre = [f"$FLIGHT={job:08X}",
           f"G21G90{st['plane']}G94",
           f"G53G0Z{safe_machine_z:.4f}"]               # up at the retract height, machine coordinates
    pre += move("XY", "G0")                             # over the start of the resume line
    if st["spindle"] in ("M3", "M4"):
        pre += [f"S{st['S']:.0f}{st['spindle']}", f"G4P{spinup:.1f}"]
    if st["coolant"] in ("M7", "M8"):
        pre.append(st["coolant"])
    pre += move("Z", "G1", f"F{feed:.1f}")              # straight down: the tool has been there
    pre.append(f"{st['units']}{st['wcs']}" + (f"F{st['F']:.1f}" if st["F"] else ""))
    return pre


def resume_lines(lines, record, st):
    """The program from the recorded line on, with the motion mode restored."""
    start = next(i for i, (number, _) in enumerate(lines) if number >= record["ln"])
    rest = numbered(lines[start:])
    if not gcodes(WORD.findall(lines[start][1])) & MOTION:
        rest[0] = f"N{lines[start][0]}{st['motion']}{lines[start][1]}"
    return rest
