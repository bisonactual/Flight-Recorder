"""Test sender for the flight recorder: numbered streaming, job stamp, resume.

The smallest version of what FlexiSender has to do:
  1. number every line N<file line> (replacing any N-words in the file);
  2. stamp the job: $FRJ=<CRC-32 of the file, hex>[,<flags>];
  3. stream with character counting;
  4. resume: read $FR, check the job id, rebuild the modal state up to the
     recorded line, re-enter straight down onto its start point, and stream
     from that line on.

Limits of this test version: absolute distance mode only, one tool, no
canned cycles or cutter compensation in the resume scan.
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

def parse_fr(line):
    """[FR:Retracted|why=timeout|job=...|ln=123|...] -> dict"""
    body = line.strip()[len("[FR:"):-1]
    parts = body.split("|")
    record = {"state": parts[0]}
    for part in parts[1:]:
        key, _, value = part.partition("=")
        record[key] = value
    record["job"] = int(record["job"], 16)
    record["ln"] = int(record["ln"])
    record["mpos"] = [float(v) for v in record["mpos"].split(",")]
    return record


def query_fr(link):
    for line in link.command("$FR"):
        if line.startswith("[FR:"):
            return parse_fr(line)
    return None


# ── Resume ───────────────────────────────────────────────────────────────────

def modal_scan(lines, upto):
    """Modal state and work position in force at the start of line `upto`."""
    st = {"units": "G21", "plane": "G17", "wcs": "G54", "feed_mode": "G94",
          "distance": "G90", "motion": "G0", "F": None, "S": None,
          "spindle": "M5", "coolant": "M9", "pos": {"X": None, "Y": None, "Z": None}}
    for number, text in lines:
        if number >= upto:
            break
        words = WORD.findall(text)
        for letter, value in words:
            if letter == "G":
                g = value.rstrip("0").rstrip(".") if "." in value else str(int(float(value)))
                if g in ("0", "1", "2", "3"):
                    st["motion"] = "G" + g
                elif g in ("20", "21"):
                    st["units"] = "G" + g
                elif g in ("17", "18", "19"):
                    st["plane"] = "G" + g
                elif g in ("90", "91"):
                    st["distance"] = "G" + g
                elif g in ("93", "94"):
                    st["feed_mode"] = "G" + g
                elif g in ("54", "55", "56", "57", "58", "59"):
                    st["wcs"] = "G" + g
            elif letter == "M":
                m = str(int(float(value)))
                if m in ("3", "4", "5"):
                    st["spindle"] = "M" + m
                elif m in ("7", "8", "9"):
                    st["coolant"] = "M" + m
            elif letter in ("F", "S"):
                st[letter] = float(value)
        letters = {letter for letter, _ in words}
        moved = any(a in letters for a in "XYZ")
        if moved and "G53" not in text and not re.search(r"G28|G30|G10|G92", text):
            if st["distance"] != "G90":
                raise ValueError(f"line {number}: incremental moves are not supported by this resume")
            for letter, value in words:
                if letter in "XYZ":
                    st["pos"][letter] = float(value)
    return st


def resume_preamble(job, st, safe_machine_z=-20.0, spinup=2.0, plunge_feed=None):
    """Lines that put the machine back onto the start of the resume line."""
    x, y, z = (st["pos"][a] for a in "XYZ")
    if None in (x, y, z):
        raise ValueError("no position before the resume line: restart the job instead")
    unit = 25.4 if st["units"] == "G20" else 1.0
    feed = plunge_feed or st["F"] or 100.0
    pre = [
        f"$FRJ={job:08X}",
        f"{st['units']}G90{st['plane']}{st['feed_mode']}{st['wcs']}",
        f"G53G0Z{safe_machine_z / unit:.4f}",   # up at the retract height, machine coordinates
        f"G0X{x:.4f}Y{y:.4f}",                   # over the start of the resume line
    ]
    if st["spindle"] in ("M3", "M4"):
        pre += [f"S{st['S'] or 0:.0f}{st['spindle']}", f"G4P{spinup:.1f}"]
    if st["coolant"] in ("M7", "M8"):
        pre.append(st["coolant"])
    pre += [f"G1Z{z:.4f}F{feed:.1f}"]            # straight down: the tool has been there
    if st["F"]:
        pre.append(f"F{st['F']:.1f}")
    return pre


def resume_lines(lines, record, st):
    """The program from the recorded line on, with the motion mode restored."""
    start = next(i for i, (number, _) in enumerate(lines) if number >= record["ln"])
    rest = numbered(lines[start:])
    first_text = lines[start][1]
    if not re.match(r"^(G0?[0-3])(?![0-9.])", first_text) and not re.search(r"G0?[0-3](?![0-9.])", first_text):
        rest[0] = f"N{lines[start][0]}{st['motion']}{first_text}"
    return rest
