"""Test sender for the flight recorder: numbered streaming, job stamp and resume.

This is the smallest version of what FlexiSender has to do:
  1. number every line N<line in the file>, replacing any N-words the file has
  2. stamp the job with $FLIGHT=<CRC-32 of the file, in hex>[,<flags>]
  3. stream with character counting
  4. to resume, read $FLIGHT, check the job id, work out the modal state up to
     the recorded line, go back in straight down onto its start point and
     stream on from that line

The resume refuses anything it can't rebuild before the machine moves:
incremental moves, inverse time or per revolution feed, cutter compensation,
canned cycles, probing, G10/G28/G30/G92, axes other than XYZ and a spindle
running with no speed set. It assumes one tool.
"""

import json
import re
import time
import zlib

WORD = re.compile(r"([A-Z])([-+]?[0-9]*\.?[0-9]+)")


def clean(raw):
    """Upper case, with comments, N-words and spaces taken out."""
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


class Streamer:
    """Character counting streamer, the way ioSender and FlexiSender do it."""

    def __init__(self, link, rx_size=1000, log=None, status_interval=0.25):
        self.link = link
        self.rx_size = rx_size
        self.log = log
        self.status_interval = status_interval
        self.in_flight = []
        self.errors = []
        self.alarms = []
        self.messages = []
        self.started = time.monotonic()
        self.next_status = 0.0

    def _log(self, kind, **fields):
        if self.log:
            fields.update(t=round(time.monotonic() - self.started, 3), kind=kind)
            self.log.write(json.dumps(fields) + "\n")

    def _handle(self, line):
        if line == "ok" or line.startswith("error"):
            sent = self.in_flight.pop(0) if self.in_flight else None
            if line.startswith("error"):
                sent = sent.decode("ascii").strip() if sent else None
                self.errors.append((sent, line))
                self._log("error", line=sent, reply=line)
        elif line.startswith("<"):
            self._log("status", report=line)
        elif line.startswith("ALARM"):
            self.alarms.append(line)
            self._log("alarm", reply=line)
        else:
            self.messages.append(line)
            self._log("message", reply=line)

    def _read_replies(self, timeout):
        line = self.link.readline(timeout)
        while line is not None:
            self._handle(line)
            line = self.link.readline(0)
        now = time.monotonic() - self.started
        if self.status_interval and now >= self.next_status:
            self.next_status = now + self.status_interval
            self.link.send(b"?")

    def send_lines(self, texts, stop=None):
        """Stream the lines. stop(index, elapsed) can return 'hang' or 'close' to cut it short."""
        for index, text in enumerate(texts):
            data = (text + "\n").encode("ascii")
            while sum(len(sent) for sent in self.in_flight) + len(data) > self.rx_size:
                self._read_replies(0.01)
                if self.alarms:
                    return "alarm", index
            if stop:
                action = stop(index, time.monotonic() - self.started)
                if action:
                    self._log("interrupt", action=action, index=index, sent=text)
                    return action, index
            self.link.send(data)
            self.in_flight.append(data)
            self._read_replies(0)
        return "done", len(texts)

    def drain(self, timeout=120):
        """Wait for all the replies and for the machine to go idle."""
        deadline = time.monotonic() + timeout
        idle_count = 0
        while time.monotonic() < deadline:
            self._read_replies(0.05)
            if self.alarms:
                return False
            if not self.in_flight:
                self.link.send(b"?")
                line = self.link.readline(0.5)
                while line is not None and not line.startswith("<"):
                    self._handle(line)
                    line = self.link.readline(0.5)
                if line:
                    self._handle(line)
                    idle_count = idle_count + 1 if line.startswith("<Idle") else 0
                    if idle_count >= 2:
                        return True
        return False


def numbered(lines):
    return [f"N{number}{text}" for number, text in lines]


# The flight record

def parse_flight(line):
    """Turn '[FLIGHT:Retracted|why=timeout|job=...|ln=123|...]' into a dict."""
    fields = line.strip()[len("[FLIGHT:"):-1].split("|")
    record = {"state": fields[0]}
    for field in fields[1:]:
        key, _, value = field.partition("=")
        record[key] = value
    record["job"] = int(record["job"], 16)
    record["ln"] = int(record["ln"])
    record["mpos"] = [float(value) for value in record["mpos"].split(",")]
    return record


def query_flight(link):
    for line in link.command("$FLIGHT"):
        if line.startswith("[FLIGHT:"):
            return parse_flight(line)
    return None


# Resuming

def gcodes(words):
    """The G numbers on a line, tidied up so that G01 is '1' and G38.20 is '38.2'."""
    return {(value.rstrip("0").rstrip(".") if "." in value else str(int(float(value))))
            for letter, value in words if letter == "G"}


MOTION = {"0", "1", "2", "3"}
REFUSED = {"93": "inverse time feed (G93)", "95": "feed per revolution (G95)", "41": "cutter compensation",
           "42": "cutter compensation", "10": "offsets set by the program (G10)", "92": "G92 offsets"}
CYCLES = {"73", "76", "81", "82", "83", "84", "85", "86", "87", "88", "89", "38.2", "38.3", "38.4", "38.5"}


def modal_scan(lines, upto):
    """The modal state in force at the start of line `upto`, and where each axis is then.

    Each axis position is (mm, frame), where the frame is the work offset the move was
    programmed in, or G53.
    """
    modes = {"units": "G21", "plane": "G17", "wcs": "G54", "distance": "G90", "motion": "G0",
             "F": None, "S": None, "spindle": "M5", "coolant": "M9",
             "pos": {"X": None, "Y": None, "Z": None}}
    for number, text in lines:
        if number > upto:
            break
        words = WORD.findall(text)
        codes = gcodes(words)
        # Checked on the resume line itself as well.
        if codes & CYCLES:
            raise ValueError(f"line {number}: canned cycles and probing are not supported by this resume")
        if number == upto:
            break
        if codes & REFUSED.keys():
            raise ValueError(f"line {number}: {REFUSED[min(codes & REFUSED.keys())]} is not supported by this resume")
        if any(letter in "ABCUVW" for letter, _ in words):
            raise ValueError(f"line {number}: axes other than XYZ are not supported by this resume")
        for code in codes:
            if code in MOTION:
                modes["motion"] = "G" + code
            elif code in ("20", "21"):
                modes["units"] = "G" + code
            elif code in ("17", "18", "19"):
                modes["plane"] = "G" + code
            elif code in ("90", "91"):
                modes["distance"] = "G" + code
            elif code in ("54", "55", "56", "57", "58", "59"):
                modes["wcs"] = "G" + code
        for letter, value in words:
            if letter == "M":
                mcode = str(int(float(value)))
                if mcode in ("3", "4", "5"):
                    modes["spindle"] = "M" + mcode
                elif mcode in ("7", "8", "9"):
                    modes["coolant"] = "M" + mcode
            elif letter in ("F", "S"):
                modes[letter] = float(value)
        axes = [(letter, float(value)) for letter, value in words if letter in "XYZ"]
        if not axes:
            continue
        if codes & {"28", "30"}:
            # A move to a stored position, which we don't know here.
            for letter, _ in axes:
                modes["pos"][letter] = None
            continue
        if modes["distance"] == "G91" and "53" not in codes:
            raise ValueError(f"line {number}: incremental moves are not supported by this resume")
        # A G20 or G21 on this line already applies to its own coordinates.
        unit = 25.4 if modes["units"] == "G20" else 1.0
        frame = "G53" if "53" in codes else modes["wcs"]
        for letter, value in axes:
            modes["pos"][letter] = (value * unit, frame)
    return modes


def resume_preamble(job, modes, safe_machine_z=-20.0, spinup=2.0):
    """The lines that take the machine back onto the start of the resume line.

    The moves back in are in mm, absolute and feed per minute, each axis in the frame
    it was programmed in. Then the program's own units, work offset and feed are put back.
    """
    pos = modes["pos"]
    if None in pos.values():
        raise ValueError("no position before the resume line, restart the job instead")
    if modes["spindle"] in ("M3", "M4") and not modes["S"]:
        raise ValueError("the spindle is on with no speed set before the resume line")
    plunge_feed = modes["F"] * (25.4 if modes["units"] == "G20" else 1.0) if modes["F"] else 100.0   # mm/min

    def move(axes, motion, feed_word=""):
        moves = []
        for frame in sorted({pos[axis][1] for axis in axes}):
            words = "".join(f"{axis}{pos[axis][0]:.4f}" for axis in axes if pos[axis][1] == frame)
            moves.append(f"{frame}{motion}{words}{feed_word}")
        return moves

    preamble = [f"$FLIGHT={job:08X}",
                f"G21G90{modes['plane']}G94",
                f"G53G0Z{safe_machine_z:.4f}"]            # up to the retract height, in machine coordinates
    preamble += move("XY", "G0")                         # over the start of the resume line
    if modes["spindle"] in ("M3", "M4"):
        preamble += [f"S{modes['S']:.0f}{modes['spindle']}", f"G4P{spinup:.1f}"]
    if modes["coolant"] in ("M7", "M8"):
        preamble.append(modes["coolant"])
    preamble += move("Z", "G1", f"F{plunge_feed:.1f}")   # straight down, the tool has been there before
    preamble.append(f"{modes['units']}{modes['wcs']}" + (f"F{modes['F']:.1f}" if modes["F"] else ""))
    return preamble


def resume_lines(lines, record, modes):
    """The program from the recorded line on, with the motion mode put back on the first line."""
    start = next(index for index, (number, _) in enumerate(lines) if number >= record["ln"])
    remaining = numbered(lines[start:])
    if not gcodes(WORD.findall(lines[start][1])) & MOTION:
        remaining[0] = f"N{lines[start][0]}{modes['motion']}{lines[start][1]}"
    return remaining
