"""Focused flight recorder scenarios in the simulator, each with its own check.

Technical review A's falsifiers (31 Oct 2026), kept as evidence. Every
scenario stamps a job, sets up a short numbered start (N1-N3: G54, a move to
X10 Y10 Z-1, spindle on) over WebSocket, then:

  dwell-descend   N4 G4P6 G1Z0 (dwell, then a plunge); the client closes during
                  the dwell -> the plunge runs, then the retract: Z ends at -20
  numbered-jog    a jog with N900 -> the record keeps line 2
  partial-line    half a line sent, then the timeout -> the first query after
                  the retract is answered (the half line was dropped)
  resume-power    retract, then a resume whose re-entry is cut by a power cut
                  -> the record keeps the line from before the resume
  reconnect       client closes and a new one connects 50 ms later, sending
                  N5 -> N5 never runs (X stays at N4's target)
  reset-control   reset during a numbered move -> why=reset
  estop-retract   e-stop during the retract -> Stopped, not Retracted
  reset-retract   reset during the retract -> Stopped, spindle off
  g20, g91        the job in inches / incremental -> retract to machine Z -20
  check           check mode -> no retract
  m0              program pause past the timeout -> no retract; cycle start
                  carries the job on (N5 recorded)
  hold-retract    feed hold during the retract until it gives up -> alarm 81,
                  Stopped. Needs a simulator built with a short limit
                  (-DFR_RETRACT_LIMIT=15000u)

Usage: focused.py <scenario> --sim <binary> --run <dir>. Prints the result
and its checks as JSON; exits 1 if a check fails.
"""

import argparse
import json
import re
import sys
import time

from evidence import setup_machine, wait_for
from link import open_link
from simrun import Sim
from stream import parse_flight

SCENARIOS = ["dwell-descend", "numbered-jog", "partial-line", "resume-power", "reconnect", "reset-control",
             "estop-retract", "reset-retract", "g20", "g91", "check", "m0", "hold-retract"]


def status_of(report):
    """'<Alarm|MPos:x,y,z,...|...>' -> (state, [x, y, z])."""
    m = re.match(r"<(\w+)[^|]*\|MPos:([-\d.]+),([-\d.]+),([-\d.]+)", report or "")
    return (m.group(1), [float(v) for v in m.group(2, 3, 4)]) if m else (None, None)


def wait_status(link, ok, timeout=40):
    """Poll the status report until ok(state, pos) or the timeout: the simulator's
    clock runs slower than real time on a loaded host, so no fixed sleeps."""
    deadline = time.monotonic() + timeout
    while True:
        state, pos = status_of(wait_for(link, lambda l: l.startswith("<"), 2))
        if (state and ok(state, pos)) or time.monotonic() > deadline:
            return state, pos
        time.sleep(0.2)


def alarm(state, pos):
    return state == "Alarm"


def rising(state, pos):
    return pos[2] > -60.9       # the retract under way (cut depth is machine Z -61)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=SCENARIOS)
    ap.add_argument("--sim", required=True)
    ap.add_argument("--run", required=True)
    args = ap.parse_args()

    sc = args.scenario
    out = {"scenario": sc, "commands": []}
    checks = {}
    sim = Sim(args.sim, args.run, speed=4).start()
    link = None

    def cmd(text, timeout=10):
        reply = link.command(text, timeout=timeout)
        out["commands"].append([text, reply])
        return reply

    def look(label, wait=0.0):
        if wait:
            time.sleep(wait)
        replies = link.command("$FLIGHT")
        record = next((parse_flight(l) for l in replies if l.startswith("[FLIGHT:")), None)
        status = wait_for(link, lambda l: l.startswith("<"), 2)
        out[label] = {"record": record, "replies": replies, "status": status}
        return record, status_of(status)

    try:
        link = open_link(sim.ws)
        link.lines_until(lambda l: False, 0.4)
        out["setup"] = setup_machine(link)
        cmd(f"$FLIGHT={0xAA00 + SCENARIOS.index(sc):08X}")
        if sc == "check":
            cmd("$C")
        if sc in ("g20", "g91", "check", "m0", "partial-line", "resume-power"):
            cmd("$791=0.5")
        for line in ("N1G21G90G54", "N2G0X10Y10Z-1", "N3S12000M3"):
            cmd(line)

        if sc == "dwell-descend":
            cmd("G53G0Z-10")
            link.send(b"N4G4P6G1Z0F3000\n")
            time.sleep(1)
            link.close()
            link = open_link(sim.usb)
            link.lines_until(lambda l: False, 0.3)
            wait_status(link, alarm, 60)
            rec, (state, pos) = look("after")
            checks["retracted"] = bool(rec) and rec["state"] == "Retracted"
            checks["at machine Z -20"] = bool(pos) and abs(pos[2] + 20.0) < 0.01

        elif sc == "numbered-jog":
            cmd("M5")
            cmd("$J=G91G21X1F600N900")
            rec, _ = look("after", 0.5)
            checks["line still 2"] = bool(rec) and rec["ln"] == 2

        elif sc == "partial-line":
            link.send(b"N50G1X")
            wait_status(link, alarm)
            rec, _ = look("after")
            checks["first query answered"] = rec is not None
            checks["retracted"] = bool(rec) and rec["state"] == "Retracted"

        elif sc == "resume-power":
            wait_status(link, alarm)
            first, _ = look("before")
            cmd("$X")
            cmd("$H")
            cmd(f"$FLIGHT={0xAA00 + SCENARIOS.index(sc):08X}")
            for line in ("G21G90G54", "G53G0Z-20", "G0X10Y10", "S12000M3"):
                cmd(line)
            link.send(b"G1Z-1F60\n")
            time.sleep(0.4)
            sim.kill()
            link.close()
            sim.start()
            link = open_link(sim.usb)
            link.lines_until(lambda l: False, 0.3)
            rec, _ = look("after")
            checks["line kept"] = bool(first and rec) and rec["ln"] == first["ln"] != 0
            checks["stopped, reboot"] = bool(rec) and (rec["state"], rec["why"]) == ("Stopped", "reboot")

        elif sc == "reconnect":
            cmd("N4G1X50F400")
            link.close()
            time.sleep(0.05)
            link = open_link(sim.ws)
            link.lines_until(lambda l: False, 0.3)
            cmd("N5G1X60F400")
            wait_status(link, alarm)
            rec, (state, pos) = look("after")
            checks["N5 never ran"] = bool(pos) and abs(pos[0] - (-250.0)) < 0.01
            checks["retracted, network"] = bool(rec) and (rec["state"], rec["why"]) == ("Retracted", "network")

        elif sc == "reset-control":
            cmd("$790=0")
            cmd("$791=60")
            cmd("N4G1X50F60")
            time.sleep(0.4)
            link.send(b"\x18")
            time.sleep(0.4)
            rec, _ = look("after", 1)
            checks["why=reset"] = bool(rec) and (rec["state"], rec["why"]) == ("Stopped", "reset")

        elif sc in ("estop-retract", "reset-retract"):
            cmd("$112=60")
            cmd("$791=0.5")
            wait_status(link, rising)
            if sc == "estop-retract":
                sim.key("e")
            else:
                link.send(b"\x18")
            time.sleep(0.4)
            rec, _ = look("after", 1)
            checks["stopped, not retracted"] = bool(rec) and rec["state"] == "Stopped"
            checks["spindle off"] = "A:S" not in (out["after"]["status"] or "")

        elif sc in ("g20", "g91"):
            cmd("N4" + sc.upper())
            wait_status(link, alarm)
            rec, (state, pos) = look("after")
            checks["retracted"] = bool(rec) and rec["state"] == "Retracted"
            checks["at machine Z -20"] = bool(pos) and abs(pos[2] + 20.0) < 0.01

        elif sc == "check":
            cmd("N4M30")
            rec, (state, _) = look("after", 4)
            checks["no retract"] = state == "Check" and bool(rec) and rec["state"] == "Running"

        elif sc == "m0":
            cmd("N4M0")
            time.sleep(4)
            state, _ = status_of(wait_for(link, lambda l: l.startswith("<"), 2))
            checks["held, no retract"] = state == "Hold"
            link.send(b"~")
            link.lines_until(lambda l: l == "ok", 3)     # N4's, held back by the pause
            cmd("N5G1X20F500")
            rec, _ = look("after", 1)
            checks["went on to N5"] = bool(rec) and rec["ln"] == 5

        elif sc == "hold-retract":
            cmd("$112=60")
            cmd("$791=0.5")
            wait_status(link, rising)
            link.send(b"!")
            seen = []
            t0 = time.monotonic()
            while time.monotonic() - t0 < 60:
                s = wait_for(link, lambda l: l.startswith("<") or seen.append(l), 2)  # keeps the rest
                if s and s.startswith("<Alarm"):
                    break
                time.sleep(1)
            out["messages"] = seen
            rec, _ = look("after")
            checks["alarm 81"] = "ALARM:81" in seen
            checks["stopped"] = bool(rec) and rec["state"] == "Stopped"
    finally:
        if link:
            try:
                link.close()
            except Exception:
                pass
        sim.stop()
        out["checks"] = checks
        out["passed"] = bool(checks) and all(checks.values())
        with open(f"{args.run}/result.json", "w") as f:
            json.dump(out, f, indent=1)
        print(json.dumps({"scenario": sc, "checks": checks, "passed": out["passed"]}))

    sys.exit(0 if out["passed"] else 1)


if __name__ == "__main__":
    main()
