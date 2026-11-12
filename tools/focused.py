"""Short flight recorder scenarios in the simulator, each with its own checks.

These are the falsifiers from technical review A (31 Oct 2026), kept as
evidence. Each scenario stamps a job and starts it over WebSocket with three
numbered lines (G54, a move to X10 Y10 Z-1, spindle on), then:

  dwell-descend   N4 G4P6 G1Z0 (a dwell, then a plunge), and the client closes
                  during the dwell. The plunge runs, then the retract, and Z
                  ends up at -20.
  numbered-jog    a jog with N900. The record stays on line 2.
  partial-line    half a line, then the timeout. The first query after the
                  retract gets an answer because the half line was dropped.
  resume-power    a retract, then a resume that loses power while it moves back
                  in. The record keeps the line from before the resume.
  reconnect       the client closes and a new one connects 50 ms later and
                  sends N5. N5 never runs, X stays at N4's target.
  reset-control   a reset during a numbered move. Recorded as why=reset.
  estop-retract   an e-stop during the retract. Stopped, not Retracted.
  reset-retract   a reset during the retract. Stopped, spindle off.
  g20, g91        the job in inches, or incremental. Retracts to machine Z -20.
  check           check mode. No retract.
  m0              a program pause longer than the timeout. No retract, and
                  cycle start carries the job on (N5 gets recorded).
  hold-retract    a feed hold during the retract until it gives up. Alarm 81,
                  Stopped. Needs a simulator built with a short limit
                  (-DFR_RETRACT_LIMIT=15000u).

Usage: focused.py <scenario> --sim <binary> --run <dir> [--sd]. Prints the checks
as JSON and exits with 1 if any of them failed. --sd runs it on the simulator's SD
card instead of its FRAM stand-in.
"""

import argparse
import json
import re
import sys
import time

from evidence import setup_machine, wait_for, skip_greeting
from link import open_link
from simrun import Sim
from stream import parse_flight

SCENARIOS = ["dwell-descend", "numbered-jog", "partial-line", "resume-power", "reconnect", "reset-control",
             "estop-retract", "reset-retract", "g20", "g91", "check", "m0", "hold-retract"]

WORK_X50 = -250.0       # machine X of work X50, where N4 goes in the reconnect scenario


def is_status(line):
    return line.startswith("<")


def parse_status(report):
    """Turn '<Alarm|MPos:x,y,z,...|...>' into (state, [x, y, z])."""
    match = re.match(r"<(\w+)[^|]*\|MPos:([-\d.]+),([-\d.]+),([-\d.]+)", report or "")
    if not match:
        return None, None
    return match.group(1), [float(value) for value in match.group(2, 3, 4)]


def wait_until(link, condition, timeout=40):
    """Poll the status until condition(state, position) holds or the timeout runs out.

    The simulator's clock runs slower than real time on a busy host, so fixed sleeps
    aren't reliable here.
    """
    deadline = time.monotonic() + timeout
    while True:
        state, position = parse_status(wait_for(link, is_status, 2))
        if (state and condition(state, position)) or time.monotonic() > deadline:
            return state, position
        time.sleep(0.2)


def in_alarm(state, position):
    return state == "Alarm"


def retract_started(state, position):
    return position[2] > -60.9      # the cut depth is machine Z -61


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("--sim", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--sd", action="store_true", help="record on the SD card (no FRAM)")
    parser.add_argument("--speed", type=float, default=4.0, help="simulator speed; m0 needs 1 on a busy host")
    args = parser.parse_args()

    scenario = args.scenario
    job = f"{0xAA00 + SCENARIOS.index(scenario):08X}"
    result = {"scenario": scenario, "commands": []}
    checks = {}
    sim = Sim(args.sim, args.run, speed=args.speed, sd=args.sd).start()
    link = None

    def send(text, timeout=10):
        reply = link.command(text, timeout=timeout)
        result["commands"].append([text, reply])
        return reply

    def read_record(label, wait=0.0):
        if wait:
            time.sleep(wait)
        replies = link.command("$FLIGHT")
        record = next((parse_flight(line) for line in replies if line.startswith("[FLIGHT:")), None)
        status = wait_for(link, is_status, 2)
        result[label] = {"record": record, "replies": replies, "status": status}
        return record, parse_status(status)

    try:
        link = open_link(sim.ws)
        skip_greeting(link, 0.4)
        result["setup"] = setup_machine(link)
        send(f"$FLIGHT={job}")
        if scenario == "check":
            send("$C")
        if scenario in ("g20", "g91", "check", "m0", "partial-line", "resume-power"):
            send("$791=0.5")
        for line in ("N1G21G90G54", "N2G0X10Y10Z-1", "N3S12000M3"):
            send(line)

        if scenario == "dwell-descend":
            send("G53G0Z-10")
            link.send(b"N4G4P6G1Z0F3000\n")
            time.sleep(1)
            link.close()
            link = open_link(sim.usb)
            skip_greeting(link, 0.3)
            wait_until(link, in_alarm, 60)
            record, (state, position) = read_record("after")
            checks["retracted"] = bool(record) and record["state"] == "Retracted"
            checks["at machine Z -20"] = bool(position) and abs(position[2] + 20.0) < 0.01

        elif scenario == "numbered-jog":
            send("M5")
            send("$J=G91G21X1F600N900")
            record, _ = read_record("after", 0.5)
            checks["line still 2"] = bool(record) and record["ln"] == 2

        elif scenario == "partial-line":
            link.send(b"N50G1X")
            wait_until(link, in_alarm)
            record, _ = read_record("after")
            checks["first query answered"] = record is not None
            checks["retracted"] = bool(record) and record["state"] == "Retracted"

        elif scenario == "resume-power":
            wait_until(link, in_alarm)
            before, _ = read_record("before")
            send("$X")
            send("$H")
            send(f"$FLIGHT={job}")
            for line in ("G21G90G54", "G53G0Z-20", "G0X10Y10", "S12000M3"):
                send(line)
            link.send(b"G1Z-1F60\n")
            time.sleep(0.4)
            sim.kill()
            link.close()
            sim.start()
            link = open_link(sim.usb)
            skip_greeting(link, 0.3)
            record, _ = read_record("after")
            checks["line kept"] = bool(before and record) and record["ln"] == before["ln"] != 0
            checks["stopped, reboot"] = bool(record) and (record["state"], record["why"]) == ("Stopped", "reboot")

        elif scenario == "reconnect":
            send("N4G1X50F400")
            link.close()
            time.sleep(0.05)
            link = open_link(sim.ws)
            skip_greeting(link, 0.3)
            send("N5G1X60F400")
            wait_until(link, in_alarm)
            record, (state, position) = read_record("after")
            checks["N5 never ran"] = bool(position) and abs(position[0] - WORK_X50) < 0.01
            checks["retracted, network"] = bool(record) and (record["state"], record["why"]) == ("Retracted", "network")

        elif scenario == "reset-control":
            send("$790=0")
            send("$791=60")
            send("N4G1X50F60")
            time.sleep(0.4)
            link.send(b"\x18")
            time.sleep(0.4)
            record, _ = read_record("after", 1)
            checks["why=reset"] = bool(record) and (record["state"], record["why"]) == ("Stopped", "reset")

        elif scenario in ("estop-retract", "reset-retract"):
            send("$112=60")
            send("$791=0.5")
            wait_until(link, retract_started)
            if scenario == "estop-retract":
                sim.key("e")
            else:
                link.send(b"\x18")
            time.sleep(0.4)
            record, _ = read_record("after", 1)
            checks["stopped, not retracted"] = bool(record) and record["state"] == "Stopped"
            checks["spindle off"] = "A:S" not in (result["after"]["status"] or "")

        elif scenario in ("g20", "g91"):
            send("N4" + scenario.upper())
            wait_until(link, in_alarm)
            record, (state, position) = read_record("after")
            checks["retracted"] = bool(record) and record["state"] == "Retracted"
            checks["at machine Z -20"] = bool(position) and abs(position[2] + 20.0) < 0.01

        elif scenario == "check":
            send("N4M30")
            record, (state, _) = read_record("after", 4)
            checks["no retract"] = state == "Check" and bool(record) and record["state"] == "Running"

        elif scenario == "m0":
            send("N4M0")
            time.sleep(4)
            state, _ = parse_status(wait_for(link, is_status, 2))
            checks["held, no retract"] = state == "Hold"
            link.send(b"~")
            link.lines_until(lambda line: line == "ok", 3)     # the ok for N4, held back by the pause
            send("N5G1X20F500")
            record, _ = read_record("after", 1)
            checks["went on to N5"] = bool(record) and record["ln"] == 5

        elif scenario == "hold-retract":
            send("$112=60")
            send("$791=0.5")
            wait_until(link, retract_started)
            link.send(b"!")
            messages = []

            def keep_messages(line):
                if not is_status(line):
                    messages.append(line)
                return is_status(line)

            started = time.monotonic()
            while time.monotonic() - started < 60:
                status = wait_for(link, keep_messages, 2)
                if status and status.startswith("<Alarm"):
                    break
                time.sleep(1)
            result["messages"] = messages
            record, _ = read_record("after")
            checks["alarm 81"] = "ALARM:81" in messages
            checks["stopped"] = bool(record) and record["state"] == "Stopped"
    finally:
        if link:
            try:
                link.close()
            except Exception:
                pass
        sim.stop()
        result["checks"] = checks
        result["passed"] = bool(checks) and all(checks.values())
        with open(f"{args.run}/result.json", "w") as f:
            json.dump(result, f, indent=1)
        print(json.dumps({"scenario": scenario, "checks": checks, "passed": result["passed"]}))

    sys.exit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
