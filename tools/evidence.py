"""Evidence runs for the flight recorder in the FlexiHAL simulator.

One scenario per invocation (each stays under two minutes):

  reference              the job, uninterrupted
  hang     --via ws|usb  sender stops sending and reading, link stays open
  close    --via ws|usb  sender closes the link (WebSocket close / USB port closed)
  power    --via ws|usb  controller power cut (simulator killed) mid-job
  dwell    --via ws|usb  6 s G4 dwell with the spindle on and a line-at-a-time
                         sender: must NOT retract; then the sender hangs: must
  events                 provoke each kind of event-log entry (garbled lines,
                         chattering input, limit trip, e-stop, links coming and
                         going, power cut) and read back $FRLOG

Interrupted scenarios then reconnect (over --resume-via), read $FR, home and
resume the job from the record. Output: <run>/summary.json, sender logs, and
the simulator's step logs (machine position in steps, one file per boot).
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from link import open_link
from simrun import Sim
from stream import Streamer, load_program, numbered, query_fr, modal_scan, resume_preamble, resume_lines

G54 = (-300.0, -300.0, -60.0)   # work zero in machine coordinates


# The spike's machine: X/Y 6000 mm/min and 500 mm/s², Z 3000 mm/min and 300 mm/s².
MACHINE = ["$110=6000", "$111=6000", "$112=3000", "$120=500", "$121=500", "$122=300"]


def setup_machine(link):
    replies = []
    for line in MACHINE:
        replies += link.command(line)
    replies += link.command("$H", timeout=10)
    replies += link.command(f"G10L2P1X{G54[0]}Y{G54[1]}Z{G54[2]}")
    replies += link.command("$10=511")   # full status reports
    return replies


def settings(link, ids):
    out = {}
    for line in link.command("$$", timeout=3):
        if line.startswith("$") and "=" in line:
            key, _, value = line[1:].partition("=")
            if key.isdigit() and int(key) in ids:
                out[int(key)] = value
    return out


def wait_for(link, pred, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.send(b"?")
        line = link.readline(0.3)
        while line is not None:
            if pred(line):
                return line
            line = link.readline(0.05)
    return None


def resume(sim, via, program, out, log):
    link = open_link(sim.usb if via == "usb" else sim.ws)
    link.lines_until(lambda l: False, 0.5)
    record = query_fr(link)
    out["record_before_resume"] = record
    job, lines = load_program(program)
    if record is None or record["job"] != job:
        out["resume"] = "refused: job mismatch"
        return link
    out["unlock"] = link.command("$X")
    out["home"] = link.command("$H", timeout=10)
    st = modal_scan(lines, record["ln"])
    pre = resume_preamble(job, st)
    rest = resume_lines(lines, record, st)
    out["resume_preamble"] = pre
    out["resume_first_line"] = rest[0]
    streamer = Streamer(link, log=log)
    result, _ = streamer.send_lines(pre + rest)
    finished = streamer.drain(timeout=100)
    out["resume"] = {"result": result, "finished": finished, "errors": streamer.errors,
                     "alarms": streamer.alarms}
    out["record_after_resume"] = query_fr(link)
    return link


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["reference", "hang", "close", "power", "dwell", "events"])
    ap.add_argument("--sim", required=True)
    ap.add_argument("--program", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--via", choices=["ws", "usb"], default="ws")
    ap.add_argument("--resume-via", choices=["ws", "usb"], default=None)
    ap.add_argument("--at", type=float, default=0.4, help="interrupt after this fraction of the lines")
    ap.add_argument("--speed", type=float, default=4.0)
    args = ap.parse_args()

    out = {"scenario": args.scenario, "via": args.via, "speed": args.speed, "g54": G54}
    sim = Sim(args.sim, args.run, speed=args.speed).start()
    log = open(os.path.join(args.run, "sender.jsonl"), "w", buffering=1)  # line-buffered: survives the cap
    try:
        link = open_link(sim.usb if args.via == "usb" else sim.ws)
        link.lines_until(lambda l: False, 0.5)
        out["setup"] = setup_machine(link)
        out["settings"] = settings(link, {100, 101, 102, 790, 791, 792})
        job, lines = load_program(args.program)
        out["job"] = f"{job:08X}"
        out["lines"] = len(lines)

        if args.scenario == "dwell":
            run_dwell(link, sim, out, log)
            return

        if args.scenario == "events":
            run_events(link, sim, out)
            return

        streamer = Streamer(link, log=log)
        cut_index = int(args.at * len(lines)) if args.scenario != "reference" else None
        stop = None
        if cut_index is not None:
            action = "close" if args.scenario == "close" else "hang"
            stop = lambda i, t: action if i >= cut_index + 1 else None   # +1: after the $FRJ line
        result, index = streamer.send_lines([f"$FRJ={job:08X}"] + numbered(lines), stop)
        out["stream_result"] = result
        out["lines_sent"] = index - 1
        out["last_line_sent"] = lines[index - 2][0] if index >= 2 else None

        if args.scenario == "reference":
            out["finished"] = streamer.drain(timeout=100)
            out["record"] = query_fr(link)
            return

        if args.scenario == "power":
            time.sleep(0.3)                    # mid-motion
            sim.kill()
            out["power_cut_at"] = time.monotonic()
            sim.start()                        # reboot
        elif args.scenario == "hang":
            time.sleep(12)                     # PC frozen: nothing sent or read
            link.close()
        else:
            link.close()
            time.sleep(8)

        resume(sim, args.resume_via or args.via, args.program, out, log)
    finally:
        sim.stop()
        log.close()
        with open(os.path.join(args.run, "summary.json"), "w") as f:
            json.dump(out, f, indent=1)
        print(json.dumps({k: out[k] for k in out if k not in ("setup", "resume_preamble")}, indent=1)[:4000])


def run_dwell(link, sim, out, log):
    """Line-at-a-time: each line waits for its ok, so the dwell leaves the input empty."""
    job = 0xD3E11
    replies = {}
    t0 = time.monotonic()
    for line in [f"$FRJ={job:08X}", "N1G21G90G54", "N2G0X10Y10Z5", "N3S12000M3", "N4G1Z-1F300",
                 "N5G4P6", "N6G1X20F1000"]:
        replies[line] = (round(time.monotonic() - t0, 2), link.command(line, timeout=40))
    out["dwell_replies"] = replies
    out["record_after_dwell"] = query_fr(link)
    out["status_after_dwell"] = wait_for(link, lambda l: l.startswith("<"), 2)
    # Now the sender hangs with the spindle on: this must retract.
    time.sleep(6)
    link.close()
    time.sleep(1)
    link2 = open_link(sim.usb if out["via"] == "ws" else sim.ws)
    link2.lines_until(lambda l: False, 0.5)
    out["record_after_hang"] = query_fr(link2)
    out["frlog"] = link2.command("$FRLOG=12", timeout=5)
    link2.close()


def run_events(link, sim, out):
    """Each kind of event-log entry, then the log read back after a power cut."""
    steps = {}
    steps["clear"] = link.command("$FRLOGC")
    # Garbled lines, as noise on the sender link would make them: three of one kind, one of another.
    steps["garbled"] = [link.command(l) for l in ("G1X1Y", "G1X1Y", "G1X1Y", "Q123")]
    # A limit switch tripping with hard limits on, then the reset to clear it.
    steps["hard_limits"] = link.command("$21=1")
    sim.key("x"); time.sleep(0.5); sim.key("x")
    time.sleep(0.5)
    link.send(b"\x18"); time.sleep(0.5)
    steps["after_limit"] = link.command("$H", timeout=10)
    # A chattering feed-hold input: five presses in quick succession, then a reset
    # so the hold doesn't hang over the next steps.
    for _ in range(5):
        sim.key("h"); time.sleep(0.05); sim.key("h"); time.sleep(0.05)
    time.sleep(0.5)
    link.send(b"\x18"); time.sleep(0.5)
    steps["after_hold"] = link.command("$X")
    # E-stop pressed and released, then the reset.
    sim.key("e"); time.sleep(0.5); sim.key("e"); time.sleep(0.5)
    link.send(b"\x18"); time.sleep(0.5)
    steps["after_estop"] = link.command("$H", timeout=10)
    # The network sender comes and goes; the USB sender closes and reopens.
    ws = open_link(sim.ws)
    ws.lines_until(lambda l: False, 0.5)
    ws.close()
    time.sleep(0.5)
    link.close()
    time.sleep(0.5)
    # Power cut and restart.
    sim.kill()
    sim.start()
    link = open_link(sim.usb)
    link.lines_until(lambda l: False, 0.5)
    out["steps"] = steps
    out["frlog"] = link.command("$FRLOG=50", timeout=5)
    link.close()


if __name__ == "__main__":
    main()
