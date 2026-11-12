"""Evidence runs for the flight recorder in the FlexiHAL simulator.

One scenario per run, each kept under two minutes:

  reference              the job, uninterrupted
  hang     --via ws|usb  the sender stops sending and reading but the link stays open
  close    --via ws|usb  the sender closes the link (WebSocket close, or the USB port)
  power    --via ws|usb  the controller loses power (the simulator is killed) mid-job
  dwell    --via ws|usb  a 6 s G4 dwell with the spindle on and a sender that waits for
                         each ok. It must not retract. Then the sender hangs, and it must.
  events                 makes each kind of event log entry (garbled lines, a
                         chattering input, a limit trip, e-stop, links coming and
                         going, a power cut) and reads $EVENTS back

The interrupted scenarios then reconnect (over --resume-via), read $FLIGHT, home
and resume the job from the record. Writes <run>/summary.json, the sender's log
and the simulator's step logs (machine position in steps, one file per boot).
"""

import argparse
import json
import os
import time

from link import open_link
from simrun import Sim
from stream import Streamer, load_program, numbered, query_flight, modal_scan, resume_preamble, resume_lines

G54 = (-300.0, -300.0, -60.0)   # work zero in machine coordinates

# X/Y 6000 mm/min and 500 mm/s², Z 3000 mm/min and 300 mm/s².
MACHINE = ["$110=6000", "$111=6000", "$112=3000", "$120=500", "$121=500", "$122=300"]


def setup_machine(link):
    replies = []
    for line in MACHINE:
        replies += link.command(line)
    replies += link.command("$H", timeout=10)
    replies += link.command(f"G10L2P1X{G54[0]}Y{G54[1]}Z{G54[2]}")
    replies += link.command("$10=511")   # full status reports
    return replies


def read_settings(link, wanted):
    values = {}
    for line in link.command("$$", timeout=3):
        if line.startswith("$") and "=" in line:
            key, _, value = line[1:].partition("=")
            if key.isdigit() and int(key) in wanted:
                values[int(key)] = value
    return values


def wait_for(link, matches, timeout):
    """Keep asking for status until a line matches, and return it (None on timeout)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.send(b"?")
        line = link.readline(0.3)
        while line is not None:
            if matches(line):
                return line
            line = link.readline(0.05)
    return None


def skip_greeting(link, seconds=0.5):
    link.lines_until(lambda line: False, seconds)


def resume(sim, via, program, summary, log):
    link = open_link(sim.usb if via == "usb" else sim.ws)
    skip_greeting(link)
    # Until the board has retracted or stopped it reads nothing at all, so keep asking.
    record = None
    deadline = time.monotonic() + 30
    while record is None and time.monotonic() < deadline:
        record = query_flight(link)
    summary["record_before_resume"] = record
    summary["log_before_resume"] = link.command("$EVENTS=3", timeout=5)
    job, lines = load_program(program)
    if record is None or record["job"] != job:
        summary["resume"] = "refused: job mismatch"
        return link
    summary["unlock"] = link.command("$X")
    summary["home"] = link.command("$H", timeout=10)
    modes = modal_scan(lines, record["ln"])
    preamble = resume_preamble(job, modes)
    remaining = resume_lines(lines, record, modes)
    summary["resume_preamble"] = preamble
    summary["resume_first_line"] = remaining[0]
    streamer = Streamer(link, log=log)
    result, _ = streamer.send_lines(preamble + remaining)
    finished = streamer.drain(timeout=100)
    summary["resume"] = {"result": result, "finished": finished, "errors": streamer.errors,
                         "alarms": streamer.alarms}
    summary["record_after_resume"] = query_flight(link)
    return link


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=["reference", "hang", "close", "power", "dwell", "events"])
    parser.add_argument("--sim", required=True)
    parser.add_argument("--program", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--via", choices=["ws", "usb"], default="ws")
    parser.add_argument("--resume-via", choices=["ws", "usb"], default=None)
    parser.add_argument("--at", type=float, default=0.4, help="interrupt after this fraction of the lines")
    parser.add_argument("--speed", type=float, default=4.0)
    parser.add_argument("--sd", action="store_true", help="record on the SD card (no FRAM)")
    args = parser.parse_args()

    summary = {"scenario": args.scenario, "via": args.via, "speed": args.speed, "g54": G54,
               "storage": "sd" if args.sd else "fram"}
    sim = Sim(args.sim, args.run, speed=args.speed, sd=args.sd).start()
    log = open(os.path.join(args.run, "sender.jsonl"), "w", buffering=1)  # line buffered, so it survives the run cap
    try:
        link = open_link(sim.usb if args.via == "usb" else sim.ws)
        skip_greeting(link)
        summary["setup"] = setup_machine(link)
        summary["settings"] = read_settings(link, {100, 101, 102, 790, 791, 792})
        job, lines = load_program(args.program)
        summary["job"] = f"{job:08X}"
        summary["lines"] = len(lines)

        if args.scenario == "dwell":
            run_dwell(link, sim, summary)
            return

        if args.scenario == "events":
            run_events(link, sim, summary)
            return

        streamer = Streamer(link, log=log)
        stop = None
        if args.scenario != "reference":
            cut_at = int(args.at * len(lines)) + 1     # + 1 for the $FLIGHT=<job> line first
            action = "close" if args.scenario == "close" else "hang"
            stop = lambda index, elapsed: action if index >= cut_at else None
        result, index = streamer.send_lines([f"$FLIGHT={job:08X}"] + numbered(lines), stop)
        summary["stream_result"] = result
        summary["lines_sent"] = index - 1
        summary["last_line_sent"] = lines[index - 2][0] if index >= 2 else None

        if args.scenario == "reference":
            summary["finished"] = streamer.drain(timeout=100)
            summary["record"] = query_flight(link)
            return

        if args.scenario == "power":
            time.sleep(0.3)                    # mid-motion
            sim.kill()
            summary["power_cut_at"] = time.monotonic()
            sim.start()                        # and boot again
        elif args.scenario == "hang":
            time.sleep(12)                     # the PC has frozen, nothing sent or read
            link.close()
        else:
            link.close()
            time.sleep(8)

        resume(sim, args.resume_via or args.via, args.program, summary, log)
    finally:
        sim.stop()
        log.close()
        with open(os.path.join(args.run, "summary.json"), "w") as f:
            json.dump(summary, f, indent=1)
        print(json.dumps({key: summary[key] for key in summary if key not in ("setup", "resume_preamble")}, indent=1)[:4000])


def run_dwell(link, sim, summary):
    """Sends a line at a time and waits for each ok, so the dwell leaves the input empty."""
    job = 0xD3E11
    replies = {}
    started = time.monotonic()
    for line in [f"$FLIGHT={job:08X}", "N1G21G90G54", "N2G0X10Y10Z5", "N3S12000M3", "N4G1Z-1F300",
                 "N5G4P6", "N6G1X20F1000"]:
        replies[line] = (round(time.monotonic() - started, 2), link.command(line, timeout=40))
    summary["dwell_replies"] = replies
    summary["record_after_dwell"] = query_flight(link)
    summary["status_after_dwell"] = wait_for(link, lambda line: line.startswith("<"), 2)
    # Now the sender hangs with the spindle on, and this time it has to retract.
    time.sleep(6)
    link.close()
    time.sleep(1)
    other_link = open_link(sim.usb if summary["via"] == "ws" else sim.ws)
    skip_greeting(other_link)
    summary["record_after_hang"] = query_flight(other_link)
    summary["events"] = other_link.command("$EVENTS=12", timeout=5)
    other_link.close()


def run_events(link, sim, summary):
    """Makes each kind of event log entry, then reads the log back after a power cut."""
    steps = {}
    steps["clear"] = link.command("$EVENTSCLEAR")
    # Garbled lines, like noise on the sender's link would cause. Three of one kind, one of another.
    steps["garbled"] = [link.command(line) for line in ("G1X1Y", "G1X1Y", "G1X1Y", "Q123")]
    # A limit switch tripping with hard limits on, then a reset to clear it.
    steps["hard_limits"] = link.command("$21=1")
    sim.key("x")
    time.sleep(0.5)
    sim.key("x")
    time.sleep(0.5)
    link.send(b"\x18")
    time.sleep(0.5)
    steps["after_limit"] = link.command("$H", timeout=10)
    # A chattering feed hold input, five presses in quick succession, then a reset so
    # the hold doesn't get in the way of the next steps.
    for _ in range(5):
        sim.key("h")
        time.sleep(0.05)
        sim.key("h")
        time.sleep(0.05)
    time.sleep(0.5)
    link.send(b"\x18")
    time.sleep(0.5)
    steps["after_hold"] = link.command("$X")
    # E-stop pressed and released, then a reset.
    sim.key("e")
    time.sleep(0.5)
    sim.key("e")
    time.sleep(0.5)
    link.send(b"\x18")
    time.sleep(0.5)
    steps["after_estop"] = link.command("$H", timeout=10)
    # A network sender connects and goes away, then the USB sender closes and opens again.
    network_link = open_link(sim.ws)
    skip_greeting(network_link)
    network_link.close()
    time.sleep(0.5)
    link.close()
    time.sleep(0.5)
    # Power cut and restart.
    sim.kill()
    sim.start()
    link = open_link(sim.usb)
    skip_greeting(link)
    summary["steps"] = steps
    summary["events"] = link.command("$EVENTS=50", timeout=5)
    link.close()


if __name__ == "__main__":
    main()
