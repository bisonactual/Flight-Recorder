"""SD card evidence runs for the flight recorder, on the simulator's card model.

The simulator has to be built with the SD card (GRBLHAL-Sim branch sd-card); it
then records on card.img in the run directory, with no FRAM fitted (-F). The
card model (src/sim_sdcard.c there) times commands, data and write busy the way
the board's diskio.c sees a card on SPI, and a power cut while the card is busy
tears the sector being written. Its trace (card.img.trace) has every write.

  power   power cuts at random points. Each cycle boots, streams a job and dies,
          either during its n-th card write (SIM_SD_CUT_AT, n picked at random)
          or at a random moment. Before the next boot the two record copies
          are read straight from the card image; the next boot must report
          the newest valid one, never one older than the last record the card
          finished, and the card must still mount.
  pause   long card pauses (SIM_SD_PAUSE) during a dense job over WebSocket: how
          long grblHAL's foreground and the WebSocket stream were held up, and
          how stale the record got.
  storm   an EMI storm: a burst of alternating errors and inputs, then an alarm
          and a power cut 50 ms later, and a plain error and a power cut 50 ms
          later. How many card writes the burst took, that the alarm got to the
          card and the error didn't (batching), and what the log kept.
  remount the card taken out and put back (the simulator's u and c keys, which
          unmount and mount it as card detect does). Nothing is written while
          it is out, the same card is taken back and the record saved again;
          a new blank card in its place is refused until a restart.

Usage:
  sd_evidence.py power --sim <binary> --run <dir> [--cycles 200] [--tear-unit 1]
  sd_evidence.py pause --sim <binary> --run <dir> [--seconds 20] [--pause 0.1,200,250]
  sd_evidence.py storm --sim <binary> --run <dir> [--events 1000]
  sd_evidence.py remount --sim <binary> --run <dir>
Each writes <run>/summary.json and prints the checks with a short summary. Exit
status 1 if one failed. The run folder is emptied first, so it has to be new,
empty, or an earlier run's (with summary.json or card.img in it).
"""

import argparse
import json
import os
import random
import re
import shutil
import struct
import sys
import time
import zlib

from evidence import setup_machine, skip_greeting
from link import open_link
from simrun import Sim
from stream import query_flight

FR_MAGIC = 0x31524646
RECORD = struct.Struct("<5I5fi5B3sI")       # fr_record_t, 56 bytes
MAGIC_HEX = struct.pack("<I", FR_MAGIC).hex()
LOG_MAGIC_HEX = struct.pack("<I", 0x31474C46).hex()     # the event log header
LOG_SECTORS = 33                            # the event ring's sectors, right after the header

# As the plugin names them in [FLIGHT:...], by number.
STATES = ["Idle", "Running", "Stopped", "Retracted", "Completed"]
REASONS = ["none", "timeout", "usb", "network", "alarm", "reset", "reboot"]


# The card model's trace and image

def read_trace(path):
    """One dict per simulator start: its writes (W), finished writes (C), cut (X) and blocked times (B)."""
    boots = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if line.startswith("# start"):
                boots.append({"start": line.strip(), "W": [], "C": [], "X": None, "B": []})
            elif not boots:
                continue
            elif parts[0] == "W":
                boots[-1]["W"].append({"t": float(parts[1]), "sector": int(parts[2]), "busy": float(parts[3]), "tear": parts[4]})
            elif parts[0] == "C":
                boots[-1]["C"].append({"t": float(parts[1]), "sector": int(parts[2]), "data": bytes.fromhex(parts[3])})
            elif parts[0] == "X":
                boots[-1]["X"] = {"t": float(parts[1]), "sector": int(parts[2])}
            elif parts[0] == "B":
                boots[-1]["B"].append({"t": float(parts[1]), "us": int(parts[2]), "what": parts[3],
                                       "waited": int(parts[4]) if len(parts) > 4 else 0})
    return boots


def decode_record(data):
    fields = RECORD.unpack(data[:RECORD.size])
    record = {"magic": fields[0], "seq": fields[1], "boot": fields[2], "job": fields[3], "ln": fields[4],
              "state": fields[11], "reason": fields[12], "crc": fields[17]}
    record["valid"] = record["magic"] == FR_MAGIC and zlib.crc32(data[:RECORD.size - 4]) == record["crc"]
    return record


def newest_valid(records):
    """Same rule as the plugin: the valid copy with the later seq, allowing for wrap."""
    valid = [r for r in records if r["valid"]]
    if len(valid) == 2:
        a, b = valid
        return b if ((b["seq"] - a["seq"]) & 0xFFFFFFFF) < 0x80000000 and b["seq"] != a["seq"] else a
    return valid[0] if valid else None


def read_sector(image, sector):
    with open(image, "rb") as f:
        f.seek(sector * 512)
        return f.read(512)


# The simulator

def connect(sim, timeout=5.0):
    """Connects over WebSocket once the simulator listens, and waits for the recorder's storage to be up.

    Raises ConnectionError if the simulator exits first, and RuntimeError if it runs but
    the recorder doesn't answer $FLIGHT in time.
    """
    deadline = time.monotonic() + timeout
    while True:
        try:
            link = open_link(sim.ws)
            break
        except OSError:
            if time.monotonic() > deadline or sim.proc.poll() is not None:
                raise
            time.sleep(0.1)
    skip_greeting(link, 0.3)
    while time.monotonic() < deadline:
        if any(line.startswith("[FLIGHT:") for line in link.command("$FLIGHT", timeout=1)):
            return link
        time.sleep(0.1)
    link.close()
    if sim.proc.poll() is not None:
        raise ConnectionError("the simulator exited before the recorder answered")
    raise RuntimeError(f"no [FLIGHT:] reply within {timeout} s")


def sd_stats(link, reset=False):
    replies = link.command("$FLIGHTSD=R" if reset else "$FLIGHTSD", timeout=3)
    line = next((line for line in replies if line.startswith("[FLIGHTSD:")), None)
    if not line:
        return None
    stats = {}
    for part in line[len("[FLIGHTSD:"):-1].split("|"):
        key, _, value = part.partition("=")
        stats[key] = float(value) if value and re.match(r"^[\d.]+$", value) else (value or key)
    return stats


def events(link, count=1000):
    return [line for line in link.command(f"$EVENTS={count}", timeout=10) if line.startswith("[EVENT:")]


def plugin_line(link):
    return next((line for line in link.command("$I", timeout=3) if "Flight Recorder" in line), None)


def program(lines=400, step=2.0, feed=1500):
    """A numbered zigzag of short cuts with the spindle on, so the line changes every 80 ms or so."""
    out = ["G21G90G54", "G0X0Y0Z5", "S12000M3", "G1Z-1F600"]
    x, y, direction = 0.0, 0.0, 1
    while len(out) < lines:
        x += direction * step
        if x > 40 or x < 0:
            direction = -direction
            x += 2 * direction * step
            y += 1.0
        out.append(f"G1X{x:.3f}Y{y:.3f}F{feed}")
    return [f"N{n}{text}" for n, text in enumerate(out, 1)]


def stream(link, lines, until, keep_going=lambda: True, rx_size=900):
    """Character counting until the lines run out, the time is up or the link dies. Returns lines sent."""
    in_flight, sent = [], 0
    try:
        while time.monotonic() < until and keep_going():
            while sent < len(lines) and sum(in_flight) + len(lines[sent]) + 1 <= rx_size:
                link.send((lines[sent] + "\n").encode())
                in_flight.append(len(lines[sent]) + 1)
                sent += 1
            line = link.readline(0.05)
            if line is not None and (line == "ok" or line.startswith("error")) and in_flight:
                in_flight.pop(0)
            if sent == len(lines) and not in_flight:
                break
    except (OSError, ConnectionError):
        pass
    return sent


# power

def run_power(args, summary):
    rng = random.Random(args.seed)
    sim = Sim(args.sim, args.run, ws_port=args.port, speed=args.speed, sd=True,
              env={"SIM_SD_SEED": args.seed, "SIM_SD_TEAR_UNIT": args.tear_unit})
    trace_path = sim.card + ".trace"

    # A first boot makes and formats the card and FLIGHT.DAT, and writes both record copies.
    sim.start()
    link = connect(sim)
    summary["setup"] = setup_machine(link)
    summary["plugin"] = plugin_line(link)
    link.command("$FLIGHTCLEAR")
    time.sleep(1.5)                     # everything batched goes out and the card finishes
    link.close()
    sim.kill()

    setup_trace = read_trace(trace_path)
    copies = sorted({c["sector"] for boot in setup_trace for c in boot["C"] if c["data"].hex().startswith(MAGIC_HEX)})
    headers = sorted({c["sector"] for boot in setup_trace for c in boot["C"] if c["data"].hex().startswith(LOG_MAGIC_HEX)})
    summary["record_sectors"] = copies
    summary["header_sector"] = headers
    if len(copies) != 2:
        summary["error"] = f"expected two record sectors in the trace, found {len(copies)}"
        return {"two record sectors": False}
    if len(headers) != 1:
        summary["error"] = f"expected one log header sector in the trace, found {len(headers)}"
        return {"one log header sector": False}

    lines = program()
    job = zlib.crc32("\n".join(lines).encode()) & 0xFFFFFFFF
    tally = {"cycles": 0, "cut_during_write": 0, "cut_at_random_time": 0, "in_flight": {}, "tear": {},
             "loaded_torn_write": 0, "unreadable_events_max": 0, "boot_died_before_link": 0}
    failures = []
    with open(os.path.join(args.run, "cycles.jsonl"), "w", buffering=1) as log:
        for cycle in range(args.cycles):
            entry = power_cycle(args, sim, trace_path, copies, headers[0], lines, job, cycle, rng, tally)
            log.write(json.dumps(entry) + "\n")
            checks = entry["checks"]
            if not all(checks.values()):
                failures.append(entry)
            print(f"cycle {cycle}: cut_at={entry['cut_at']} in_flight={(entry['in_flight'] or {}).get('kind', 'none')} "
                  f"ln={entry['reported'] and entry['reported']['ln']} "
                  f"{'ok' if all(checks.values()) else 'FAIL ' + str(checks)}", flush=True)

    summary["tally"] = tally
    summary["failures"] = failures[:20]
    return {"every cycle passed": not failures}


def newest_finished(trace, copies):
    """The last record copy the card finished writing, in any boot so far."""
    finished = [c for boot in trace for c in boot["C"] if c["sector"] in copies]
    return decode_record(finished[-1]["data"]) if finished else None


def power_cycle(args, sim, trace_path, copies, header, lines, job, cycle, rng, tally):
    """One boot that streams the job and is cut, then a boot that checks what it left."""
    tally["cycles"] += 1
    by_write = rng.random() >= args.random_share
    cut_at = rng.randint(1, args.max_cut) if by_write else 0
    kill_after = rng.uniform(0.3, args.stream_seconds) if not by_write else None
    sim.env.update({"SIM_SD_CUT_AT": cut_at, "SIM_SD_SEED": args.seed * 100000 + cycle})
    timing = {}
    t_cycle = time.monotonic()
    sim.start()
    started = time.monotonic()
    timing["boot"] = round(started - t_cycle, 2)
    try:
        link = connect(sim, timeout=4)
        link.command("$H", timeout=5)
        link.command(f"$FLIGHT={job:08X}")
        stream(link, lines, started + (kill_after or args.stream_seconds), lambda: sim.proc.poll() is None)
        link.close()
    except (OSError, ConnectionError):
        if sim.proc.poll() is not None and time.monotonic() - started < 2:
            tally["boot_died_before_link"] += 1
    died_by_cut = sim.proc.poll() is not None
    sim.kill()
    timing["run"] = round(time.monotonic() - started, 2)

    trace = read_trace(trace_path)
    boot = trace[-1]
    last_finished = newest_finished(trace, copies)
    open_writes = {}
    for w in boot["W"]:
        open_writes[w["sector"]] = w
    for c in boot["C"]:
        if c["sector"] in open_writes and open_writes[c["sector"]]["t"] <= c["t"]:
            del open_writes[c["sector"]]
    in_flight = max(open_writes.values(), key=lambda w: w["t"]) if open_writes else None
    kind = "none"
    if in_flight:
        kind = "record" if in_flight["sector"] in copies else (
            "header" if in_flight["sector"] == header else "log" if header < in_flight["sector"] <= header + LOG_SECTORS else "other")
        tally["tear"][in_flight["tear"]] = tally["tear"].get(in_flight["tear"], 0) + 1
    tally["in_flight"][kind] = tally["in_flight"].get(kind, 0) + 1
    tally["cut_during_write" if died_by_cut and cut_at else "cut_at_random_time"] += 1

    on_card = [decode_record(read_sector(sim.card, sector)) for sector in copies]
    expected = newest_valid(on_card)

    # The next boot: no cut. A torn log sector is near the head, in the newest 200 events.
    t_check = time.monotonic()
    sim.env["SIM_SD_CUT_AT"] = 0
    sim.start()
    link = connect(sim)
    reported = query_flight(link)
    mounted = plugin_line(link) or ""
    log_lines = events(link, 200)
    unreadable = [line for line in log_lines if "unreadable" in line]
    time.sleep(1.2)                 # let this boot's own writes finish before the next cut
    link.close()
    sim.kill()
    timing["check"] = round(time.monotonic() - t_check, 2)

    checks = {
        "a copy survived": expected is not None,
        "not older than the last finished": expected is None or last_finished is None or
                                            ((expected["seq"] - last_finished["seq"]) & 0xFFFFFFFF) < 0x80000000,
        "card mounts, SD storage": "(SD card)" in mounted,
        "torn log sector at most": len(unreadable) <= 32,
    }
    if expected and reported:
        # A record still Running at boot is reported as Stopped, why=reboot.
        running = expected["state"] == STATES.index("Running")
        state = "Stopped" if running else STATES[expected["state"]]
        why = "reboot" if running else REASONS[expected["reason"]]
        checks["boot reports the newest valid copy"] =             (reported["ln"], reported["job"], int(reported["boot"]), reported["state"], reported["why"]) ==             (expected["ln"], expected["job"], expected["boot"] + 1, state, why)
    else:
        checks["boot reports the newest valid copy"] = reported is not None
    if expected and last_finished and expected["seq"] != last_finished["seq"]:
        tally["loaded_torn_write"] += 1
    tally["unreadable_events_max"] = max(tally["unreadable_events_max"], len(unreadable))

    return {"cycle": cycle, "cut_at": cut_at, "kill_after": kill_after, "died_by_cut": died_by_cut,
            "in_flight": in_flight and {**in_flight, "kind": kind},
            "last_finished": last_finished and {k: last_finished[k] for k in ("seq", "ln", "boot", "state")},
            "on_card": [{k: r[k] for k in ("valid", "seq", "ln")} for r in on_card],
            "reported": reported, "unreadable": len(unreadable), "checks": checks, "timing": timing}


# pause

def run_pause(args, summary):
    sim = Sim(args.sim, args.run, ws_port=args.port, speed=1.0, sd=True,
              env={"SIM_SD_SEED": args.seed, "SIM_SD_PAUSE": args.pause})
    trace_path = sim.card + ".trace"
    sim.start()
    try:
        link = connect(sim)
        summary["plugin"] = plugin_line(link)
        summary["setup"] = setup_machine(link)
        # A dense job: 0.3 mm segments at 2000 mm/min, about 9 ms each.
        lines = program(lines=args.lines, step=0.3, feed=2000)
        job = zlib.crc32("\n".join(lines).encode()) & 0xFFFFFFFF
        link.command("$791=30")
        link.command(f"$FLIGHT={job:08X}")
        sd_stats(link, reset=True)
        trace_before = len(read_trace(trace_path)[-1]["W"])

        latencies, states, asked = [], [], None
        in_flight, sent = [], 0
        started = time.monotonic()
        next_status = started
        while time.monotonic() - started < args.seconds:
            while sent < len(lines) and sum(in_flight) + len(lines[sent]) + 1 <= 900:
                link.send((lines[sent] + "\n").encode())
                in_flight.append(len(lines[sent]) + 1)
                sent += 1
            now = time.monotonic()
            if asked is None and now >= next_status:
                link.send(b"?")
                asked = now
                next_status = now + 0.05
            line = link.readline(0.01)
            if line is None:
                continue
            if (line == "ok" or line.startswith("error")) and in_flight:
                in_flight.pop(0)
            elif line.startswith("<") and asked is not None:
                latencies.append(time.monotonic() - asked)
                states.append(line[1:].split("|")[0])
                asked = None
            if sent == len(lines) and not in_flight:
                break

        # Let the queued lines finish, as grblHAL only takes $ commands once the machine is idle.
        # $791 is set long enough that the idle spindle doesn't count as a lost stream.
        deadline = time.monotonic() + 15
        while in_flight and time.monotonic() < deadline:
            line = link.readline(0.1)
            if line is not None and (line == "ok" or line.startswith("error")):
                in_flight.pop(0)
        while time.monotonic() < deadline:
            link.send(b"?")
            line = link.readline(0.3)
            if line and line.startswith("<Idle"):
                break
        summary["stats"] = sd_stats(link)

        # What the card went through while the job ran. Times in the trace are simulated.
        boot = read_trace(trace_path)[-1]
        writes = boot["W"][trace_before:]
        t0, t1 = (writes[0]["t"], writes[-1]["t"]) if writes else (0.0, 0.0)
        copies = sorted({c["sector"] for c in boot["C"] if c["data"].hex().startswith(MAGIC_HEX)})
        # How far the record on the card fell behind: the time between records reaching the card
        # from line 10 on, while the job runs and the line changes every 9 ms or so.
        commits = [(c["t"], decode_record(c["data"])) for c in boot["C"] if c["sector"] in copies and t0 <= c["t"] <= t1]
        dense = [(t, r) for t, r in commits if r["ln"] >= 10 and r["state"] == 1]
        gaps = [b[0] - a[0] for a, b in zip(dense, dense[1:])]
        summary["record"] = {"commits_dense": len(dense), "lines_behind_max": max((b[1]["ln"] - a[1]["ln"] for a, b in zip(dense, dense[1:])), default=0),
                             "superseded_in_ram": sum(b[1]["seq"] - a[1]["seq"] - 1 for a, b in zip(commits, commits[1:]))}
        latencies.sort()
        summary["lines_sent"] = sent
        summary["status"] = {
            "replies": len(latencies),
            "latency_ms_p50": round(1000 * latencies[len(latencies) // 2], 1) if latencies else None,
            "latency_ms_p99": round(1000 * latencies[int(len(latencies) * 0.99)], 1) if latencies else None,
            "latency_ms_max": round(1000 * latencies[-1], 1) if latencies else None,
            "idle_mid_job": sum(1 for s in states[5:-5] if s == "Idle"),
        }
        summary["card"] = {
            "writes": len(writes),
            "long_pauses": sum(1 for w in writes if w["busy"] >= 100),
            "busy_ms_max": max((w["busy"] for w in writes), default=0),
            "record_age_ms_max": round(max(gaps, default=0), 1),
            "disk_calls_over_1ms": sum(1 for b in boot["B"] if t0 <= b["t"] <= t1 + 300),
            "foreground_in_one_disk_call_us_max": max((b["us"] for b in boot["B"] if t0 <= b["t"] <= t1 + 300), default=0),
            "disk_calls_that_waited_for_the_card": sum(1 for b in boot["B"] if t0 <= b["t"] <= t1 + 300 and b["waited"]),
            "longest_wait_for_the_card_us": max((b["waited"] for b in boot["B"] if t0 <= b["t"] <= t1 + 300), default=0),
        }
        link.close()
    finally:
        time.sleep(1.0)
        sim.kill()

    stats = summary["stats"] or {}
    checks = {
        "no idle mid-job": summary["status"]["idle_mid_job"] == 0,
        "never waited for a busy card": summary["card"]["disk_calls_that_waited_for_the_card"] == 0,
        "foreground gap under 20 ms": stats.get("gap_max", 1e9) < 20,   # the simulator's clock moves in 10 ms frames
    }
    if args.pause.split(",")[0] not in ("0", "0.0"):
        checks["had long pauses"] = summary["card"]["long_pauses"] > 0
    return checks


# storm

def run_storm(args, summary):
    sim = Sim(args.sim, args.run, ws_port=args.port, speed=1.0, sd=True, env={"SIM_SD_SEED": args.seed})
    trace_path = sim.card + ".trace"
    sim.start()
    link = connect(sim)
    summary["plugin"] = plugin_line(link)
    summary["setup"] = setup_machine(link)
    link.command("$EVENTSCLEAR")
    time.sleep(1.5)
    boot_writes = lambda: read_trace(trace_path)[-1]["W"]
    before = len(boot_writes())

    # The burst, with the machine idle: feed hold and cycle start pressed in turn 10 ms apart, so
    # each press gets its own entry instead of being counted into the last, and a garbled line
    # now and then. In this simulator every error after the first repeats the first one's code,
    # so those all count into one entry.
    started = time.monotonic()
    replies = 0
    for n in range(args.events // 2):
        for key in ("h", "s"):
            sim.key(key)
            sim.key(key)
            time.sleep(0.01)
        if n % 20 == 10:
            link.send(b"Q123\n")
            if link.readline(1.0) is not None:
                replies += 1
    burst_seconds = time.monotonic() - started
    time.sleep(1.5)
    link.lines_until(lambda line: False, 0.5)
    burst_writes = boot_writes()[before:]
    held = events(link)
    summary["burst"] = {"lines": args.events, "seconds": round(burst_seconds, 2), "replies": replies,
                        "card_writes": len(burst_writes), "events_logged": len(held),
                        "kinds": sorted({line.split("|")[3] for line in held if line.count("|") > 3}),
                        "storage_events": [line for line in held if "STORAGE" in line]}

    # An alarm, and the power goes 50 ms after grblHAL reports it.
    summary["stats_after_burst"] = sd_stats(link)
    sim.key("9")                        # alarm 9 injected
    summary["alarm_reply"] = link.lines_until(lambda line: line.startswith("ALARM"), 3)
    time.sleep(0.05)
    sim.kill()
    link.close()
    sim.start()
    link = connect(sim)
    after_alarm = events(link, 20)
    time.sleep(1.5)
    # A plain error, and the power goes 50 ms after grblHAL replies.
    link.command("$X")
    summary["error_reply"] = link.command("Q999")
    time.sleep(0.05)
    sim.kill()
    link.close()
    sim.start()
    link = connect(sim)
    after_error = events(link, 20)
    final = events(link, 1000)
    time.sleep(1.2)
    link.close()
    sim.kill()

    indexes = [int(m.group(1)) for line in final if (m := re.match(r"\[EVENT:(\d+)\|", line))]
    summary["after_alarm_cut"] = after_alarm
    summary["after_error_cut"] = after_error
    summary["log"] = {"events": len(final), "unreadable": sum("unreadable" in line for line in final),
                      "in_order": indexes == sorted(indexes) and len(set(indexes)) == len(indexes)}
    # The alarm was in boot 1 and the plain error in boot 2; each was followed by the cut.
    alarm_kept = any("|boot=1|" in line and "ALARM|alarm=9" in line for line in after_alarm)
    error_kept = any("|boot=2|" in line and "ERROR" in line for line in after_error)
    summary["alarm_kept"] = alarm_kept
    summary["error_on_card"] = error_kept     # False when events are batched: the cut came first
    return {
        "alarm on the card despite the cut": alarm_kept,
        "log readable, in order": summary["log"]["unreadable"] == 0 and summary["log"]["in_order"],
        "no events dropped in the burst": not summary["burst"]["storage_events"],
    }


# remount

def wait_for_text(link, text, timeout=5.0):
    """Reads lines until one has text in it, and returns it. None if none came."""
    lines = link.lines_until(lambda line: text in line, timeout)
    return lines[-1] if lines and text in lines[-1] else None


def wait_idle(link, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.send(b"?")
        line = link.readline(0.3)
        if line and line.startswith("<Idle"):
            return True
    return False


def card_writes(link):
    return (sd_stats(link) or {}).get("writes")


def run_remount(args, summary):
    sim = Sim(args.sim, args.run, ws_port=args.port, speed=1.0, sd=True, env={"SIM_SD_SEED": args.seed})
    steps = summary["steps"] = {}

    def move_to(n):
        link.command(f"N{n}G21G90G0X{n}")
        wait_idle(link)
        time.sleep(1.0)                 # the record goes out and the card finishes

    sim.start()
    try:
        link = connect(sim)
        summary["plugin"] = plugin_line(link)
        summary["setup"] = setup_machine(link)
        link.command("$FLIGHT=0000ABCD")
        move_to(1)

        # The same card out and back in.
        before_out = card_writes(link)
        sim.key("u")
        steps["unmounted"] = wait_for_text(link, "SD card unmounted")
        move_to(2)
        while_out = card_writes(link)
        sim.key("c")
        steps["back"] = wait_for_text(link, "SD card back")
        time.sleep(1.0)
        after_back = card_writes(link)
        steps["writes"] = {"before_out": before_out, "while_out": while_out, "after_back": after_back}
        steps["record_after_back"] = query_flight(link)
        steps["events_after_back"] = events(link, 10)

        # A power cut, to see what the card kept.
        link.close()
        sim.kill()
        sim.start()
        link = connect(sim)
        steps["record_after_cut"] = query_flight(link)
        link.command("$X")
        link.command("$H", timeout=10)
        link.command("$FLIGHT=0000ABCE")
        move_to(3)

        # A new blank card in its place: the simulator makes and formats one when the image is gone.
        sim.key("u")
        steps["unmounted_again"] = wait_for_text(link, "SD card unmounted")
        os.remove(sim.card)
        sim.key("c")
        steps["new_card"] = wait_for_text(link, "FLIGHT.DAT isn't where it was", 15)
        at_swap = card_writes(link)
        move_to(4)
        after_swap = card_writes(link)
        steps["writes_new_card"] = {"at_swap": at_swap, "after": after_swap}

        # After a restart the recorder makes FLIGHT.DAT on the new card and uses it.
        link.close()
        sim.kill()
        sim.start()
        link = connect(sim)
        steps["plugin_after_restart"] = plugin_line(link)
        steps["record_after_restart"] = query_flight(link)
        link.close()
    finally:
        sim.kill()

    back, cut, restart = steps["record_after_back"], steps["record_after_cut"], steps["record_after_restart"]
    return {
        "unmount noticed": steps["unmounted"] is not None,
        "nothing written while out": while_out == before_out,
        "same card taken back": steps["back"] is not None,
        "written again once back": after_back is not None and while_out is not None and after_back > while_out,
        "record from while out is on the card": bool(back and cut) and back["ln"] == 2 and
                                                (cut["ln"], cut["state"], cut["why"]) == (2, "Stopped", "reboot"),
        "new card refused": steps["new_card"] is not None,
        "nothing written to the new card": at_swap is not None and after_swap == at_swap,
        "new card used after a restart": "(SD card)" in (steps["plugin_after_restart"] or "") and
                                         bool(restart) and restart["state"] == "Idle",
    }



# What each scenario prints besides the checks. The whole summary is in summary.json.
SHOWN = {
    "power": ["plugin", "record_sectors", "header_sector", "tally"],
    "pause": ["plugin", "lines_sent", "status", "card", "record", "stats"],
    "storm": ["plugin", "burst", "log", "alarm_kept", "error_on_card"],
    "remount": ["plugin", "steps"],
}


def prepare_run(path):
    """Empties the run folder, but only one that is new, empty or an earlier run's."""
    if os.path.isdir(path) and os.listdir(path):
        if not any(os.path.exists(os.path.join(path, name)) for name in ("summary.json", "card.img")):
            sys.exit(f"{path} has files in it but doesn't look like an earlier run (no summary.json or card.img), "
                     "so it isn't deleted. Pick another --run.")
        shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)


def print_result(summary, path):
    for key in SHOWN[summary["scenario"]]:
        if key in summary:
            print(f"{key}: {json.dumps(summary[key], default=str)}")
    if "error" in summary:
        print(f"error: {summary['error']}")
    for name, ok in summary["checks"].items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    print(f"{'passed' if summary['passed'] else 'FAILED'}, full summary in {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=list(SHOWN))
    parser.add_argument("--sim", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--port", type=int, default=8190, help="WebSocket port, so runs can go side by side")
    parser.add_argument("--cycles", type=int, default=200)
    parser.add_argument("--tear-unit", type=int, default=1, help="sectors a torn write can damage (card model)")
    parser.add_argument("--max-cut", type=int, default=40, help="the cut is during write 1 to this, of a boot")
    parser.add_argument("--random-share", type=float, default=0.25, help="share of cycles cut at a random time instead")
    parser.add_argument("--stream-seconds", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--lines", type=int, default=4000)
    parser.add_argument("--pause", default="0.1,200,250", help="SIM_SD_PAUSE: chance, min ms, max ms")
    parser.add_argument("--events", type=int, default=1000)
    args = parser.parse_args()

    prepare_run(args.run)
    summary = {"scenario": args.scenario, "args": vars(args)}
    checks = {}
    path = os.path.join(args.run, "summary.json")
    try:
        checks = {"power": run_power, "pause": run_pause, "storm": run_storm,
                  "remount": run_remount}[args.scenario](args, summary)
    finally:
        summary["checks"] = checks
        summary["passed"] = bool(checks) and all(checks.values())
        with open(path, "w") as f:
            json.dump(summary, f, indent=1, default=str)
        print_result(summary, path)
    sys.exit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
