"""Runs the FlexiHAL simulator as a child process for the scripted evidence runs.

The simulator's stdin is a pipe, so the script can press its console keys (feed
hold, alarms and so on). kill() sends SIGKILL, with no flush and no shutdown,
which is what a power cut or a dead controller looks like.

sd=True starts it with no FRAM fitted (-F) and an SD card image in the run
directory (-d card.img), so the recorder uses the SD card. env adds to the
environment, for the card model's SIM_SD_* knobs.
"""

import ctypes
import os
import signal
import subprocess
import sys
import time


def _die_with_parent():
    # On Linux, if the script gets killed (by the run cap, say) the simulator goes with it.
    if sys.platform.startswith("linux"):
        ctypes.CDLL("libc.so.6").prctl(1, signal.SIGKILL)  # PR_SET_PDEATHSIG


class Sim:
    def __init__(self, binary, run_dir, ws_port=8090, step_report=0.01, speed=1.0, sd=False, env=None):
        self.binary = binary
        self.run_dir = run_dir
        self.ws_port = ws_port
        self.step_report = step_report
        self.speed = speed
        self.sd = sd
        self.env = dict(env or {})
        self.proc = None
        self.boots = 0
        os.makedirs(run_dir, exist_ok=True)

    @property
    def card(self):
        return os.path.join(self.run_dir, "card.img")

    @property
    def usb(self):
        return "usb:" + os.path.join(self.run_dir, "usb")

    @property
    def ws(self):
        return f"ws://localhost:{self.ws_port}"

    def start(self):
        self.boots += 1
        tag = f"boot{self.boots}"
        command = [self.binary,
                   "-u", os.path.join(self.run_dir, "usb"),
                   "-w", str(self.ws_port),
                   "-e", os.path.join(self.run_dir, "EEPROM.DAT"),
                   "-r", str(self.step_report),
                   "-t", str(self.speed)]
        if self.sd:
            command += ["-F", "-d", self.card]
        self.out = open(os.path.join(self.run_dir, f"{tag}.sim.log"), "wb")
        self.steps = open(os.path.join(self.run_dir, f"{tag}.steps.log"), "wb")
        env = dict(os.environ)
        env.update({key: str(value) for key, value in self.env.items()})
        self.proc = subprocess.Popen(command, cwd=self.run_dir, stdin=subprocess.PIPE,
                                     stdout=self.out, stderr=self.steps, env=env,
                                     preexec_fn=_die_with_parent)
        # Wait for the USB port's symlink to appear, then give grblHAL a moment to boot.
        port = os.path.join(self.run_dir, "usb")
        deadline = time.monotonic() + 5
        while not os.path.exists(port) and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.5)
        return self

    def key(self, c):
        self.proc.stdin.write(c.encode())
        self.proc.stdin.flush()

    def kill(self):
        """Power cut. SIGKILL, so nothing gets flushed."""
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGKILL)
            self.proc.wait()
        self._close_logs()

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._close_logs()

    def _close_logs(self):
        for log in (getattr(self, "out", None), getattr(self, "steps", None)):
            if log and not log.closed:
                log.close()
