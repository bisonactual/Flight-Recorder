"""Run the FlexiHAL simulator as a child process for scripted evidence runs.

The simulator's stdin is a pipe, so its console keys (feed hold, alarms, ...)
can be pressed from the script. kill() is SIGKILL: no flush, no shutdown,
which is what a power cut or a dead controller looks like.
"""

import ctypes
import os
import signal
import subprocess
import sys
import time


def _die_with_parent():
    # Linux: if the script is killed (e.g. by the run cap), take the simulator with it.
    if sys.platform.startswith("linux"):
        ctypes.CDLL("libc.so.6").prctl(1, signal.SIGKILL)  # PR_SET_PDEATHSIG


class Sim:
    def __init__(self, binary, run_dir, ws_port=8090, step_report=0.01, speed=1.0):
        self.binary = binary
        self.run_dir = run_dir
        self.ws_port = ws_port
        self.step_report = step_report
        self.speed = speed
        self.proc = None
        self.boots = 0
        os.makedirs(run_dir, exist_ok=True)

    @property
    def usb(self):
        return "usb:" + os.path.join(self.run_dir, "usb")

    @property
    def ws(self):
        return f"ws://localhost:{self.ws_port}"

    def start(self):
        self.boots += 1
        tag = f"boot{self.boots}"
        cmd = [self.binary,
               "-u", os.path.join(self.run_dir, "usb"),
               "-w", str(self.ws_port),
               "-e", os.path.join(self.run_dir, "EEPROM.DAT"),
               "-r", str(self.step_report),
               "-t", str(self.speed)]
        self.out = open(os.path.join(self.run_dir, f"{tag}.sim.log"), "wb")
        self.steps = open(os.path.join(self.run_dir, f"{tag}.steps.log"), "wb")
        self.proc = subprocess.Popen(cmd, cwd=self.run_dir, stdin=subprocess.PIPE,
                                     stdout=self.out, stderr=self.steps,
                                     preexec_fn=_die_with_parent)
        # Wait for the USB port symlink, then give grblHAL a moment to boot.
        link = os.path.join(self.run_dir, "usb")
        deadline = time.monotonic() + 5
        while not os.path.exists(link) and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.5)
        return self

    def key(self, c):
        self.proc.stdin.write(c.encode())
        self.proc.stdin.flush()

    def kill(self):
        """Power cut: SIGKILL, nothing flushed."""
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
        for f in (getattr(self, "out", None), getattr(self, "steps", None)):
            if f and not f.closed:
                f.close()
