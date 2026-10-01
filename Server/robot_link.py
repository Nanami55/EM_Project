"""
robot_link.py
=============
Owns the serial connection to the Arduino over the paired HC-05 Bluetooth
link. This is the ONLY place in the whole codebase that talks to the
robot's MCU — vision_webapp_bridge.py calls into this, never touches
pyserial directly. That keeps the hardware boundary in one file.

Runs its own reconnecting reader thread. Every write goes through a lock
so the defect-stop path and the manual-drive path (both can fire from
different threads/coroutines) can't interleave bytes on the wire.

Wiring reminder: the HC-05 paired with this machine shows up as a serial
port — e.g. /dev/rfcomm0 on Linux, or a "Bluetooth Standard Serial" COM
port on Windows. Pair it once at the OS level first; this module just
opens whatever port you give it.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import serial

logger = logging.getLogger("robot_link")

# Command strings — must match the protocol comment at the top of
# arduino_rover.ino exactly.
_DRIVE_COMMANDS = {
    "forward": "D:F",
    "backward": "D:B",
    "left": "D:L",
    "right": "D:R",
    "stop": "D:S",
}


class RobotLink:
    PING_INTERVAL_SEC = 2.0
    PING_TIMEOUT_SEC = 3.0   # if no PONG within this long, treat as lost

    def __init__(self, port: str, baud: int = 9600):
        self.port = port
        self.baud = baud
        self._ser: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._telemetry = {"battery_v": None, "state": "UNKNOWN"}
        self._telemetry_lock = threading.Lock()
        self._connected = False

        # --- latency tracking (Bluetooth RTT) -----------------------------
        self._latency_lock = threading.Lock()
        self._latency_ms: Optional[float] = None
        self._ping_sent_at: Optional[float] = None

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._ping_loop, daemon=True).start()

    # ---- connection + reader thread ---------------------------------------
    def _run(self):
        while True:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                self._connected = True
                logger.info(f"Robot link up on {self.port}")
                self._read_loop()
            except serial.SerialException as e:
                self._connected = False
                logger.warning(f"Robot link down ({e}); retrying in 3s")
                time.sleep(3.0)

    def _read_loop(self):
        while True:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                self._connected = False
                return
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                self._parse_line(line)

    def _parse_line(self, line: str):
        if line.startswith("BATT:"):
            try:
                v = float(line.split(":", 1)[1])
                with self._telemetry_lock:
                    self._telemetry["battery_v"] = v
            except ValueError:
                pass
        elif line.startswith("STATE:"):
            with self._telemetry_lock:
                self._telemetry["state"] = line.split(":", 1)[1]
        elif line == "PONG":
            with self._latency_lock:
                if self._ping_sent_at is not None:
                    self._latency_ms = (time.time() - self._ping_sent_at) * 1000.0
                    self._ping_sent_at = None

    def get_telemetry(self) -> dict:
        with self._telemetry_lock:
            return dict(self._telemetry)

    # ---- Bluetooth RTT probe ------------------------------------------
    def _ping_loop(self):
        """Runs independently of everything else — a PING every
        PING_INTERVAL_SEC, one at a time. If a PONG doesn't come back
        within PING_TIMEOUT_SEC, latency is reported as None (i.e. the
        UI should show "no link" rather than a stale number)."""
        while True:
            time.sleep(self.PING_INTERVAL_SEC)
            if not self._connected:
                continue
            with self._latency_lock:
                self._ping_sent_at = time.time()
            self._send("PING")

            # give it up to PING_TIMEOUT_SEC to arrive; if the read loop
            # hasn't cleared _ping_sent_at by then, this ping was lost.
            time.sleep(self.PING_TIMEOUT_SEC)
            with self._latency_lock:
                if self._ping_sent_at is not None:
                    # no PONG arrived in time
                    self._latency_ms = None
                    self._ping_sent_at = None

    def get_latency_ms(self) -> Optional[float]:
        with self._latency_lock:
            return self._latency_ms

    # ---- low-level send -----------------------------------------------
    def _send(self, cmd: str):
        if not self._connected or self._ser is None:
            logger.warning(f"Dropped command '{cmd}' — robot link not connected")
            return
        with self._write_lock:
            try:
                self._ser.write((cmd + "\n").encode("utf-8"))
            except serial.SerialException:
                self._connected = False

    # ---- public command API --------------------------------------------
    def defect_stop(self):
        """Called by the vision pipeline the instant a new defect is logged.
        The Arduino owns the resume timing — this just triggers the stop."""
        self._send("STOP")

    def emergency_stop(self):
        """Dashboard E-Stop button. Latches until resume() is called."""
        self._send("ESTOP")

    def resume(self):
        self._send("RESUME")

    def set_mode(self, mode: str):
        mode = mode.upper()
        if mode not in ("AUTO", "MANUAL"):
            raise ValueError("mode must be AUTO or MANUAL")
        self._send(f"M:{mode}")

    def drive(self, direction: str):
        if direction not in _DRIVE_COMMANDS:
            raise ValueError(f"unknown direction '{direction}', expected one of {list(_DRIVE_COMMANDS)}")
        self._send(_DRIVE_COMMANDS[direction])

    def ping(self):
        self._send("PING")


_link_singleton: Optional[RobotLink] = None


def get_robot_link(port: Optional[str] = None, baud: int = 9600) -> RobotLink:
    global _link_singleton
    if _link_singleton is None:
        if port is None:
            raise RuntimeError("get_robot_link() needs a port on first call")
        _link_singleton = RobotLink(port, baud)
        _link_singleton.start()
    return _link_singleton
