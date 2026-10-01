"""
Sensor_link.py
================
Thin, fault-tolerant client for the ESP8266 sensor + pan/tilt node
(see esp8266_sensor_pantilt.ino). Talks HTTP/JSON over WiFi.

Mirrors the same pattern as robot_link.py / cam_link.py:
    sensor_link = get_sensor_link("192.168.1.50")
    data = sensor_link.get_sensors()
    sensor_link.set_pan(45)
    sensor_link.set_tilt(120)

FALLBACK BEHAVIOR
------------------
Built to never take down the caller, whether that's a one-off test run
or the vision_webapp_bridge main loop:
  - If the ESP8266 is unreachable entirely (wrong IP, not powered on,
    not on the WiFi yet), get_sensors() returns the last-known reading
    with "connected": False, rather than raising or blocking.
  - If the ESP is reachable but a specific sensor failed to init on its
    end (e.g. no MPU6050 wired up), that sensor's block in the JSON
    already reads {"connected": false} from the firmware itself — this
    class just passes it through untouched.
  - Network hiccups are retried quietly on a background thread. A
    warning is logged once per state change (connected -> lost, and
    lost -> reconnected), not spammed on every poll.

RUN DIRECTLY to pull data from the command line:
    python Sensor_link.py 192.168.1.50
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import requests

logger = logging.getLogger("sensor_link")

_DEFAULT_TIMEOUT_S = 1.5

_DISCONNECTED_READING = {
    "connected": False,
    "dht22": {"connected": False, "temperature_c": None, "humidity_pct": None},
    "hall": {"state": None},
    "mpu6050": {"connected": False, "accel": None, "gyro": None, "temp_c": None},
    "servo": {"pan_deg": None, "tilt_deg": None},
    "uptime_ms": None,
}


class SensorLink:
    def __init__(self, host: str, port: int = 80, poll_hz: float = 2.0):
        self.base_url = f"http://{host}:{port}"
        self.poll_interval = 1.0 / poll_hz if poll_hz > 0 else 0.5

        self._lock = threading.Lock()
        self._latest: dict = dict(_DISCONNECTED_READING)
        self._latency_ms: Optional[float] = None
        self._was_connected = False

        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ---- lifecycle ---------------------------------------------------------
    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_evt.set()

    # ---- public reads -------------------------------------------------------
    def get_sensors(self) -> dict:
        """Always returns a dict — even if the ESP has never responded."""
        with self._lock:
            return dict(self._latest)

    def get_latency_ms(self) -> Optional[float]:
        with self._lock:
            return self._latency_ms

    # ---- public commands -----------------------------------------------------
    def set_pan(self, angle: int) -> bool:
        return self._send_angle("pan", angle)

    def set_tilt(self, angle: int) -> bool:
        return self._send_angle("tilt", angle)

    def _send_angle(self, axis: str, angle: int) -> bool:
        angle = max(0, min(180, int(angle)))
        try:
            r = requests.get(f"{self.base_url}/{axis}",
                              params={"angle": angle}, timeout=_DEFAULT_TIMEOUT_S)
            r.raise_for_status()
            return bool(r.json().get("ok", False))
        except (requests.RequestException, ValueError) as e:
            logger.warning(f"Failed to set {axis} to {angle}: {e}")
            return False

    # ---- background polling ---------------------------------------------------
    def _poll_loop(self):
        while not self._stop_evt.is_set():
            start = time.time()
            try:
                r = requests.get(f"{self.base_url}/data", timeout=_DEFAULT_TIMEOUT_S)
                r.raise_for_status()
                data = r.json()
                data["connected"] = True
                with self._lock:
                    self._latest = data
                    self._latency_ms = (time.time() - start) * 1000.0
                if not self._was_connected:
                    logger.info(f"Sensor node connected at {self.base_url}")
                    self._was_connected = True
            except (requests.RequestException, ValueError) as e:
                if self._was_connected:
                    logger.warning(f"Sensor node lost ({self.base_url}): {e}")
                    self._was_connected = False
                with self._lock:
                    self._latest = dict(self._latest, connected=False)
                    self._latency_ms = None
            elapsed = time.time() - start
            time.sleep(max(0.0, self.poll_interval - elapsed))


# ---- singleton, matching get_robot_link()/get_cam_link() ---------------------
_instance: Optional[SensorLink] = None
_instance_lock = threading.Lock()


def get_sensor_link(host: Optional[str], port: int = 80, poll_hz: float = 2.0) -> Optional[SensorLink]:
    """Returns None if no host is given — mirrors get_cam_link()'s behavior
    for an optional/unconfigured link, so callers can do:

        self.sensor_link = get_sensor_link(args.sensor_host)
        ...
        data = self.sensor_link.get_sensors() if self.sensor_link else None
    """
    global _instance
    if not host:
        return None
    with _instance_lock:
        if _instance is None:
            _instance = SensorLink(host, port, poll_hz)
            _instance.start()
        return _instance


if __name__ == "__main__":
    import argparse
    import json as _json

    parser = argparse.ArgumentParser(description="Manual test: pull data from the ESP8266 sensor node")
    parser.add_argument("host", help="ESP8266 IP address, e.g. 192.168.1.50")
    parser.add_argument("--port", type=int, default=80)
    args = parser.parse_args()

    link = get_sensor_link(args.host, args.port)
    print(f"Polling {args.host}:{args.port} — Ctrl+C to stop")
    try:
        while True:
            print(_json.dumps(link.get_sensors(), indent=2))
            time.sleep(1.0)
    except KeyboardInterrupt:
        link.stop()
