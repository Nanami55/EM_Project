"""
esp32_display_link.py
======================
A small, dependency-free "link" to the TFT screen attached to your
ESP32-S3-CAM. Import ESP32DisplayLink and Stat into ANY of your own Python
scripts — your bot script, ugv_vision_system.py, a quick test script,
whatever — and call `.send(...)` whenever you want to change what's on
the robot's screen.

This talks to esp32s3cam_vision_display.ino over UDP using a tiny JSON
protocol. It does NOT touch the camera stream at all — that's a
completely separate UDP port/socket, so this module works whether or not
you're running any vision code.

QUICK TEST (no wiring into your bot needed):
    python esp32_display_link.py --ip 192.168.0.42 --demo

USE IN YOUR OWN SCRIPT:
    from esp32_display_link import ESP32DisplayLink, Stat

    display = ESP32DisplayLink("192.168.0.42")   # the ESP32's IP address
    display.send(
        title="MY BOT",
        status="Patrolling zone 2",
        status_level="good",
        stats=[
            Stat("BATTERY", "78%", "good"),
            Stat("DIST", "42cm", "warn"),
            Stat("MODE", "AUTO", "neutral"),
            Stat("GPS", "LOCKED", "good"),
        ],
    )

PROTOCOL NOTES (kept in sync with esp32s3cam_vision_display.ino):
 - Any field you leave as None is left UNCHANGED on the screen — you can
   call send(status="Rebooting...") on its own without wiping the stat grid.
 - "stats" always replaces the whole 2x2 grid when provided (up to 4 slots).
 - "value" is always sent as a plain string — format numbers yourself
   (e.g. f"{temp:.1f}C") so the ESP32 doesn't have to guess.
 - "level" is one of "good" / "warn" / "bad" / "neutral" and controls the
   color the ESP32 draws that text in.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass, field
from typing import List, Optional, Union

Level = str  # "good" | "warn" | "bad" | "neutral" — kept as str, not an enum, on purpose


@dataclass
class Stat:
    """One label/value card in the 2x2 grid on the TFT."""
    label: str
    value: Union[str, int, float]
    level: Level = "neutral"

    def __post_init__(self):
        # Labels are shown in all caps for a consistent dashboard look;
        # values are coerced to str because the ESP32 never formats numbers.
        self.label = str(self.label).upper()[:12]
        self.value = str(self.value)[:10]
        if self.level not in ("good", "warn", "bad", "neutral"):
            self.level = "neutral"

    def to_dict(self) -> dict:
        return {"label": self.label, "value": self.value, "level": self.level}


class ESP32DisplayLink:
    """
    Fire-and-forget UDP link to the ESP32's display listener.

    Uses UDP on purpose (same philosophy as the camera stream): if a status
    update gets lost, the next one arrives a fraction of a second later and
    nothing needs to stall waiting for a retransmit.
    """

    def __init__(self, esp32_ip: str, port: int = 5006, min_interval: float = 0.2):
        """
        esp32_ip:     IP address of your ESP32-S3-CAM (printed to Serial on boot).
        port:         must match DISPLAY_LISTEN_PORT in esp32s3cam_vision_display.ino.
        min_interval: minimum seconds between sends. Prevents flooding the ESP32
                       (and the WiFi link) if you call send() every video frame.
                       Set to 0 to disable throttling.
        """
        self.addr = (esp32_ip, port)
        self.min_interval = min_interval
        self._last_sent = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(
        self,
        title: Optional[str] = None,
        status: Optional[str] = None,
        status_level: Level = "neutral",
        stats: Optional[List[Stat]] = None,
        force: bool = False,
    ) -> bool:
        """
        Push an update to the screen. Returns True if a packet was actually
        sent, False if it was throttled or the send failed.

        Only include the fields you want to CHANGE — anything left as None
        stays as whatever was last shown.
        """
        now = time.time()
        if not force and self.min_interval > 0 and (now - self._last_sent) < self.min_interval:
            return False

        payload: dict = {}
        if title is not None:
            payload["title"] = str(title)[:20]
        if status is not None:
            payload["status"] = str(status)[:46]
            payload["status_level"] = status_level if status_level in ("good", "warn", "bad", "neutral") else "neutral"
        if stats is not None:
            payload["stats"] = [s.to_dict() if isinstance(s, Stat) else s for s in stats[:4]]

        if not payload:
            return False  # nothing to send

        data = json.dumps(payload).encode("utf-8")
        try:
            self._sock.sendto(data, self.addr)
        except OSError as exc:
            print(f"[ESP32DisplayLink] send failed: {exc}")
            return False

        self._last_sent = now
        return True

    def close(self):
        self._sock.close()

    # Convenience for `with ESP32DisplayLink(...) as display:` usage
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ---------------------------------------------------------------------------
# Demo / self-test — run this file directly to sanity-check the display
# BEFORE wiring it into your bot script.
# ---------------------------------------------------------------------------
def _demo_loop(ip: str, port: int):
    levels = ["good", "warn", "bad", "neutral"]
    print(f"Sending demo data to {ip}:{port} — press Ctrl+C to stop.")
    print("You should see the title, status color, and 4 stat cards change every second.")

    with ESP32DisplayLink(ip, port, min_interval=0.0) as display:
        i = 0
        try:
            while True:
                i += 1
                ok = display.send(
                    title="DEMO BOT",
                    status=f"Demo tick #{i}",
                    status_level=levels[i % 4],
                    stats=[
                        Stat("FPS", f"{18 + (i % 6)}.0", "good"),
                        Stat("BATTERY", f"{max(0, 100 - i)}%", "warn" if i > 50 else "good"),
                        Stat("OBJECTS", str(i % 6), "neutral"),
                        Stat("MODE", "TEST", "neutral"),
                    ],
                    force=True,
                )
                print(f"  tick {i}: {'sent' if ok else 'FAILED'}")
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopped.")


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3-CAM display link")
    parser.add_argument("--ip", required=True, help="IP address of your ESP32-S3-CAM")
    parser.add_argument("--port", type=int, default=5006, help="Display UDP port (default 5006)")
    parser.add_argument("--demo", action="store_true", help="Send looping fake test data")
    args = parser.parse_args()

    if args.demo:
        _demo_loop(args.ip, args.port)
    else:
        print("Nothing to do — pass --demo to send test data, or import this module")
        print("into your own script and call ESP32DisplayLink(...).send(...) directly.")


if __name__ == "__main__":
    main()
