"""
cam_link.py
===========
Pings the ESP32-CAM's lightweight /ping endpoint on its own thread to
measure real WiFi round-trip latency — deliberately separate from the
(much heavier, much noisier) video stream itself, so a busy frame doesn't
skew the number.

Only meaningful when the camera source is an HTTP URL (i.e. the real
ESP32-CAM, not a local USB webcam index) — vision_webapp_bridge.py only
starts this when that's the case.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional
from urllib.parse import urlsplit

import requests

logger = logging.getLogger("cam_link")


class CamLink:
    def __init__(self, stream_url: str, interval_sec: float = 2.0, timeout_sec: float = 1.5):
        parts = urlsplit(stream_url)
        self.ping_url = f"{parts.scheme}://{parts.netloc}/ping"
        self.interval_sec = interval_sec
        self.timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._latency_ms: Optional[float] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            t0 = time.time()
            try:
                r = requests.get(self.ping_url, timeout=self.timeout_sec)
                if r.status_code == 200:
                    latency = (time.time() - t0) * 1000.0
                    with self._lock:
                        self._latency_ms = latency
                        self._connected = True
                else:
                    with self._lock:
                        self._connected = False
                        self._latency_ms = None
            except requests.RequestException:
                with self._lock:
                    self._connected = False
                    self._latency_ms = None
            time.sleep(self.interval_sec)

    def get_latency_ms(self) -> Optional[float]:
        with self._lock:
            return self._latency_ms


_cam_link_singleton: Optional[CamLink] = None


def get_cam_link(stream_url: Optional[str] = None) -> Optional[CamLink]:
    """Returns None (rather than raising) when stream_url isn't an HTTP
    URL — e.g. a local webcam index — since there's nothing to ping."""
    global _cam_link_singleton
    if _cam_link_singleton is None:
        if not stream_url or not str(stream_url).startswith("http"):
            return None
        _cam_link_singleton = CamLink(stream_url)
        _cam_link_singleton.start()
    return _cam_link_singleton
