"""
ws_client.py
============
Background-thread WebSocket client used by app.py (Streamlit) to talk to
vision_webapp_bridge.py without ever blocking the Streamlit rerun loop.

app.py only ever calls:
    bridge = get_bridge()
    bridge.start()
    bridge.drain()                -> list[dict], messages received since last call
    bridge.send_command(payload)  -> queue a dict to be sent as JSON
    bridge.connected              -> bool

Everything else (reconnect-on-drop, event loop management) is internal.
"""

from __future__ import annotations

import asyncio
import json
import queue
import threading

import websockets

WS_URL = "ws://localhost:8765"


class RobotBridge:
    def __init__(self, url: str = WS_URL):
        self.url = url
        self.connected = False
        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self._outbox: "queue.Queue[dict]" = queue.Queue()
        self._started = False

    def start(self):
        if self._started:
            return
        self._started = True
        threading.Thread(target=self._run, daemon=True).start()

    def drain(self) -> list:
        msgs = []
        while True:
            try:
                msgs.append(self._inbox.get_nowait())
            except queue.Empty:
                break
        return msgs

    def send_command(self, payload: dict):
        self._outbox.put_nowait(payload)

    # ---- internal -----------------------------------------------------
    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._main())

    async def _main(self):
        while True:
            try:
                async with websockets.connect(self.url) as ws:
                    self.connected = True
                    await asyncio.gather(self._recv(ws), self._send(ws))
            except Exception:
                self.connected = False
                await asyncio.sleep(1.0)

    async def _recv(self, ws):
        async for raw in ws:
            try:
                self._inbox.put_nowait(json.loads(raw))
            except json.JSONDecodeError:
                continue

    async def _send(self, ws):
        while True:
            try:
                payload = self._outbox.get_nowait()
                await ws.send(json.dumps(payload))
            except queue.Empty:
                await asyncio.sleep(0.05)


_bridge_singleton: RobotBridge | None = None


def get_bridge() -> RobotBridge:
    global _bridge_singleton
    if _bridge_singleton is None:
        _bridge_singleton = RobotBridge()
    return _bridge_singleton
