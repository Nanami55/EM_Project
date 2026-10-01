"""
vision_webapp_bridge.py
========================
Wraps YOUR `ugv_vision_system.py` so it can feed the Streamlit
dashboard, AND drives the Arduino over HC-05 via robot_link.py.

Includes:
  - Dataset collection support (--collect, dataset_collector.py)
  - Distance tracking from hall-effect sensor via SensorLink
  - obstacle class filtering during YOLO inference
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import queue
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

import cv2
from flask import Flask, Response, request
import websockets

from ugv_vision_system import (
    UGVVisionSystem,
    VisionConfig,
    logger,
)

from robot_link import get_robot_link
from cam_link import get_cam_link
from Sensor_link import get_sensor_link
from esp32_display_link import ESP32DisplayLink, Stat


# ---------------------------------------------------------------------------
# Severity classification
# ---------------------------------------------------------------------------
DEFAULT_SEVERITY_BY_CLASS = {
    # "cracked_rail": "Critical",
    # "missing_bolt": "Critical",
    # "worn_fastener": "Moderate",
    # "misalignment": "Moderate",
}


def classify_severity(label: str, confidence: float) -> str:
    if label in DEFAULT_SEVERITY_BY_CLASS:
        return DEFAULT_SEVERITY_BY_CLASS[label]

    # NOTE:
    # These values can be customized later according to your actual model.
    if confidence >= 0.80:
        return "Critical"

    if confidence >= 0.50:
        return "Moderate"

    return "Minor"


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------
class TelemetrySource:

    def __init__(self, robot_link):
        self.robot_link = robot_link

    def read_battery_voltage(self) -> float:
        v = self.robot_link.get_telemetry().get("battery_v")
        return v if v is not None else 0.0

    def read_position(self) -> dict:
        return {
            "x": 0.0,
            "y": 0.0
        }


# ---------------------------------------------------------------------------
# Distance Tracker
# ---------------------------------------------------------------------------
class DistanceTracker:
    """Converts hall-effect magnet passes into cumulative distance traveled."""

    def __init__(
        self,
        wheel_radius_cm: float,
        magnets_per_rev: int = 8
    ):

        wheel_circumference_m = (
            2 * math.pi * (wheel_radius_cm / 100.0)
        )

        self.distance_per_pulse_m = (
            wheel_circumference_m / magnets_per_rev
        )

        self._pulse_count = 0
        self._last_state: Optional[int] = None
        self._lock = threading.Lock()

    def update_from_state(self, hall_state: Optional[int]):

        if hall_state is None:
            return

        with self._lock:

            if self._last_state == 1 and hall_state == 0:
                self._pulse_count += 1

            self._last_state = hall_state

    def update_from_count(self, cumulative_count: int):

        with self._lock:
            self._pulse_count = cumulative_count

    def get_distance_m(self) -> float:

        with self._lock:
            return self._pulse_count * self.distance_per_pulse_m

    def get_pulse_count(self) -> int:

        with self._lock:
            return self._pulse_count


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
@dataclass
class WebBridgeConfig(VisionConfig):

    ws_host: str = "0.0.0.0"
    ws_port: int = 8765

    http_host: str = "0.0.0.0"
    http_port: int = 5001

    telemetry_push_hz: float = 2.0
    defect_debounce_sec: float = 8.0

    jpeg_quality: int = 80

    robot_port: str = "/dev/rfcomm0"
    robot_baud: int = 9600

    sensor_host: Optional[str] = None
    sensor_poll_hz: float = 10.0

    wheel_radius_cm: float = 5.0
    magnets_per_rev: int = 8

    display_ip: Optional[str] = None
    display_port: int = 5006
    display_min_interval: float = 0.2

    # -------------------------------------------------------
    # Classes which YOLO should completely ignore.
    #
    # obstacle is excluded by default.
    # Case insensitive:
    # -------------------------------------------------------
    exclude_classes: list[str] = field(
        default_factory=lambda: ["obstacle"]
    )


# ---------------------------------------------------------------------------
# Shared Frame Hand-off
# ---------------------------------------------------------------------------
class SharedFrameState:

    def __init__(self):

        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None

    def update(self, jpeg_bytes: Optional[bytes]):

        with self._lock:
            self._jpeg = jpeg_bytes

    def snapshot(self) -> Optional[bytes]:

        with self._lock:
            return self._jpeg


# ---------------------------------------------------------------------------
# Main Web Bridge
# ---------------------------------------------------------------------------
class WebDashboardVisionSystem(UGVVisionSystem):

    def __init__(self, config: WebBridgeConfig):

        super().__init__(config)

        self.config: WebBridgeConfig = config

        self.shared = SharedFrameState()

        self.robot_link = get_robot_link(
            config.robot_port,
            config.robot_baud
        )

        self.sensor_link = get_sensor_link(
            config.sensor_host,
            poll_hz=config.sensor_poll_hz
        )

        self.distance_tracker = DistanceTracker(
            config.wheel_radius_cm,
            config.magnets_per_rev
        )

        cam_source_for_link = (
            config.camera_source
            if config.transport == "tcp"
            and isinstance(config.camera_source, str)
            else None
        )

        self.cam_link = get_cam_link(cam_source_for_link)

        self.display_link = (
            ESP32DisplayLink(
                config.display_ip,
                config.display_port,
                min_interval=config.display_min_interval
            )
            if config.display_ip
            else None
        )

        self.telemetry_source = TelemetrySource(
            self.robot_link
        )

        self.ws_clients: set = set()

        self.outbound: "queue.Queue[dict]" = queue.Queue()

        self.stop_requested = False

        self._last_logged_at: dict[str, float] = {}

        self._defect_seq = 0

        # Print model classes for debugging
        logger.info(
            f"YOLO model classes: {self.model.names}"
        )

        logger.info(
            f"Excluded classes: {self.config.exclude_classes}"
        )

    # -----------------------------------------------------------------------
    # CLASS FILTER
    #
    # THIS IS THE IMPORTANT FIX.
    #
    # Only sends allowed class IDs to YOLO.
    # "normal" never reaches inference output.
    # -----------------------------------------------------------------------
    def _resolve_class_filter(self):

        names = self.model.names

        excluded = {
            str(name).strip().lower()
            for name in self.config.exclude_classes
        }

        # If user specified --classes,
        # only those classes are permitted.
        requested = None

        if self.config.target_classes:

            requested = {
                str(name).strip().lower()
                for name in self.config.target_classes
            }

        allowed_ids = []

        for cls_id, cls_name in names.items():

            normalized_name = (
                str(cls_name)
                .strip()
                .lower()
            )

            # Skip NORMAL
            if normalized_name in excluded:

                logger.debug(
                    f"Ignoring YOLO class: "
                    f"{cls_name} (ID {cls_id})"
                )

                continue

            # If --classes was provided,
            # only allow requested classes
            if (
                requested is not None
                and normalized_name not in requested
            ):
                continue

            allowed_ids.append(int(cls_id))

        logger.info(
            "YOLO enabled class IDs: "
            f"{allowed_ids}"
        )

        logger.info(
            "YOLO enabled classes: "
            f"{[names[i] for i in allowed_ids]}"
        )

        return allowed_ids

    # -----------------------------------------------------------------------
    # Convert YOLO boxes to dashboard boxes
    # -----------------------------------------------------------------------
    def _boxes_from_results(self, results) -> list[dict]:

        r = results[0]

        h, w = r.orig_shape

        out = []

        for box in r.boxes:

            x0, y0, x1, y1 = (
                box.xyxy[0].tolist()
            )

            conf = float(box.conf[0])

            cls_id = int(box.cls[0])

            label = self.model.names.get(
                cls_id,
                str(cls_id)
            )

            # Extra safety filter
            if (
                str(label).strip().lower()
                in {
                    str(x).strip().lower()
                    for x in self.config.exclude_classes
                }
            ):
                continue

            out.append({
                "label": label,

                "confidence": round(
                    conf,
                    3
                ),

                "bbox": [
                    x0 / w,
                    y0 / h,
                    x1 / w,
                    y1 / h,
                ],
            })

        return out

    # -----------------------------------------------------------------------
    # Snapshot
    # -----------------------------------------------------------------------
    def _capture_snapshot_b64(
        self,
        frame,
        bbox_norm,
        pad_frac: float = 0.30
    ) -> Optional[str]:

        try:

            h, w = frame.shape[:2]

            x0, y0, x1, y1 = bbox_norm

            bw = x1 - x0
            bh = y1 - y0

            x0 = max(
                0.0,
                x0 - bw * pad_frac
            )

            y0 = max(
                0.0,
                y0 - bh * pad_frac
            )

            x1 = min(
                1.0,
                x1 + bw * pad_frac
            )

            y1 = min(
                1.0,
                y1 + bh * pad_frac
            )

            px0 = int(x0 * w)
            py0 = int(y0 * h)
            px1 = int(x1 * w)
            py1 = int(y1 * h)

            crop = frame[
                py0:py1,
                px0:px1
            ]

            if crop.size == 0:
                crop = frame

            ok, buf = cv2.imencode(
                ".jpg",
                crop,
                [
                    int(
                        cv2.IMWRITE_JPEG_QUALITY
                    ),
                    self.config.jpeg_quality
                ]
            )

            if not ok:
                return None

            return base64.b64encode(
                buf.tobytes()
            ).decode("ascii")

        except Exception:

            logger.exception(
                "Failed to capture defect snapshot"
            )

            return None

    # -----------------------------------------------------------------------
    # Defect logging
    # -----------------------------------------------------------------------
    def _maybe_log_defects(
        self,
        boxes_out: list[dict],
        position: dict,
        frame
    ) -> list[dict]:

        now = time.time()

        new_detections = []

        excluded = {
            str(x).strip().lower()
            for x in self.config.exclude_classes
        }

        for b in boxes_out:

            label_normalized = (
                str(b["label"])
                .strip()
                .lower()
            )

            # Extra safety check
            if label_normalized in excluded:
                continue

            last = self._last_logged_at.get(
                label_normalized,
                0.0
            )

            if (
                now - last
                < self.config.defect_debounce_sec
            ):
                continue

            self._last_logged_at[
                label_normalized
            ] = now

            self._defect_seq += 1

            new_detections.append({

                "id":
                    f"D-{self._defect_seq:04d}",

                "label":
                    b["label"],

                "confidence":
                    b["confidence"],

                "severity":
                    classify_severity(
                        b["label"],
                        b["confidence"]
                    ),

                "bbox":
                    b["bbox"],

                "timestamp":
                    datetime.now(
                        timezone.utc
                    ).isoformat(),

                "gps":
                    position,

                "distance_m":
                    round(
                        self.distance_tracker
                        .get_distance_m(),
                        2
                    ),

                "snapshot_b64":
                    self._capture_snapshot_b64(
                        frame,
                        b["bbox"]
                    ),
            })

        return new_detections

    # -----------------------------------------------------------------------
    # Sensor summary
    # -----------------------------------------------------------------------
    def _sensor_summary(self) -> dict:

        raw = (
            self.sensor_link.get_sensors()
            if self.sensor_link
            else None
        )

        dht = (
            (raw or {})
            .get("dht22", {})
        )

        return {

            "connected":
                bool(raw.get("connected"))
                if raw
                else False,

            "temperature_c":
                dht.get(
                    "temperature_c"
                ),

            "humidity_pct":
                dht.get(
                    "humidity_pct"
                ),

            "distance_m":
                round(
                    self.distance_tracker
                    .get_distance_m(),
                    2
                ),

            "pulse_count":
                self.distance_tracker
                .get_pulse_count(),
        }

    # -----------------------------------------------------------------------
    # Robot status
    # -----------------------------------------------------------------------
    def _status_level(
        self,
        status: str
    ) -> str:

        s = (
            status or ""
        ).upper()

        if any(
            k in s
            for k in (
                "STOP",
                "ESTOP",
                "FAULT",
                "ERROR"
            )
        ):
            return "bad"

        if any(
            k in s
            for k in (
                "MANUAL",
                "PAUSE",
                "IDLE"
            )
        ):
            return "warn"

        if any(
            k in s
            for k in (
                "AUTO",
                "PATROL",
                "RUN"
            )
        ):
            return "good"

        return "neutral"

    # -----------------------------------------------------------------------
    # TFT defect status
    # -----------------------------------------------------------------------
    def _defect_stat(
        self,
        live_boxes: list[dict]
    ) -> Stat:

        excluded = {
            str(x).strip().lower()
            for x in self.config.exclude_classes
        }

        defects = [
            b
            for b in live_boxes
            if (
                str(b["label"])
                .strip()
                .lower()
                not in excluded
            )
        ]

        if not defects:

            return Stat(
                "DEFECT",
                "NONE",
                "good"
            )

        worst = max(
            defects,
            key=lambda b: b["confidence"]
        )

        severity = classify_severity(
            worst["label"],
            worst["confidence"]
        )

        level = (
            "bad"
            if severity == "Critical"
            else "warn"
        )

        return Stat(
            "DEFECT",
            worst["label"],
            level
        )

    # -----------------------------------------------------------------------
    # TFT update
    # -----------------------------------------------------------------------
    def _push_display(
        self,
        status: str,
        sensors: dict,
        live_boxes: list[dict]
    ):

        if not self.display_link:
            return

        temp = sensors.get(
            "temperature_c"
        )

        hum = sensors.get(
            "humidity_pct"
        )

        dist = sensors.get(
            "distance_m"
        )

        self.display_link.send(

            status=status,

            status_level=self._status_level(
                status
            ),

            stats=[

                Stat(
                    "TEMP",
                    f"{temp:.1f}C"
                    if temp is not None
                    else "--",
                    "neutral"
                ),

                Stat(
                    "HUM",
                    f"{hum:.0f}%"
                    if hum is not None
                    else "--",
                    "neutral"
                ),

                Stat(
                    "DIST",
                    f"{dist:.1f}m"
                    if dist is not None
                    else "--",
                    "neutral"
                ),

                self._defect_stat(
                    live_boxes
                ),
            ],
        )

    # -----------------------------------------------------------------------
    # Dashboard telemetry
    # -----------------------------------------------------------------------
    def _broadcast_telemetry(
        self,
        status: str,
        position: dict,
        live_boxes: list[dict],
        detections: list[dict]
    ):

        cam_latency_ms = (
            self.cam_link.get_latency_ms()
            if self.cam_link
            else None
        )

        robot_latency_ms = (
            self.robot_link.get_latency_ms()
        )

        known = [
            v
            for v in (
                cam_latency_ms,
                robot_latency_ms
            )
            if v is not None
        ]

        combined_latency_ms = (
            sum(known)
            if known
            else None
        )

        sensors = self._sensor_summary()

        msg = {

            "type":
                "telemetry",

            "ts":
                datetime.now(
                    timezone.utc
                ).isoformat(),

            "battery_v":
                round(
                    self.telemetry_source
                    .read_battery_voltage(),
                    2
                ),

            "cam_latency_ms":
                round(
                    cam_latency_ms,
                    1
                )
                if cam_latency_ms is not None
                else None,

            "robot_latency_ms":
                round(
                    robot_latency_ms,
                    1
                )
                if robot_latency_ms is not None
                else None,

            "combined_latency_ms":
                round(
                    combined_latency_ms,
                    1
                )
                if combined_latency_ms is not None
                else None,

            "status":
                status,

            "position":
                position,

            "live_boxes":
                live_boxes,

            "detections":
                detections,

            "sensors":
                sensors,
        }

        self.outbound.put_nowait(
            msg
        )

        self._push_display(
            status,
            sensors,
            live_boxes
        )

    # -----------------------------------------------------------------------
    # MAIN LOOP
    # -----------------------------------------------------------------------
    def run(self):

        self.grabber.start()

        time.sleep(1.0)

        threading.Thread(
            target=self._run_ws_server,
            daemon=True
        ).start()

        threading.Thread(
            target=self._run_http_server,
            daemon=True
        ).start()

        if self.display_link:

            self.display_link.send(
                title="RAIL UGV",
                status="Booting",
                status_level="neutral",
                force=True
            )

        logger.info(
            f"Headless inference loop starting — "
            f"telemetry on "
            f"ws://{self.config.ws_host}:{self.config.ws_port}, "
            f"video on "
            f"http://{self.config.http_host}:{self.config.http_port}/video_feed, "
            f"robot link on "
            f"{self.config.robot_port}"
        )

        fps_smoothed = 0.0

        alpha = 0.1

        prev_time = time.time()

        empty_streak = 0

        pending_detections: list[dict] = []

        last_push = 0.0

        # Resolve class filter once.
        #
        # Example:
        #
        # model.classes:
        # 0 = crack
        # 1 = normal
        # 2 = missing_bolt
        #
        # result:
        # [0, 2]
        #
        allowed_class_ids = (
            self._resolve_class_filter()
        )

        try:

            while not self.stop_requested:

                frame = (
                    self.grabber.read()
                )

                if frame is None:

                    empty_streak += 1

                    if (
                        empty_streak
                        >= self.config.max_consecutive_failures
                    ):

                        logger.error(
                            "No frames received for too long. Exiting."
                        )

                        break

                    continue

                empty_streak = 0

                # -----------------------------------------------------------
                # Hall sensor
                # -----------------------------------------------------------
                if self.sensor_link:

                    sensor_data = (
                        self.sensor_link
                        .get_sensors()
                    )

                    hall_state = (
                        sensor_data
                        .get("hall", {})
                        .get("state")
                    )

                    self.distance_tracker.update_from_state(
                        hall_state
                    )

                # -----------------------------------------------------------
                # YOLO INFERENCE
                #
                # NORMAL class is excluded HERE.
                # -----------------------------------------------------------
                results = self.model.predict(

                    frame,

                    device=self.device,

                    conf=self.config.confidence_threshold,

                    classes=allowed_class_ids,

                    verbose=False,
                )

                # -----------------------------------------------------------
                # Dataset collector
                # -----------------------------------------------------------
                if (
                    getattr(
                        self,
                        "collector",
                        None
                    )
                    is not None
                ):

                    self.collector.maybe_save(
                        frame,
                        results,
                        self.model.names
                    )

                # -----------------------------------------------------------
                # Plot only allowed detections
                # -----------------------------------------------------------
                annotated = (
                    results[0].plot()
                )

                # FPS
                now = time.time()

                instant_fps = (
                    1.0
                    / max(
                        now - prev_time,
                        1e-6
                    )
                )

                fps_smoothed = (
                    alpha * instant_fps
                    + (
                        1 - alpha
                    ) * fps_smoothed
                )

                prev_time = now

                cv2.putText(
                    annotated,
                    f"FPS: {fps_smoothed:.1f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                # -----------------------------------------------------------
                # JPEG stream
                # -----------------------------------------------------------
                ok, jpg = cv2.imencode(

                    ".jpg",

                    annotated,

                    [
                        int(
                            cv2.IMWRITE_JPEG_QUALITY
                        ),
                        self.config.jpeg_quality
                    ]
                )

                self.shared.update(
                    jpg.tobytes()
                    if ok
                    else None
                )

                # -----------------------------------------------------------
                # Dashboard detections
                # -----------------------------------------------------------
                boxes_out = (
                    self._boxes_from_results(
                        results
                    )
                )

                position = (
                    self.telemetry_source
                    .read_position()
                )

                new_defects = (
                    self._maybe_log_defects(
                        boxes_out,
                        position,
                        annotated
                    )
                )

                # -----------------------------------------------------------
                # STOP robot if actual defect detected
                # -----------------------------------------------------------
                if new_defects:

                    logger.warning(
                        f"New defect(s) logged: "
                        f"{[d['id'] for d in new_defects]} "
                        f"— sending STOP"
                    )

                    self.robot_link.defect_stop()

                pending_detections.extend(
                    new_defects
                )

                status = (
                    self.robot_link
                    .get_telemetry()
                    .get(
                        "state",
                        "UNKNOWN"
                    )
                )

                # -----------------------------------------------------------
                # Telemetry update
                # -----------------------------------------------------------
                if (
                    now - last_push
                    >= (
                        1.0
                        / self.config.telemetry_push_hz
                    )
                ):

                    self._broadcast_telemetry(

                        status,

                        position,

                        boxes_out,

                        pending_detections
                    )

                    pending_detections = []

                    last_push = now

        except KeyboardInterrupt:

            logger.info(
                "Interrupted by user (Ctrl+C)."
            )

        finally:

            self.shutdown()

    # -----------------------------------------------------------------------
    # WebSocket
    # -----------------------------------------------------------------------
    def _run_ws_server(self):

        asyncio.run(
            self._ws_main()
        )

    async def _ws_handler(
        self,
        websocket
    ):

        self.ws_clients.add(
            websocket
        )

        logger.info(
            f"Dashboard connected "
            f"({len(self.ws_clients)} total)"
        )

        try:

            async for raw in websocket:

                try:

                    cmd = json.loads(
                        raw
                    )

                except json.JSONDecodeError:

                    continue

                if (
                    cmd.get("type")
                    != "command"
                ):
                    continue

                self._handle_dashboard_command(
                    cmd
                )

        finally:

            self.ws_clients.discard(
                websocket
            )

    # -----------------------------------------------------------------------
    # Dashboard commands
    # -----------------------------------------------------------------------
    def _handle_dashboard_command(
        self,
        cmd: dict
    ):

        action = cmd.get(
            "action"
        )

        if action == "emergency_stop":

            self.stop_requested_by_dashboard()

        elif action == "set_mode":

            mode = cmd.get(
                "mode",
                ""
            )

            try:

                self.robot_link.set_mode(
                    mode
                )

                logger.info(
                    f"Mode set to "
                    f"{mode.upper()} "
                    f"by dashboard"
                )

            except ValueError as e:

                logger.warning(
                    str(e)
                )

        elif action == "drive":

            direction = cmd.get(
                "direction",
                ""
            )

            try:

                self.robot_link.drive(
                    direction
                )

            except ValueError as e:

                logger.warning(
                    str(e)
                )

        elif action == "resume":

            self.robot_link.resume()

    def stop_requested_by_dashboard(
        self
    ):

        self.robot_link.emergency_stop()

        logger.warning(
            "EMERGENCY STOP sent via dashboard."
        )

    # -----------------------------------------------------------------------
    # WebSocket broadcast
    # -----------------------------------------------------------------------
    async def _broadcaster_loop(
        self
    ):

        while True:

            try:

                msg = (
                    self.outbound
                    .get_nowait()
                )

                if self.ws_clients:

                    await asyncio.gather(
                        *[
                            c.send(
                                json.dumps(
                                    msg
                                )
                            )
                            for c
                            in list(
                                self.ws_clients
                            )
                        ],
                        return_exceptions=True,
                    )

            except queue.Empty:

                await asyncio.sleep(
                    0.05
                )

    async def _ws_main(self):

        async with websockets.serve(
            self._ws_handler,
            self.config.ws_host,
            self.config.ws_port
        ):

            await self._broadcaster_loop()

    # -----------------------------------------------------------------------
    # HTTP video server
    # -----------------------------------------------------------------------
    def _run_http_server(self):

        app = Flask(
            __name__
        )

        @app.route(
            "/video_feed"
        )
        def video_feed():

            def gen():

                while True:

                    jpg = (
                        self.shared.snapshot()
                    )

                    if jpg is not None:

                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpg
                            + b"\r\n"
                        )

                    time.sleep(
                        0.03
                    )

            return Response(
                gen(),
                mimetype=(
                    "multipart/x-mixed-replace; "
                    "boundary=frame"
                )
            )

        @app.route(
            "/drive"
        )
        def drive():

            direction = request.args.get(
                "dir",
                ""
            )

            try:

                self.robot_link.drive(
                    direction
                )

                return {
                    "ok": True
                }

            except ValueError as e:

                return {
                    "ok": False,
                    "error": str(e)
                }, 400

        app.run(

            host=self.config.http_host,

            port=self.config.http_port,

            threaded=True,

            debug=False,

            use_reloader=False
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args() -> WebBridgeConfig:

    parser = argparse.ArgumentParser(
        description=(
            "UGV YOLO Vision System "
            "— webapp + robot bridge"
        )
    )

    parser.add_argument(
        "--model",
        default="best.pt",
        help="Path to YOLO model weights"
    )

    parser.add_argument(
        "--transport",
        choices=[
            "udp",
            "tcp"
        ],
        default="udp",
        help=(
            "udp = low-latency ESP32-S3 UDP stream. "
            "tcp = MJPEG/HTTP/USB source."
        ),
    )

    parser.add_argument(
        "--source",
        default=(
            "http://192.168.1.XX/stream"
        ),
        help=(
            "[--transport tcp only] "
            "ESP32-CAM MJPEG stream URL "
            "or USB camera number"
        )
    )

    parser.add_argument(
        "--udp-listen-ip",
        default="0.0.0.0"
    )

    parser.add_argument(
        "--udp-port",
        type=int,
        default=5005
    )

    parser.add_argument(
        "--conf",
        type=float,
        default=0.20,
        help="YOLO confidence threshold"
    )

    parser.add_argument(
        "--classes",
        nargs="*",
        default=None,
        help=(
            "Optional classes to detect. "
            "Example: --classes crack missing_bolt"
        )
    )

    # -------------------------------------------------------
    # obstacle IS IGNORED BY DEFAULT
    # -------------------------------------------------------
    parser.add_argument(
        "--exclude-classes",
        nargs="*",
        default=["obstacle"],
        help=(
            "Classes YOLO should completely ignore. "
            "Default: obstacle"
        )
    )

    parser.add_argument(
        "--ws-port",
        type=int,
        default=8765
    )

    parser.add_argument(
        "--http-port",
        type=int,
        default=5001
    )

    parser.add_argument(
        "--debounce",
        type=float,
        default=8.0
    )

    parser.add_argument(
        "--robot-port",
        default="/dev/rfcomm0"
    )

    parser.add_argument(
        "--robot-baud",
        type=int,
        default=9600
    )

    # -----------------------------------------------------------------------
    # Sensors
    # -----------------------------------------------------------------------
    parser.add_argument(
        "--sensor-port",
        type=str,
        default="192.168.137.105"
    )

    parser.add_argument(
        "--sensor-poll-hz",
        type=float,
        default=10.0
    )

    parser.add_argument(
        "--wheel-radius-cm",
        type=float,
        default=5.0
    )

    parser.add_argument(
        "--magnets-per-rev",
        type=int,
        default=8
    )

    # -----------------------------------------------------------------------
    # TFT
    # -----------------------------------------------------------------------
    parser.add_argument(
        "--display-ip",
        type=str,
        default=None
    )

    parser.add_argument(
        "--display-port",
        type=int,
        default=5006
    )

    # -----------------------------------------------------------------------
    # Dataset collection
    # -----------------------------------------------------------------------
    parser.add_argument(
        "--collect",
        action="store_true"
    )

    parser.add_argument(
        "--dataset-dir",
        default="collected_frames"
    )

    parser.add_argument(
        "--collect-conf",
        type=float,
        default=0.6
    )

    parser.add_argument(
        "--collect-cooldown",
        type=float,
        default=1.0
    )

    parser.add_argument(
        "--collect-max",
        type=int,
        default=None
    )

    parser.add_argument(
        "--no-collect-labels",
        action="store_true"
    )

    args = parser.parse_args()

    source = args.source

    if str(source).isdigit():
        source = int(source)

    return WebBridgeConfig(

        model_path=args.model,

        transport=args.transport,

        camera_source=source,

        udp_listen_ip=args.udp_listen_ip,

        udp_port=args.udp_port,

        confidence_threshold=args.conf,

        target_classes=args.classes,

        exclude_classes=args.exclude_classes,

        ws_port=args.ws_port,

        http_port=args.http_port,

        defect_debounce_sec=args.debounce,

        robot_port=args.robot_port,

        robot_baud=args.robot_baud,

        sensor_host=(
            args.sensor_port or None
        ),

        sensor_poll_hz=args.sensor_poll_hz,

        wheel_radius_cm=args.wheel_radius_cm,

        magnets_per_rev=args.magnets_per_rev,

        display_ip=args.display_ip,

        display_port=args.display_port,

        collect_dataset=args.collect,

        dataset_dir=args.dataset_dir,

        collect_min_confidence=args.collect_conf,

        collect_cooldown_sec=args.collect_cooldown,

        collect_max_frames=args.collect_max,

        collect_labels=(
            not args.no_collect_labels
        ),
    )


# ---------------------------------------------------------------------------
# START
# ---------------------------------------------------------------------------
if __name__ == "__main__":

    config = parse_args()

    system = WebDashboardVisionSystem(
        config
    )

    system.run()