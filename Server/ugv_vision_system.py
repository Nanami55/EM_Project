"""
UGV Vision System
==================
A real-time object-detection pipeline for an Unmanned Ground Vehicle (UGV),
using a YOLO model (Ultralytics) and either:
  - a UDP JPEG stream from the ESP32-S3-CAM (low latency, default now), or
  - the older TCP/HTTP MJPEG or USB webcam source (kept for fallback).

WHAT CHANGED IN THIS VERSION
------------------------------
1. UDP TRANSPORT (new): Added a UDPFrameGrabber that listens for the
   chunked-JPEG UDP packets sent by esp32s3cam_udp_streamer.ino, reassembles
   them into full frames, and decodes them. It plugs into the exact same
   queue-based interface the old TCP-based FrameGrabber used, so the rest
   of the pipeline (inference loop, display, recording) didn't need to
   change at all.

   WHY THIS LOWERS LATENCY (plain English): TCP guarantees delivery by
   retransmitting lost packets and delivering everything in order — which
   means one dropped packet can stall the whole stream while it waits for
   a resend. UDP just drops what's lost and moves on. For live video we'd
   rather skip one damaged frame than freeze waiting for it, so UDP trades
   perfect reliability for lower, steadier latency.

2. DISPLAY WINDOW SIZE (fix): The old code called cv2.imshow() with a
   fixed, non-resizable window, which displayed at the camera's native
   (often small) resolution. Now the window is created with
   cv2.WINDOW_NORMAL (user-resizable) and the frame is explicitly resized
   to a configurable "medium" size before display, controlled by
   --display-width / --display-height.

Run it with:
    # New UDP mode (matches esp32s3cam_udp_streamer.ino)
    python ugv_vision_system.py --transport udp --udp-port 5005

    # Old TCP/HTTP or USB modes still work:
    python ugv_vision_system.py --transport tcp --source http://192.168.1.15:8080/video
    python ugv_vision_system.py --transport tcp --source 0
"""

from __future__ import annotations

import argparse
import logging
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Union

import cv2
import numpy as np
import torch
from ultralytics import YOLO


# ---------------------------------------------------------------------------
# OPTIONAL / DETACHABLE ADD-ON: training-frame collector
# ---------------------------------------------------------------------------
# dataset_collector.py is a separate file, not required for this script to
# run. If it's present next to this file, --collect becomes available and
# good frames get saved to disk for retraining on real ESP32-S3 image
# quality. If it's missing (deleted, moved, never added), this import just
# fails and gets caught -- everything below runs exactly as it did before,
# with collection silently disabled. Nothing else needs to change either way.
try:
    from dataset_collector import DatasetCollector, CollectorConfig
    _COLLECTOR_AVAILABLE = True
except ImportError:
    DatasetCollector = None
    CollectorConfig = None
    _COLLECTOR_AVAILABLE = False


# ---------------------------------------------------------------------------
# 1. CONFIGURATION
# ---------------------------------------------------------------------------
@dataclass
class VisionConfig:
    """
    Every "settings knob" for the system lives here in ONE place.
    """
    model_path: str = "best.pt"
    transport: str = "udp"                        # "udp" (new, low-latency) or "tcp" (old)
    camera_source: Union[str, int] = 0             # used only when transport == "tcp"
    udp_listen_ip: str = "0.0.0.0"                 # used only when transport == "udp"
    udp_port: int = 5005                           # must match UDP_TARGET_PORT on the ESP32
    confidence_threshold: float = 0.15
    frame_queue_size: int = 1
    window_name: str = "UGV Vision System"
    display_width: int = 960                       # medium-sized display, not the tiny native box
    display_height: int = 720
    target_classes: Optional[list] = None
    exclude_classes: Optional[list] = None
    save_video_path: Optional[str] = None
    reconnect_delay_sec: float = 1.0
    max_consecutive_failures: int = 30000

    # Optional dataset collection (needs dataset_collector.py present; see
    # the try/except import above). Ignored entirely if that file is absent.
    collect_dataset: bool = False
    dataset_dir: str = "collected_frames"
    collect_min_confidence: float = 0.1
    collect_cooldown_sec: float = 1.0
    collect_max_frames: Optional[int] = None
    collect_labels: bool = True


# ---------------------------------------------------------------------------
# 2. LOGGING
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("UGVVision")


# ---------------------------------------------------------------------------
# 3a. THREADED TCP/HTTP FRAME GRABBER (kept for fallback / USB webcam use)
# ---------------------------------------------------------------------------
class FrameGrabber(threading.Thread):
    """
    Reads frames from a TCP/HTTP source (IP Webcam app, MJPEG stream, or a
    USB camera) in its own background thread. Unchanged from the original
    version — kept as a fallback transport.
    """

    def __init__(self, source: Union[str, int], queue_size: int = 1):
        super().__init__(daemon=True)
        self.source = source
        self.frame_queue: "queue.Queue" = queue.Queue(maxsize=queue_size)
        self._stop_event = threading.Event()
        self.cap: Optional[cv2.VideoCapture] = None
        self.consecutive_failures = 0

    def _open_capture(self) -> bool:
        self.cap = cv2.VideoCapture(self.source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return self.cap.isOpened()

    def run(self):
        if not self._open_capture():
            logger.error(f"Could not open camera source: {self.source}")
            return

        logger.info("TCP frame grabber thread started.")
        while not self._stop_event.is_set():
            success, frame = self.cap.read()

            if not success:
                self.consecutive_failures += 1
                logger.warning(
                    f"Frame read failed ({self.consecutive_failures} in a row). "
                    f"Reconnecting in {self.reconnect_wait()}s..."
                )
                time.sleep(self.reconnect_wait())
                if self.cap:
                    self.cap.release()
                self._open_capture()
                continue

            self.consecutive_failures = 0

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put(frame)

        if self.cap:
            self.cap.release()
        logger.info("TCP frame grabber thread stopped.")

    def reconnect_wait(self) -> float:
        return 1.0

    def read(self, timeout: float = 2.0):
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self):
        self._stop_event.set()


# ---------------------------------------------------------------------------
# 3b. THREADED UDP FRAME GRABBER (new — matches esp32s3cam_udp_streamer.ino)
# ---------------------------------------------------------------------------
class UDPFrameGrabber(threading.Thread):
    """
    Listens for chunked JPEG frames sent over UDP by the ESP32-S3-CAM and
    reassembles them.

    PACKET FORMAT (must match the .ino sender exactly):
        uint32 frame_id, uint16 chunk_index, uint16 total_chunks, then
        raw JPEG bytes for that chunk. Little-endian ("<IHH" in struct terms).

    WHY REASSEMBLY IS NEEDED (plain English): A JPEG frame is bigger than
    one UDP packet can safely carry, so the ESP32 chops it into pieces and
    numbers them. This class collects pieces by frame_id until either (a)
    all pieces for that frame have arrived — decode it — or (b) a newer
    frame_id shows up before the old one finished — throw the incomplete
    old one away. This means one lost chunk costs you exactly one frame,
    never a stall.

    Exposes the same read()/stop() interface as the TCP FrameGrabber, so
    the rest of the app doesn't care which transport is in use.
    """

    HEADER_FORMAT = "<IHH"  # frame_id (u32), chunk_index (u16), total_chunks (u16)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

    def __init__(self, listen_ip: str, port: int, queue_size: int = 1):
        super().__init__(daemon=True)
        self.listen_ip = listen_ip
        self.port = port
        self.frame_queue: "queue.Queue" = queue.Queue(maxsize=queue_size)
        self._stop_event = threading.Event()
        self.sock: Optional[socket.socket] = None

        # Reassembly state for whichever frame is currently in progress.
        self._current_frame_id: Optional[int] = None
        self._chunks: dict[int, bytes] = {}
        self._total_chunks_expected: Optional[int] = None

    def _open_socket(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.listen_ip, self.port))
            self.sock.settimeout(1.0)  # lets the stop-check loop run periodically
            return True
        except OSError as exc:
            logger.error(f"Could not bind UDP socket on {self.listen_ip}:{self.port} -> {exc}")
            return False

    def _reset_reassembly(self, new_frame_id: int, total_chunks: int):
        self._current_frame_id = new_frame_id
        self._chunks = {}
        self._total_chunks_expected = total_chunks

    def _try_complete_frame(self) -> Optional[bytes]:
        if self._total_chunks_expected is None:
            return None
        if len(self._chunks) != self._total_chunks_expected:
            return None
        try:
            return b"".join(self._chunks[i] for i in range(self._total_chunks_expected))
        except KeyError:
            return None  # shouldn't happen given the length check, but stay safe

    def run(self):
        if not self._open_socket():
            return

        logger.info(f"UDP frame grabber listening on {self.listen_ip}:{self.port}")

        while not self._stop_event.is_set():
            try:
                packet, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(packet) < self.HEADER_SIZE:
                continue  # malformed / too-small packet, ignore

            frame_id, chunk_index, total_chunks = struct.unpack(
                self.HEADER_FORMAT, packet[: self.HEADER_SIZE]
            )
            payload = packet[self.HEADER_SIZE :]

            # A new frame_id means the previous one (if incomplete) is stale —
            # drop it rather than waiting for chunks that will never arrive.
            if frame_id != self._current_frame_id:
                self._reset_reassembly(frame_id, total_chunks)

            self._chunks[chunk_index] = payload

            jpeg_bytes = self._try_complete_frame()
            if jpeg_bytes is None:
                continue

            # Decode JPEG bytes -> OpenCV BGR frame.
            np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Ready to start collecting the next frame.
            self._current_frame_id = None
            self._total_chunks_expected = None
            self._chunks = {}

            if frame is None:
                continue  # corrupted JPEG (partial chunk loss) — skip silently

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put(frame)

        if self.sock:
            self.sock.close()
        logger.info("UDP frame grabber thread stopped.")

    def read(self, timeout: float = 2.0):
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self):
        self._stop_event.set()


# ---------------------------------------------------------------------------
# 4. MAIN VISION SYSTEM
# ---------------------------------------------------------------------------
class UGVVisionSystem:
    """Orchestrates model loading, frame grabbing, inference, and display."""

    def __init__(self, config: VisionConfig):
        self.config = config
        self.device = self._select_device()
        self.model = self._load_model()
        self.grabber = self._build_grabber()
        self.collector = self._build_collector()
        self.video_writer: Optional[cv2.VideoWriter] = None

    # ---- Setup helpers ----------------------------------------------------
    def _select_device(self) -> str:
        device = "0" if torch.cuda.is_available() else "cpu"
        logger.info(f"Using device: {'GPU (CUDA)' if device == '0' else 'CPU'}")
        return device

    def _load_model(self) -> YOLO:
        logger.info(f"Loading YOLO model from '{self.config.model_path}'...")
        try:
            model = YOLO(self.config.model_path)
        except Exception as exc:
            logger.error(f"Failed to load model: {exc}")
            raise
        logger.info("Model loaded successfully.")
        return model

    def _build_grabber(self):
        if self.config.transport == "udp":
            logger.info("Transport: UDP (low latency)")
            return UDPFrameGrabber(
                self.config.udp_listen_ip, self.config.udp_port, self.config.frame_queue_size
            )
        logger.info("Transport: TCP/HTTP")
        return FrameGrabber(self.config.camera_source, self.config.frame_queue_size)

    def _build_collector(self):
        """Returns a DatasetCollector, or None if collection wasn't asked
        for, or was asked for but dataset_collector.py isn't available."""
        if not self.config.collect_dataset:
            return None
        if not _COLLECTOR_AVAILABLE:
            logger.warning(
                "--collect was passed but dataset_collector.py wasn't found next to "
                "this script. Skipping frame collection -- add that file back to enable it."
            )
            return None
        return DatasetCollector(CollectorConfig(
            output_dir=self.config.dataset_dir,
            min_confidence=self.config.collect_min_confidence,
            cooldown_sec=self.config.collect_cooldown_sec,
            max_frames=self.config.collect_max_frames,
            save_labels=self.config.collect_labels,
        ))

    def _setup_video_writer(self, frame_shape) -> None:
        if not self.config.save_video_path:
            return
        h, w = frame_shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            self.config.save_video_path, fourcc, 20.0, (w, h)
        )
        logger.info(f"Recording annotated output to {self.config.save_video_path}")

    def _resolve_class_filter(self):
        """Return class IDs that YOLO is allowed to output.

        --classes acts as an allow-list.
        --exclude-classes removes unwanted classes (for example: normal).
        Class matching is case-insensitive.
        """
        name_to_id = {str(v).strip().lower(): k for k, v in self.model.names.items()}

        if self.config.target_classes:
            allowed = {
                name_to_id[name.strip().lower()]
                for name in self.config.target_classes
                if name.strip().lower() in name_to_id
            }
        else:
            allowed = set(self.model.names.keys())

        for name in (self.config.exclude_classes or []):
            class_id = name_to_id.get(name.strip().lower())
            if class_id is not None:
                allowed.discard(class_id)
            else:
                logger.warning("Requested excluded class '%s' was not found. Model classes: %s",
                               name, list(self.model.names.values()))

        if not allowed:
            logger.warning("Class filter removed every class; no detections will be shown.")
            return []

        return sorted(allowed)

    # ---- Main loop ----------------------------------------------------------
    def run(self):
        self.grabber.start()
        time.sleep(1.0)

        # WINDOW_NORMAL makes the window user-resizable (the default,
        # WINDOW_AUTOSIZE, locks it to the frame's native — often tiny —
        # resolution). We also set an explicit medium starting size.
        cv2.namedWindow(self.config.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.config.window_name, self.config.display_width, self.config.display_height)

        logger.info("Starting live inference. Press 'q' in the video window to quit.")

        fps_smoothed = 0.0
        alpha = 0.1
        prev_time = time.time()
        empty_frame_streak = 0

        try:
            while True:
                frame = self.grabber.read()

                if frame is None:
                    empty_frame_streak += 1
                    if empty_frame_streak >= self.config.max_consecutive_failures:
                        logger.error("No frames received for too long. Exiting.")
                        break
                    continue
                empty_frame_streak = 0

                results = self.model.predict(
                    frame,
                    device=self.device,
                    conf=(self.config.confidence_threshold),
                    classes=self._resolve_class_filter(),
                    verbose=False,
                )

                # Optional: hand the RAW frame (pre-annotation) + this frame's
                # detections to the collector. No-op if collection is off or
                # dataset_collector.py isn't present.
                if self.collector is not None:
                    self.collector.maybe_save(frame, results, self.model.names)

                annotated_frame = results[0].plot()

                now = time.time()
                instant_fps = 1.0 / max(now - prev_time, 1e-6)
                fps_smoothed = alpha * instant_fps + (1 - alpha) * fps_smoothed
                prev_time = now

                cv2.putText(
                    annotated_frame,
                    f"FPS: {fps_smoothed:.1f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

                # Recording keeps the ORIGINAL resolution (undownscaled) —
                # only the on-screen preview is resized for a bigger window.
                if self.config.save_video_path:
                    if self.video_writer is None:
                        self._setup_video_writer(annotated_frame.shape)
                    self.video_writer.write(annotated_frame)

                display_frame = cv2.resize(
                    annotated_frame,
                    (self.config.display_width, self.config.display_height),
                    interpolation=cv2.INTER_LINEAR,
                )
                cv2.imshow(self.config.window_name, display_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Quit key pressed. Shutting down.")
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user (Ctrl+C).")
        finally:
            self.shutdown()

    def shutdown(self):
        self.grabber.stop()
        self.grabber.join(timeout=2.0)
        if self.video_writer:
            self.video_writer.release()
        if self.collector is not None:
            logger.info(self.collector.summary())
        cv2.destroyAllWindows()
        logger.info("Vision system shut down cleanly.")


# ---------------------------------------------------------------------------
# 5. COMMAND-LINE INTERFACE
# ---------------------------------------------------------------------------
def parse_args() -> VisionConfig:
    parser = argparse.ArgumentParser(description="UGV YOLO Vision System")
    parser.add_argument("--model", default="best.pt", help="Path to YOLO model weights")
    parser.add_argument(
        "--transport",
        choices=["udp", "tcp"],
        default="udp",
        help="udp = low-latency ESP32-S3 UDP stream (default). tcp = old MJPEG/HTTP/USB source.",
    )
    parser.add_argument(
        "--source",
        default="http://192.168.1.XX:8080/video",
        help="[--transport tcp only] IP Webcam URL, or an integer for a USB camera (e.g. 0)",
    )
    parser.add_argument(
        "--udp-listen-ip",
        default="0.0.0.0",
        help="[--transport udp only] local IP to listen on (0.0.0.0 = all interfaces)",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=5005,
        help="[--transport udp only] must match UDP_TARGET_PORT in the ESP32 sketch",
    )
    parser.add_argument("--conf", type=float, default=0.2, help="Confidence threshold (0.0 - 1.0)")
    parser.add_argument("--save", default=None, help="Optional path to save output video, e.g. output.mp4")
    parser.add_argument(
        "--classes",
        nargs="*",
        default=None,
        help="Optional list of class names to detect only, e.g. --classes person car",
    )
    parser.add_argument(
        "--exclude-classes",
        nargs="*",
        default=["normal"],
        help="Class names to ignore completely. Default: normal",
    )
    parser.add_argument("--display-width", type=int, default=960, help="On-screen preview window width")
    parser.add_argument("--display-height", type=int, default=720, help="On-screen preview window height")

    # Optional dataset collection -- only works if dataset_collector.py is
    # present next to this script (see the try/except import near the top).
    parser.add_argument(
        "--collect",
        action="store_true",
        help="Save good frames to disk while running, for retraining on real camera quality. "
             "Requires dataset_collector.py to be present next to this script.",
    )
    parser.add_argument("--dataset-dir", default="collected_frames", help="Where to save collected training frames")
    parser.add_argument("--collect-conf", type=float, default=0.1, help="Min detection confidence for a frame to be saved")
    parser.add_argument("--collect-cooldown", type=float, default=1.0, help="Min seconds between saved frames")
    parser.add_argument("--collect-max", type=int, default=None, help="Stop collecting after this many frames (default: unlimited)")
    parser.add_argument("--no-collect-labels", action="store_true", help="Don't save YOLO-format .txt labels alongside saved images")

    args = parser.parse_args()

    source: Union[str, int] = args.source
    if str(source).isdigit():
        source = int(source)

    return VisionConfig(
        model_path=args.model,
        transport=args.transport,
        camera_source=source,
        udp_listen_ip=args.udp_listen_ip,
        udp_port=args.udp_port,
        confidence_threshold=args.conf,
        save_video_path=args.save,
        target_classes=args.classes,
        exclude_classes=args.exclude_classes,
        display_width=args.display_width,
        display_height=args.display_height,
        collect_dataset=args.collect,
        dataset_dir=args.dataset_dir,
        collect_min_confidence=args.collect_conf,
        collect_cooldown_sec=args.collect_cooldown,
        collect_max_frames=args.collect_max,
        collect_labels=not args.no_collect_labels,
    )


if __name__ == "__main__":
    config = parse_args()
    system = UGVVisionSystem(config)
    system.run()
