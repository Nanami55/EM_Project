# 🚂 Vision-Guided UGV for Railway Track Defect Inspection

A prototype Unmanned Ground Vehicle (UGV) that patrols a railway track, streams live video to a laptop, detects track defects with a custom-trained **YOLO** model, and logs *where* each defect was found using wheel-odometry from a Hall-effect sensor. An operator dashboard (Streamlit) provides manual driving, live detections, sensor telemetry, and per-link latency.

> **Status:** research / demonstration prototype. See [Known Issues](#-known-issues--roadmap).

<!-- Add a demo GIF or screenshots here -->
<!-- ![Dashboard](docs/dashboard.png) -->
<!-- ![Rover](docs/rover.jpg) -->

---

## ✨ Features

- **Live video over UDP** from an ESP32-S3-CAM (chunked JPEG frames, low latency, drops stale frames instead of stalling)
- **Real-time defect detection** with a custom YOLO model (`bestv4.pt`) running on the laptop
- **Defect localisation** using Hall-effect pulse counting + wheel radius to estimate distance travelled
- **Environmental telemetry** (temperature / humidity) from a DHT22
- **Manual / Auto drive modes** with a keyboard D-pad in the dashboard
- **On-vehicle status display** driven by the ESP32-S3-CAM board
- **Pan/tilt sensor mount** driven by the ESP8266
- **Per-link latency panel** (camera, robot, sensor, WebSocket) in the dashboard
- **Safe boot:** firmware starts in manual-idle so motors never spin on power-up

---

## 🧱 System Architecture

```
                    ┌──────────────────────────────────────────────┐
                    │                   LAPTOP                      │
                    │                                               │
  ESP32-S3-CAM ─UDP─▶  vision_webapp_bridge_with_display.py          │
   (video + TFT)    │     ├─ cam_link.py            (UDP frames)      │
        ▲           │     ├─ ugv_vision_system.py   (YOLO inference)  │
        └─UDP───────┤     ├─ esp32_display_link.py  (display output)  │──▶ Flask  :5001
     (display)      │     ├─ Sensor_link.py         (ESP8266 poll)    │    /video_feed
                    │     └─ robot_link.py          (Arduino serial)  │         │
  ESP8266 ◀─WiFi────┤                                                 │         ▼
 (Hall+DHT22+       │                                                 │   app.py (Streamlit)
  pan/tilt)         │                                                 │    + ws_client.py
                    │                                                 │    (operator dashboard)
  Arduino Uno ◀─COM5┤                                                 │
 (BTS7960 motors)   └──────────────────────────────────────────────┘
```

**Why this split?**

| Decision | Reason |
|---|---|
| YOLO runs on the **laptop**, not on the ESP32 | The ESP32-S3 can't run a detection model at a usable frame rate |
| **UDP** for video instead of TCP | Avoids head-of-line blocking; an incomplete frame is simply dropped when a newer `frame_id` arrives |
| Sensors are **polled** by the laptop | Keeps the ESP8266 firmware simple and lets the laptop timestamp every reading |
| Hall sensor via **digital pulse counting** | The A3144E is a digital sensor; its analog pin is *not* a usable output |

---

## 📁 Repository Structure

```
.
├── app.py                              # Streamlit operator dashboard
├── bestv4.pt                           # Custom-trained YOLO weights
├── cam_link.py                         # UDP camera receiver / frame reassembly
├── esp32_display_link.py               # Sends status/detections to the on-vehicle display
├── robot_link.py                       # Serial link to the Arduino motor controller
├── Sensor_link.py                      # WiFi link to the ESP8266 (Hall, DHT22, pan/tilt)
├── ugv_vision_system.py                # YOLO inference + detection logic
├── vision_webapp_bridge_with_display.py# Main server: ties everything together, serves video feed
├── ws_client.py                        # WebSocket client used by the dashboard
│
├── arduino_rover/                      # Firmware: Arduino Uno + BTS7960 motor drivers
├── esp32s3cam_vision_display/          # Firmware: ESP32-S3-CAM (UDP video streamer + display)
└── esp8266_sensor_pantilt/             # Firmware: ESP8266 (Hall-effect, DHT22, pan/tilt)
```

---

## 🔧 Hardware

| Component | Role |
|---|---|
| ESP32-S3-CAM (OV3640) | Video capture, UDP streaming, on-vehicle display |
| ESP8266 | Hall-effect pulse counting, DHT22 readings, pan/tilt control |
| Arduino Uno | Motor control |
| 2× BTS7960 motor drivers | Drive four DC motors |
| A3144E Hall-effect sensor | Wheel odometry (pulse counting on D0) |
| DHT22 | Temperature / humidity |
| HC-05 (ZS-040) | Bluetooth serial link to the Arduino (appears as a COM port) |
| 4× LM2596 buck converters | Voltage regulation |
| 3S LiPo battery | Power |

> 💡 Add a wiring diagram / photo under `docs/` and link it here.

---

## 🧰 Software Requirements

- Python 3.9+
- A virtual environment is strongly recommended

```bash
pip install ultralytics opencv-python numpy pyserial websockets streamlit flask
```

> ⚠️ **Install `pyserial`, not `serial`.** Having the unrelated `serial` package installed causes
> `AttributeError: module 'serial' has no attribute 'Serial'`. If you hit it:
> ```bash
> pip uninstall serial
> pip install --force-reinstall pyserial
> ```

Firmware is built with the **Arduino IDE** (or ESP-IDF for the ESP32-S3).

---

## 🚀 Getting Started

### 1. Flash the firmware

| Board | Folder | Notes |
|---|---|---|
| Arduino Uno | `arduino_rover/` | **Disconnect HC-05 TX from Arduino pin 0 (RX) before uploading**, reconnect afterwards |
| ESP32-S3-CAM | `esp32s3cam_vision_display/` | Set WiFi credentials and the laptop IP before flashing |
| ESP8266 | `esp8266_sensor_pantilt/` | Set WiFi credentials before flashing |

### 2. Network setup

All devices and the laptop must be on the same network. This project was developed using a **Windows Mobile Hotspot** (laptop on `192.168.137.x`).

Example addresses used in this README (change to match your setup):

| Device | IP |
|---|---|
| ESP8266 (sensors) | `192.168.137.57` |
| ESP32-S3-CAM (display) | `192.168.137.47` |
| Arduino (via Bluetooth) | `COM5` |

### 3. Start the vision bridge (Terminal 1)

```powershell
python .\vision_webapp_bridge_with_display.py `
  --model bestv4.pt `
  --transport udp --udp-port 5005 `
  --robot-port COM5 `
  --sensor-port 192.168.137.57 `
  --wheel-radius-cm 3.5 `
  --display-ip 192.168.137.47 `
  --conf 0.1
```

### 4. Start the dashboard (Terminal 2)

```powershell
$env:UGV_VIDEO_URL = "http://localhost:5001/video_feed"
streamlit run app.py
```

Open the URL Streamlit prints (usually `http://localhost:8501`).

### Command-line arguments

| Argument | Description |
|---|---|
| `--model` | Path to YOLO weights. **Always pass this** — otherwise Ultralytics may fall back to a generic COCO model |
| `--transport` | Video transport (`udp`) |
| `--udp-port` | UDP port the ESP32-S3-CAM streams to (`5005`) |
| `--robot-port` | Serial/Bluetooth COM port of the Arduino (`COM5`) |
| `--sensor-port` | Address of the ESP8266 sensor node (IP, e.g. `192.168.137.57`) |
| `--wheel-radius-cm` | Wheel radius in cm, used to convert Hall pulses into distance (`3.5`) |
| `--display-ip` | IP of the on-vehicle display (ESP32-S3-CAM) |
| `--conf` | YOLO confidence threshold (`0.1` is permissive — raise it to cut false positives) |

---

## 🎮 Usage

- **Manual mode** (default on boot): drive with the on-screen / keyboard D-pad in the dashboard.
- **Auto mode**: toggle in the dashboard to let the rover patrol on its own.
- Detections are overlaid on the live feed and displayed on the vehicle screen.
- The latency panel shows the round-trip health of each link (camera, robot, sensors, WebSocket).

---

## 🛠 Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Detects generic objects, not rail defects | `--model` omitted — YOLO fell back to a COCO model |
| `module 'serial' has no attribute 'Serial'` | Wrong package installed — keep only `pyserial` |
| Arduino upload fails | HC-05 TX is still connected to pin 0 — unplug it while uploading |
| Camera image upside-down | `s->set_vflip(s, 1)` in the ESP32-S3-CAM sensor init |
| Video stutters or freezes | Weak WiFi / 2.4 GHz congestion; UDP drops incomplete frames by design |
| Bluetooth link unstable | 2.4 GHz interference from camera WiFi, or Windows power management; try re-pairing via the classic Bluetooth SPP dialog |
| Arduino resets when motors start | Motor inrush causing brown-out on a shared rail — use separate/decoupled supply for logic |
| DHT22 reacts slowly | Physical sensor lag (seconds to tens of seconds) — not a software issue |

---

## ⚠️ Known Issues & Roadmap

- [ ] **Motor brown-out:** Arduino may reset under motor inrush on a shared power rail
- [ ] **Bluetooth stability:** evaluating replacing the HC-05 with an ESP8266-based WiFi link for longer range
- [ ] **Defect position offset:** querying Hall-sensor distance *after* detection introduces a speed-dependent error. Planned fix: timestamped rolling buffer on the ESP8266 so distance can be looked up at frame-capture time
- [ ] Battery voltage sensing (divider on A0) not yet calibrated
- [ ] Dataset / training notebook and model evaluation results to be added

---

## 🧠 Model

- Framework: [Ultralytics YOLO](https://docs.ultralytics.com/)
- Weights: `bestv4.pt` (manual trained yolo26x model)
- Inference runs on the laptop (GPU recommended)

<!-- Add: dataset source, classes, training settings, mAP / precision / recall -->

---

## 🤝 Contributing

This is a student prototype, so expect rough edges.



## 👤 Team :
1.
2.
3.
4. Md. Faisal Sheikh (ME-2210035,BUET,Bangladesh)
