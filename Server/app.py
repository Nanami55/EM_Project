"""
app.py — Railway Track Inspection UGV: Command & Control Dashboard
====================================================================
IMPORTANT ARCHITECTURAL RULE:
This file must never run YOLO, read serial ports, or do anything CPU/IO
heavy. It only:
  1. Drains messages that ws_client.RobotBridge has already received
     from the robot/AI process, and
  2. Renders them.
All the "real work" happens in mock_robot_server.py (replace with your
actual robot loop) and is delivered here over a WebSocket via ws_client.py.
"""

import os
import time
from datetime import datetime

import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from streamlit_autorefresh import st_autorefresh
import streamlit.components.v1 as components
from ws_client import get_bridge

# --------------------------------------------------------------------------
# Page setup
# --------------------------------------------------------------------------
st.set_page_config(
    page_title="UGV Track Inspection — Command & Control",
    page_icon="🚆",
    layout="wide",
    initial_sidebar_state="expanded",
)

CUSTOM_CSS = """
<style>
#MainMenu, footer, header {visibility: visible;}
.block-container {padding-top: 1.2rem; padding-bottom: 1rem;}

.status-badge {
    display: inline-block; padding: 6px 14px; border-radius: 6px;
    font-weight: 700; font-size: 0.85rem; letter-spacing: 0.04em;
    text-transform: uppercase; text-align: center; width: 100%;
}
.status-patrol   { background: #0d3b2e; color: #00E5A0; border: 1px solid #00E5A0; }
.status-stopped  { background: #3b2e0d; color: #FFC24B; border: 1px solid #FFC24B; }
.status-defect   { background: #3b0d0d; color: #FF5C5C; border: 1px solid #FF5C5C; animation: pulse 1.2s infinite; }
.status-offline  { background: #1a1a1a; color: #888; border: 1px solid #555; }
@keyframes pulse { 0%{opacity:1;} 50%{opacity:0.55;} 100%{opacity:1;} }

.metric-card {
    background: #131A22; border: 1px solid #223; border-radius: 10px;
    padding: 10px 14px; margin-bottom: 10px;
}
.video-wrap {
    position: relative; width: 100%; aspect-ratio: 16/9;
    background: #000; border: 2px solid #1c2733; border-radius: 10px; overflow: hidden;
}
.video-wrap video, .video-wrap img { width: 100%; height: 100%; object-fit: cover; }
.bbox {
    position: absolute; border: 2px solid #00E5A0; box-sizing: border-box;
}
.bbox.defect { border-color: #FF5C5C; }
.bbox-label {
    position: absolute; top: -20px; left: -2px; background: inherit;
    background-color: #00E5A0; color: #06110c; font-size: 11px; font-weight: 700;
    padding: 1px 6px; border-radius: 3px; white-space: nowrap;
}
.bbox.defect .bbox-label { background-color: #FF5C5C; color: #2b0505; }

.estop-btn button {
    background: #B00020 !important; color: white !important;
    font-weight: 800 !important; font-size: 1.1rem !important;
    height: 3.4rem !important; border: 2px solid #ff4d4d !important;
    letter-spacing: 0.06em;
}
.estop-confirm button {
    background: #7a0000 !important; color: white !important;
    font-weight: 800 !important; height: 3rem !important;
    border: 2px dashed #ff4d4d !important;
}
</style>
"""
st.markdown(CUSTOM_CSS, unsafe_allow_html=True)

# --------------------------------------------------------------------------
# Session state init
# --------------------------------------------------------------------------
defaults = {
    "battery_v": 12.6,
    "ping_ms": 0.0,
    "status": "OFFLINE",
    "position": {"x": 0.0, "y": 0.0},
    "path_history": [],
    "live_boxes": [],
    "defects": [],  # list of dicts: id, label, severity, ts, gps, confidence
    "last_update_ts": 0.0,
    "confirm_stop_pending": False,
    "estop_sent_at": None,
    "selected_defect_id": None,
    "drive_mode": "AUTO",
    "cam_latency_ms": None,
    "robot_latency_ms": None,
    "combined_latency_ms": None,
    "ws_latency_ms": None,
    "sensor_connected": False,
    "temperature_c": None,
    "humidity_pct": None,
    "distance_m": 0.0,
}
for k, v in defaults.items():
    st.session_state.setdefault(k, v)

# --------------------------------------------------------------------------
# Wire up the background bridge (started once per server process)
# --------------------------------------------------------------------------
bridge = get_bridge()
bridge.start()

# Pull the auto-refresh tick — this is what drives "real-time" redraws.
# 500ms is a reasonable UI refresh rate; it is independent of how fast the
# robot/AI loop actually pushes data (that's governed by mock_robot_server.py).
st_autorefresh(interval=500, key="tick")

# Drain whatever arrived on the background thread since last rerun.
for msg in bridge.drain():
    if msg.get("type") == "telemetry":
        st.session_state.battery_v = msg["battery_v"]
        st.session_state.cam_latency_ms = msg.get("cam_latency_ms")
        st.session_state.robot_latency_ms = msg.get("robot_latency_ms")
        st.session_state.combined_latency_ms = msg.get("combined_latency_ms")
        # Genuine WebSocket round-trip: how long ago the bridge stamped
        # this message vs. now. Not perfectly synced clocks, but on a LAN
        # this is a reasonable real number rather than a hardcoded one.
        try:
            msg_ts = datetime.fromisoformat(msg["ts"]).timestamp()
            st.session_state.ws_latency_ms = max(0.0, (time.time() - msg_ts) * 1000.0)
        except (KeyError, ValueError):
            pass
        st.session_state.status = msg["status"]
        st.session_state.position = msg["position"]
        st.session_state.live_boxes = msg.get("live_boxes", [])
        sensors = msg.get("sensors") or {}
        st.session_state.sensor_connected = sensors.get("connected", False)
        st.session_state.temperature_c = sensors.get("temperature_c")
        st.session_state.humidity_pct = sensors.get("humidity_pct")
        st.session_state.distance_m = sensors.get("distance_m", st.session_state.distance_m)
        st.session_state.path_history.append((msg["position"]["x"], msg["position"]["y"]))
        st.session_state.path_history = st.session_state.path_history[-500:]
        st.session_state.last_update_ts = time.time()
        for d in msg.get("detections", []):
            if not any(existing["id"] == d["id"] for existing in st.session_state.defects):
                st.session_state.defects.append(d)

link_is_stale = (time.time() - st.session_state.last_update_ts) > 3
effective_status = "OFFLINE" if (not bridge.connected or link_is_stale) else st.session_state.status

# --------------------------------------------------------------------------
# Sidebar — telemetry + emergency stop
# --------------------------------------------------------------------------
with st.sidebar:
    st.markdown("### 🚆 UGV CONSOLE")
    st.caption("Track Inspection Unit — Line 4B")

    status_class = {
        "PATROL": "status-patrol",
        "STOPPED": "status-stopped",
        "DEFECT_PAUSE": "status-stopped",
        "DEFECT_DETECTED": "status-defect",
        "ESTOPPED": "status-stopped",
        "MANUAL": "status-patrol",
        "OFFLINE": "status-offline",
    }.get(effective_status, "status-offline")
    status_text = effective_status.replace("_", " ")
    st.markdown(f'<div class="status-badge {status_class}">{status_text}</div>', unsafe_allow_html=True)

    st.markdown("---")

    # Battery gauge
    batt_pct = max(0.0, min(1.0, (st.session_state.battery_v - 9.0) / (12.6 - 9.0)))
    fig_batt = go.Figure(go.Indicator(
        mode="gauge+number",
        value=st.session_state.battery_v,
        number={"suffix": " V", "font": {"size": 26}},
        gauge={
            "axis": {"range": [9.0, 12.6], "tickcolor": "#889"},
            "bar": {"color": "#00E5A0" if batt_pct > 0.3 else "#FF5C5C"},
            "bgcolor": "#131A22",
            "borderwidth": 0,
            "steps": [
                {"range": [9.0, 10.5], "color": "#3b0d0d"},
                {"range": [10.5, 11.4], "color": "#3b2e0d"},
                {"range": [11.4, 12.6], "color": "#0d3b2e"},
            ],
        },
        title={"text": "BATTERY (target 11.1V nominal)", "font": {"size": 11}},
    ))
    fig_batt.update_layout(height=170, margin=dict(l=20, r=20, t=40, b=10),
                            paper_bgcolor="rgba(0,0,0,0)", font_color="#E6EDF3")
    st.plotly_chart(fig_batt, use_container_width=True, config={"displayModeBar": False})

    # Connection quality — per-link, real round-trip measurements
    def _quality_label_color(ms):
        if ms is None:
            return "NO LINK", "#888"
        if ms < 40:
            return "EXCELLENT", "#00E5A0"
        if ms < 120:
            return "FAIR", "#FFC24B"
        return "POOR", "#FF5C5C"

    def _latency_row(label, ms, subtitle=""):
        display_ms = None if (not bridge.connected or link_is_stale) else ms
        text, color = _quality_label_color(display_ms)
        value = f"{display_ms:.0f} ms" if display_ms is not None else "—"
        sub_html = f'<div style="font-size:10px;color:#556;">{subtitle}</div>' if subtitle else ""
        return (
                    '<div style="display:flex;justify-content:space-between;align-items:baseline;padding:4px 0;">'
                    f'<div><div style="font-size:11px;color:#889;">{label}</div>{sub_html}</div>'
                    f'<div style="font-size:1.05rem;font-weight:700;color:{color};">'
                    f'{value} <span style="font-size:0.7rem;">· {text}</span></div></div>'
                )

    latency_html = (
        _latency_row("ESP32-CAM (WiFi)", st.session_state.cam_latency_ms, "camera round trip")
        + _latency_row("Arduino (Bluetooth)", st.session_state.robot_latency_ms, "HC-05 round trip")
        + _latency_row("Dashboard (WebSocket)", st.session_state.ws_latency_ms, "browser round trip")
    )

    st.markdown(
        f"""<div class="metric-card">
        <div style="font-size:11px;color:#889;letter-spacing:.04em;margin-bottom:2px;">CONNECTION QUALITY</div>
        {latency_html}
        <div style="border-top:1px solid #223; margin-top:6px; padding-top:6px;
                    display:flex; justify-content:space-between;">
            <div style="font-size:11px;color:#889;">HARDWARE TOTAL (cam + BT)</div>
            <div style="font-size:1.1rem;font-weight:800;">
                {f"{st.session_state.combined_latency_ms:.0f} ms" if st.session_state.combined_latency_ms is not None else "—"}
            </div>
        </div>
        </div>""",
        unsafe_allow_html=True,
    )

    pos = st.session_state.position
    st.markdown(
        f"""<div class="metric-card">
        <div style="font-size:11px;color:#889;letter-spacing:.04em;">POSITION (GPS/ENCODER)</div>
        <div style="font-size:1.1rem;font-weight:700;">x={pos['x']:.1f}  y={pos['y']:.1f}</div>
        </div>""",
        unsafe_allow_html=True,
    )

    # Hall-effect distance + DHT22 temp/humidity — continuous sensor readout
    dist_m = st.session_state.distance_m or 0.0
    dist_text = f"{dist_m/1000:.2f} km" if dist_m >= 1000 else f"{dist_m:.1f} m"
    temp_text = f"{st.session_state.temperature_c:.1f}°C" if st.session_state.temperature_c is not None else "—"
    hum_text = f"{st.session_state.humidity_pct:.0f}%" if st.session_state.humidity_pct is not None else "—"
    sensor_dot = "#00E5A0" if st.session_state.sensor_connected else "#888"
    sensor_label = "connected" if st.session_state.sensor_connected else "no link"
    st.markdown(
        f"""<div class="metric-card">
        <div style="display:flex;justify-content:space-between;align-items:baseline;">
            <div style="font-size:11px;color:#889;letter-spacing:.04em;">SENSOR NODE (ESP8266)</div>
            <div style="font-size:10px;color:{sensor_dot};">● {sensor_label}</div>
        </div>
        <div style="display:flex;justify-content:space-between;align-items:baseline;padding-top:6px;">
            <div style="font-size:11px;color:#889;">Distance traveled</div>
            <div style="font-size:1.1rem;font-weight:800;">{dist_text}</div>
        </div>
        <div style="display:flex;justify-content:space-between;align-items:baseline;padding-top:4px;">
            <div style="font-size:11px;color:#889;">Temp / Humidity</div>
            <div style="font-size:1.05rem;font-weight:700;">{temp_text} · {hum_text}</div>
        </div>
        </div>""",
        unsafe_allow_html=True,
    )

    st.markdown("---")
    st.markdown("##### ⚠️ EMERGENCY STOP")
    st.caption("Two-step confirmation — prevents accidental trips during demos.")

    if not st.session_state.confirm_stop_pending:
        st.markdown('<div class="estop-btn">', unsafe_allow_html=True)
        if st.button("EMERGENCY STOP", use_container_width=True, key="estop_arm"):
            st.session_state.confirm_stop_pending = True
            st.rerun()
        st.markdown("</div>", unsafe_allow_html=True)
    else:
        st.markdown('<div class="estop-confirm">', unsafe_allow_html=True)
        c1, c2 = st.columns(2)
        with c1:
            if st.button("CONFIRM STOP", use_container_width=True, key="estop_confirm"):
                bridge.send_command({"type": "command", "action": "emergency_stop"})
                st.session_state.confirm_stop_pending = False
                st.session_state.estop_sent_at = time.time()
                st.rerun()
        with c2:
            if st.button("Cancel", use_container_width=True, key="estop_cancel"):
                st.session_state.confirm_stop_pending = False
                st.rerun()
        st.markdown("</div>", unsafe_allow_html=True)
        st.warning("Confirm to cut power via Bluetooth/Serial. This cannot be undone remotely.")

    if st.session_state.estop_sent_at and (time.time() - st.session_state.estop_sent_at) < 5:
        st.success("STOP command sent to robot.")

    st.markdown("---")
    st.markdown("##### 🎮 DRIVE MODE")
    st.caption("Auto: robot patrols and stops itself on defects. Manual: you steer, camera still streams.")

    mode_choice = st.radio(
        "Mode", ["AUTO", "MANUAL"],
        index=0 if st.session_state.drive_mode == "AUTO" else 1,
        horizontal=True, label_visibility="collapsed", key="mode_radio",
    )
    if mode_choice != st.session_state.drive_mode:
        st.session_state.drive_mode = mode_choice
        bridge.send_command({"type": "command", "action": "set_mode", "mode": mode_choice.lower()})
        st.rerun()

    if st.session_state.drive_mode == "MANUAL":
        st.caption("Click into the box below, then use arrow keys to steer, 'P' to stop.")

        components.html(
            """
            <div id="drive-catcher" tabindex="0"
                 style="outline:2px dashed #445; border-radius:8px; padding:14px;
                        text-align:center; cursor:pointer; color:#889;">
                <div style="margin-bottom:10px; font-size:0.8rem;">
                    Click here, then use ⬆️⬇️⬅️➡️ and P (stop)
                </div>

                <div style="display:flex; justify-content:center;">
                    <div id="key-up" class="dpad-key">⬆️</div>
                </div>
                <div style="display:flex; justify-content:center; gap:14px; margin:4px 0;">
                    <div id="key-left" class="dpad-key">⬅️</div>
                    <div id="key-stop" class="dpad-key">⏹️</div>
                    <div id="key-right" class="dpad-key">➡️</div>
                </div>
                <div style="display:flex; justify-content:center;">
                    <div id="key-down" class="dpad-key">⬇️</div>
                </div>
            </div>

            <style>
            .dpad-key {
                width: 42px; height: 42px; line-height: 42px;
                font-size: 1.3rem; border-radius: 8px;
                background: #131A22; border: 1px solid #223;
                transition: background 0.08s, border-color 0.08s, transform 0.08s;
            }
            .dpad-key.active {
                background: #00E5A0; border-color: #00E5A0;
                transform: scale(1.08);
            }
            </style>

            <script>
            const box = document.getElementById("drive-catcher");
            let lastSent = 0;
            const THROTTLE_MS = 150;

            const keyToEl = {
                "ArrowUp": document.getElementById("key-up"),
                "ArrowDown": document.getElementById("key-down"),
                "ArrowLeft": document.getElementById("key-left"),
                "ArrowRight": document.getElementById("key-right"),
                "p": document.getElementById("key-stop"),
                "P": document.getElementById("key-stop"),
            };
            const keyToDir = {
                "ArrowUp": "forward", "ArrowDown": "backward",
                "ArrowLeft": "left", "ArrowRight": "right",
                "p": "stop", "P": "stop"
            };

            function send(dir) {
                const now = Date.now();
                if (now - lastSent < THROTTLE_MS) return;
                lastSent = now;
                fetch("http://localhost:5001/drive?dir=" + dir).catch(() => {});
            }

            box.addEventListener("keydown", (e) => {
                if (!keyToDir[e.key]) return;
                e.preventDefault();
                keyToEl[e.key].classList.add("active");
                send(keyToDir[e.key]);
            });

            box.addEventListener("keyup", (e) => {
                if (!keyToEl[e.key]) return;
                keyToEl[e.key].classList.remove("active");
            });

            // if focus is lost mid-press, clear all highlights so a stuck
            // highlight never implies the robot is still driving
            box.addEventListener("blur", () => {
                Object.values(keyToEl).forEach(el => el.classList.remove("active"));
            });
            </script>
            """,
            height=190,
        )
    st.markdown("---")
    st.caption(f"Bridge: {'🟢 connected' if bridge.connected else '🔴 disconnected'} · ws://localhost:8765")

# --------------------------------------------------------------------------
# Main layout — video feed (focal point) + map, then defect log
# --------------------------------------------------------------------------
col_video, col_map = st.columns([3, 2], gap="medium")

with col_video:
    st.markdown("#### 📷 Live Feed — ESP32-CAM (YOLO overlay)")

    # NOTE: vision_webapp_bridge.py already draws YOLO's boxes directly onto
    # the frame (annotated = results[0].plot()) before streaming it over
    # MJPEG. Those baked-in boxes are the accurate, single source of truth.
    # We intentionally do NOT re-draw a second CSS overlay here from
    # `live_boxes` anymore — that overlay was both redundant and prone to
    # drifting out of alignment (object-fit: cover crops the displayed image,
    # which the normalized bbox coordinates don't account for), which is
    # exactly the "less accurate second box" you were seeing.
    # `live_boxes` is still received/stored in session state in case you want
    # it later (e.g. a non-visual use), it's just not rendered as an overlay.

    # Set UGV_VIDEO_URL to your vision_webapp_bridge.py MJPEG endpoint, e.g.
    # export UGV_VIDEO_URL="http://localhost:5001/video_feed"
    # Falls back to a placeholder image when running against the mock server.
    video_url = os.environ.get("UGV_VIDEO_URL", "")
    if video_url:
        video_tag = f'<img src="{video_url}" alt="live camera feed"/>'
    else:
        video_tag = ('<img src="https://images.unsplash.com/photo-1474487548417-781cb71495f3'
                     '?w=1200&q=60" alt="camera feed placeholder"/>')

    st.markdown(
        f"""
        <div class="video-wrap">
            {video_tag}
        </div>
        """,
        unsafe_allow_html=True,
    )
    if not video_url:
        st.caption("No UGV_VIDEO_URL")

with col_map:
    st.markdown("#### 🗺️ Track Map & Defect Locations")

    fig_map = go.Figure()

    # track polyline (static reference geometry)
    track_x = [0, 40, 80, 100, 90, 40, 0, 0]
    track_y = [0, 5, 0, -20, -50, -60, -40, 0]
    fig_map.add_trace(go.Scatter(
        x=track_x, y=track_y, mode="lines",
        line=dict(color="#334455", width=6), name="Track", hoverinfo="skip",
    ))

    # traveled path
    if st.session_state.path_history:
        px, py = zip(*st.session_state.path_history)
        fig_map.add_trace(go.Scatter(
            x=px, y=py, mode="lines", line=dict(color="#00E5A0", width=3),
            name="Path traveled", hoverinfo="skip",
        ))

    # robot marker
    fig_map.add_trace(go.Scatter(
        x=[st.session_state.position["x"]], y=[st.session_state.position["y"]],
        mode="markers", marker=dict(color="#00E5A0", size=16, symbol="circle",
                                     line=dict(color="white", width=2)),
        name="UGV", hovertext="Robot (live)",
    ))

    # defect pins
    if st.session_state.defects:
        dx = [d["gps"]["x"] for d in st.session_state.defects]
        dy = [d["gps"]["y"] for d in st.session_state.defects]
        dtext = [f"{d['id']} · {d['label']} · {d['severity']}" for d in st.session_state.defects]
        fig_map.add_trace(go.Scatter(
            x=dx, y=dy, mode="markers", marker=dict(color="#FF5C5C", size=14, symbol="x"),
            name="Defects", text=dtext, hoverinfo="text",
            customdata=[d["id"] for d in st.session_state.defects],
        ))

    fig_map.update_layout(
        height=430, margin=dict(l=10, r=10, t=10, b=10),
        paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="#0B0F14",
        xaxis=dict(showgrid=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, zeroline=False, visible=False, scaleanchor="x"),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, font=dict(size=10)),
        font_color="#E6EDF3",
    )

    map_event = st.plotly_chart(
        fig_map, use_container_width=True, config={"displayModeBar": False},
        on_select="rerun", key="track_map",
    )

    # handle pin click -> open modal with defect snapshot
    if map_event and map_event.get("selection", {}).get("points"):
        pt = map_event["selection"]["points"][0]
        if pt.get("curve_number") is not None and st.session_state.defects:
            idx = pt.get("point_index")
            defects_trace = st.session_state.defects
            if idx is not None and idx < len(defects_trace) and pt.get("legendgroup", "") != "":
                pass  # legend clicks shouldn't open modal
            clicked_id = None
            try:
                clicked_id = pt["customdata"]
            except (KeyError, TypeError):
                pass
            if clicked_id:
                st.session_state.selected_defect_id = clicked_id


def get_defect_image(defect):
    """Resolve the actual captured frame for a defect, rather than the
    hardcoded placeholder. The AI/backend process is expected to attach the
    snapshot to each detection under one of these keys:
      - "snapshot_b64" / "image_b64": raw base64 JPEG/PNG bytes (no prefix)
      - "snapshot" / "image": either a data URI ("data:image/...;base64,...")
        or an already-base64 string
      - "snapshot_url" / "image_url": a URL/path the frontend can load
    Adjust the key names below if your backend uses a different field.
    Returns (src, is_placeholder).
    """
    for key in ("snapshot_url", "image_url"):
        val = defect.get(key)
        if val:
            return val, False

    for key in ("snapshot_b64", "image_b64", "snapshot", "image"):
        val = defect.get(key)
        if val:
            if val.startswith("data:image"):
                return val, False
            return f"data:image/jpeg;base64,{val}", False

    return (
        "https://images.unsplash.com/photo-1474487548417-781cb71495f3?w=800&q=60",
        True,
    )


@st.dialog("Defect Detail")
def show_defect_modal(defect):
    st.markdown(f"**{defect['id']} — {defect['label'].replace('_', ' ').title()}**")
    sev_color = {"Critical": "#FF5C5C", "Moderate": "#FFC24B", "Minor": "#00E5A0"}.get(defect["severity"], "#888")
    st.markdown(f"Severity: <span style='color:{sev_color};font-weight:700;'>{defect['severity']}</span>",
                unsafe_allow_html=True)
    ts = datetime.fromisoformat(defect["timestamp"]).strftime("%Y-%m-%d %H:%M:%S UTC")
    st.caption(f"Detected: {ts}  ·  Confidence: {defect['confidence']:.0%}")
    st.caption(f"GPS: x={defect['gps']['x']:.1f}, y={defect['gps']['y']:.1f}")
    if defect.get("distance_m") is not None:
        st.caption(f"Distance traveled at detection: {defect['distance_m']:.1f} m")
    img_src, is_placeholder = get_defect_image(defect)
    st.image(
        img_src,
        caption=("No snapshot received from backend — showing placeholder"
                  if is_placeholder else "Captured frame at time of detection"),
    )
    if st.button("Close"):
        st.session_state.selected_defect_id = None
        st.rerun()


if st.session_state.selected_defect_id:
    match = next((d for d in st.session_state.defects if d["id"] == st.session_state.selected_defect_id), None)
    if match:
        show_defect_modal(match)

# --------------------------------------------------------------------------
# Data log table
# --------------------------------------------------------------------------
st.markdown("#### 📋 Defect Log")

if st.session_state.defects:
    rows = []
    for d in reversed(st.session_state.defects):
        thumb_src, _ = get_defect_image(d)
        rows.append({
            "Timestamp": datetime.fromisoformat(d["timestamp"]).strftime("%Y-%m-%d %H:%M:%S"),
            "ID": d["id"],
            "Type": d["label"].replace("_", " ").title(),
            "Severity": d["severity"],
            "Confidence": d["confidence"]*100,
            "Distance (m)": d.get("distance_m"),
            "Thumbnail": thumb_src,
        })
    df = pd.DataFrame(rows)

    st.dataframe(
        df,
        use_container_width=True,
        hide_index=True,
        column_config={
            "Thumbnail": st.column_config.ImageColumn("Thumbnail", width="small"),
            "Confidence": st.column_config.ProgressColumn("Confidence", min_value=0, max_value=100, format="%.0f%%"),
            "Severity": st.column_config.TextColumn("Severity"),
            "Distance (m)": st.column_config.NumberColumn("Distance (m)", format="%.1f m"),
        },
        height=280,
    )
else:
    st.info("No defects logged yet. This table updates automatically as the AI process reports detections.")

st.caption(
    "Frontend is a pure consumer: all telemetry, YOLO boxes, and defect events "
    "originate from the robot/AI process and arrive here over WebSocket only."
)