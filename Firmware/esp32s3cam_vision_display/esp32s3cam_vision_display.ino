/*
  esp32s3cam_vision_display.ino
  ================================
  ONE ESP32-S3-CAM board doing TWO independent jobs at the same time:

    1) CAMERA -> PC (unchanged from esp32s3cam_udp_streamer.ino)
       Grabs JPEG frames and fires them as chunked UDP packets to your PC,
       where ugv_vision_system.py reassembles and runs YOLO on them.

    2) PC -> TFT (new)
       Listens on a second UDP port for small JSON "what to show" messages
       and draws them on the attached ILI9341 screen. Your PC script decides
       what appears on the robot's screen — battery, FPS, detections,
       distance sensor, whatever you want — the ESP32 just renders it.

  These two jobs don't know about each other. If the camera stream drops,
  the display keeps working (and vice versa) — useful for debugging, since
  you can tell at a glance which half of the system is actually broken.

  DISPLAY PROTOCOL (JSON over UDP, one packet = one full update)
  ------------------------------------------------------------------
  {
    "title":        "UGV VISION",       // optional, top header text
    "status":       "Tracking 2",       // optional, shown in the status bar
    "status_level": "good",             // optional, one of good/warn/bad/neutral
    "stats": [                          // optional, up to 4 cards, 2x2 grid
      {"label": "FPS",    "value": "24.1", "level": "good"},
      {"label": "OBJECTS","value": "2",    "level": "neutral"}
    ]
  }

  Rules that keep this predictable:
   - Any top-level key you omit is simply left unchanged on screen (e.g.
     send only "status" to update the status bar without touching the grid).
   - "stats" always replaces the WHOLE 2x2 grid (up to 4 entries); fewer
     than 4 entries blanks the remaining cards.
   - "value" should be a pre-formatted STRING ("31.4C", "78%") — the ESP32
     does not do number formatting, your PC script does.
   - "level" controls color: good=green, warn=amber, bad=red,
     neutral/unset=white.

  This file pairs with esp32_display_link.py on the PC side.

  LIBRARIES YOU NEED (Arduino IDE Library Manager):
   - Adafruit GFX Library
   - Adafruit ILI9341
   - ArduinoJson (by Benoit Blanchon) — v6.21+ or v7.x

  BOARD SETTINGS: same as the camera-only sketch — "ESP32S3 Dev Module",
  PSRAM: OPI PSRAM, Partition Scheme: Huge APP.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ---------------------------------------------------------------------------
// WiFi — fill these in
// ---------------------------------------------------------------------------
const char* WIFI_SSID     = "Faisal";
const char* WIFI_PASSWORD = "faisal09";

// ---------------------------------------------------------------------------
// Camera -> PC target (must match ugv_vision_system.py --udp-port)
// ---------------------------------------------------------------------------
IPAddress UDP_TARGET_IP(192, 168, 137, 1);   // <-- CHANGE to your PC's LAN IP
const uint16_t UDP_TARGET_PORT = 5005;

// ---------------------------------------------------------------------------
// PC -> Display listen port (must match esp32_display_link.py --port)
// ---------------------------------------------------------------------------
const uint16_t DISPLAY_LISTEN_PORT = 5006;

// ---------------------------------------------------------------------------
// Camera chunking (unchanged from the camera-only sketch)
// ---------------------------------------------------------------------------
#define UDP_CHUNK_SIZE 1400
#define UDP_HEADER_SIZE 8
const unsigned long FRAME_INTERVAL_MS = 50;  // ~20 fps target

// ---------------------------------------------------------------------------
// ESP32-S3-CAM pin map (OV2640) — unchanged from the camera-only sketch.
// If your board variant is wired differently, fix these first.
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

// ---------------------------------------------------------------------------
// TFT pin map — from your test sketch. Verified to NOT overlap the camera
// pins above. If you're on a different board variant, double-check yours.
// ---------------------------------------------------------------------------
#define TFT_CS   1
#define TFT_DC   42
#define TFT_RST  -1
#define TFT_MOSI 47
#define TFT_SCLK 41
#define TFT_MISO -1

SPIClass mySPI(FSPI);
Adafruit_ILI9341 tft = Adafruit_ILI9341(&mySPI, TFT_DC, TFT_CS, TFT_RST);

// ---- Theme (RGB565) ----
#define BG        0x0841
#define CARD_BG   0x18C3
#define ACCENT    0x07FF
#define WARN      0xFD20
#define GOOD      0x07E0
#define BAD       0xF800
#define TXT       0xFFFF
#define SUBTXT    0x8C71

const int SCR_W = 320, SCR_H = 240;
const int PAD = 10;

// ---------------------------------------------------------------------------
// Networking objects
// ---------------------------------------------------------------------------
WiFiUDP camUdp;       // sends camera frames to the PC
WiFiUDP displayUdp;   // receives display JSON from the PC
uint32_t frameCounter = 0;
unsigned long lastFrameSentAt = 0;
bool cameraOK = false;

char incomingPacket[600];

// ---------------------------------------------------------------------------
// Display state (what's currently on screen)
// ---------------------------------------------------------------------------
struct Rect { int x, y, w, h; };
struct StatCell { String label; String value; uint16_t color; };

String title  = "ESP32-S3-CAM";
String status = "Waiting for data...";
uint16_t statusColor = SUBTXT;
StatCell cells[4] = {
  {"", "--", TXT}, {"", "--", TXT}, {"", "--", TXT}, {"", "--", TXT}
};

bool everReceivedPacket = false;
bool isOnline = false;
unsigned long lastPacketAt = 0;
const unsigned long OFFLINE_TIMEOUT_MS = 5000;
unsigned long lastFreshnessTickAt = 0;
const unsigned long FRESHNESS_TICK_MS = 1000;

const Rect cellRect[4] = {
  {10,  34, 145, 55},
  {163, 34, 145, 55},
  {10,  97, 145, 55},
  {163, 97, 145, 55},
};
const Rect statusRect = {10, 160, 300, 34};

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
String truncate(const String &s, int maxLen) {
  if ((int)s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 1) + ".";
}

uint16_t colorForLevel(const char* level) {
  if (strcmp(level, "good") == 0) return GOOD;
  if (strcmp(level, "warn") == 0) return WARN;
  if (strcmp(level, "bad") == 0)  return BAD;
  return TXT; // "neutral" or unrecognized
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
void drawBootMessage(const String &line1, const String &line2 = "") {
  tft.fillScreen(BG);
  tft.setTextColor(ACCENT, BG);
  tft.setTextSize(2);
  tft.setCursor(PAD, 40);
  tft.println("ESP32-S3-CAM");
  tft.setTextSize(1);
  tft.setTextColor(SUBTXT, BG);
  tft.setCursor(PAD, 80);
  tft.println(line1);
  if (line2.length()) {
    tft.setCursor(PAD, 96);
    tft.println(line2);
  }
}

void drawTitleBar() {
  tft.fillRect(0, 0, SCR_W, 30, BG);
  tft.setTextColor(ACCENT, BG);
  tft.setTextSize(2);
  tft.setCursor(PAD, 6);
  tft.print(truncate(title, 20));

  uint16_t dotColor = isOnline ? GOOD : (everReceivedPacket ? BAD : SUBTXT);
  tft.fillCircle(SCR_W - 16, 14, 6, dotColor);

  tft.drawFastHLine(0, 28, SCR_W, 0x2965);
}

void drawCellBackgrounds() {
  for (int i = 0; i < 4; i++) {
    tft.fillRoundRect(cellRect[i].x, cellRect[i].y, cellRect[i].w, cellRect[i].h, 8, CARD_BG);
  }
}

void drawCell(int i) {
  Rect r = cellRect[i];
  tft.fillRect(r.x + 8, r.y + 6, r.w - 16, 12, CARD_BG);
  tft.setTextColor(SUBTXT, CARD_BG);
  tft.setTextSize(1);
  tft.setCursor(r.x + 8, r.y + 6);
  tft.print(cells[i].label);

  tft.fillRect(r.x + 8, r.y + 22, r.w - 16, 26, CARD_BG);
  tft.setTextColor(cells[i].color, CARD_BG);
  tft.setTextSize(2);
  tft.setCursor(r.x + 8, r.y + 24);
  tft.print(cells[i].value);
}

void drawStatusBackground() {
  tft.fillRoundRect(statusRect.x, statusRect.y, statusRect.w, statusRect.h, 8, CARD_BG);
}

void drawStatus() {
  drawStatusBackground();
  tft.setTextColor(SUBTXT, CARD_BG);
  tft.setTextSize(1);
  tft.setCursor(statusRect.x + 10, statusRect.y + 6);
  tft.print("STATUS");
  tft.setTextColor(statusColor, CARD_BG);
  tft.setCursor(statusRect.x + 10, statusRect.y + 18);
  tft.print(truncate(status, 46));
}

void drawFreshness(unsigned long now) {
  tft.fillRect(0, 204, SCR_W, 14, BG);
  tft.setTextSize(1);
  tft.setCursor(PAD, 205);
  if (!everReceivedPacket) {
    tft.setTextColor(SUBTXT, BG);
    tft.print("Waiting for first update from PC...");
    return;
  }
  unsigned long ageSec = (now - lastPacketAt) / 1000;
  if ((now - lastPacketAt) >= OFFLINE_TIMEOUT_MS) {
    tft.setTextColor(BAD, BG);
    tft.print("OFFLINE - no data for ");
    tft.print(ageSec);
    tft.print("s");
  } else {
    tft.setTextColor(SUBTXT, BG);
    tft.print("Updated ");
    tft.print(ageSec);
    tft.print("s ago");
  }
}

void drawStaticFrame() {
  tft.fillScreen(BG);
  drawTitleBar();
  drawCellBackgrounds();
  for (int i = 0; i < 4; i++) drawCell(i);
  drawStatus();
  drawFreshness(millis());
}

// ---------------------------------------------------------------------------
// Handle one incoming display JSON packet (non-blocking, called every loop)
// ---------------------------------------------------------------------------
void checkForDisplayPacket() {
  int packetSize = displayUdp.parsePacket();
  if (packetSize <= 0) return;

  int len = displayUdp.read(incomingPacket, sizeof(incomingPacket) - 1);
  if (len <= 0) return;
  incomingPacket[len] = '\0';

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, incomingPacket, len);
  if (err) {
    Serial.print("[display] JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  bool wasOnline = isOnline;
  everReceivedPacket = true;
  isOnline = true;
  lastPacketAt = millis();

  bool titleChanged = false;
  const char* newTitleC = doc["title"] | "";
  if (newTitleC[0] != '\0') {
    String newTitle = String(newTitleC);
    if (newTitle != title) { title = newTitle; titleChanged = true; }
  }

  bool statusChanged = false;
  const char* newStatusC = doc["status"] | "";
  if (newStatusC[0] != '\0') {
    String newStatus = String(newStatusC);
    uint16_t newColor = colorForLevel(doc["status_level"] | "neutral");
    if (newStatus != status || newColor != statusColor) {
      status = newStatus;
      statusColor = newColor;
      statusChanged = true;
    }
  }

  // "stats" is only applied if the key is actually present, so you can
  // update just the status line (e.g. "Rebooting...") without wiping the grid.
  if (doc["stats"].is<JsonArray>()) {
    JsonArray stats = doc["stats"];
    for (int i = 0; i < 4; i++) {
      String newLabel = "";
      String newValue = "--";
      uint16_t newColor = TXT;
      if (i < (int)stats.size()) {
        JsonObject s = stats[i];
        newLabel = truncate(String((const char*)(s["label"] | "")), 12);
        newValue = truncate(String((const char*)(s["value"] | "--")), 10);
        newColor = colorForLevel(s["level"] | "neutral");
      }
      if (newLabel != cells[i].label || newValue != cells[i].value || newColor != cells[i].color) {
        cells[i].label = newLabel;
        cells[i].value = newValue;
        cells[i].color = newColor;
        drawCell(i);
      }
    }
  }

  if (titleChanged || wasOnline != isOnline) drawTitleBar();
  if (statusChanged) drawStatus();
}

// ---------------------------------------------------------------------------
// Camera init (unchanged logic from the camera-only sketch)
// ---------------------------------------------------------------------------
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk    = XCLK_GPIO_NUM;
  config.pin_pclk    = PCLK_GPIO_NUM;
  config.pin_vsync   = VSYNC_GPIO_NUM;
  config.pin_href    = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn    = PWDN_GPIO_NUM;
  config.pin_reset   = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;    // 640x480
    config.jpeg_quality = 15;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;   // 320x240
    config.jpeg_quality = 18;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 1);
    s->set_whitebal(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_vflip(s, 1);     // most S3-CAM boards mount the sensor upside-down
    // s->set_hmirror(s, 1); // uncomment too if the image is ALSO mirrored
  }
  return true;
}

void sendFrameOverUDP(camera_fb_t* fb) {
  uint16_t totalChunks = (fb->len + UDP_CHUNK_SIZE - 1) / UDP_CHUNK_SIZE;
  uint8_t packet[UDP_HEADER_SIZE + UDP_CHUNK_SIZE];

  for (uint16_t i = 0; i < totalChunks; i++) {
    size_t offset = i * UDP_CHUNK_SIZE;
    size_t chunkLen = min((size_t)UDP_CHUNK_SIZE, fb->len - offset);

    memcpy(packet + 0, &frameCounter, 4);
    memcpy(packet + 4, &i, 2);
    memcpy(packet + 6, &totalChunks, 2);
    memcpy(packet + UDP_HEADER_SIZE, fb->buf + offset, chunkLen);

    camUdp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    camUdp.write(packet, UDP_HEADER_SIZE + chunkLen);
    camUdp.endPacket();
  }
  frameCounter++;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  mySPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(1);

  drawBootMessage("Starting camera...");
  cameraOK = initCamera();
  if (!cameraOK) {
    drawBootMessage("CAMERA INIT FAILED", "Check wiring / board select.");
    Serial.println("Camera init failed - continuing without it.");
    Serial.println("The display will still work once WiFi connects.");
  }

  drawBootMessage("Connecting to WiFi...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. ESP32 IP: ");
    Serial.println(WiFi.localIP());
    drawBootMessage("WiFi connected!", WiFi.localIP().toString());
    delay(1200);
  } else {
    Serial.println("WiFi not connected yet - will keep retrying in the background.");
    drawBootMessage("WiFi not connected yet", "Will keep retrying...");
    delay(1200);
  }

  camUdp.begin(UDP_TARGET_PORT);
  displayUdp.begin(DISPLAY_LISTEN_PORT);

  drawStaticFrame();
  lastFreshnessTickAt = millis();
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1) Camera frame -> PC (rate-limited, skipped entirely if camera failed)
  if (cameraOK && now - lastFrameSentAt >= FRAME_INTERVAL_MS) {
    lastFrameSentAt = now;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      sendFrameOverUDP(fb);
      esp_camera_fb_return(fb);
    }
  }

  // 2) PC -> Display (non-blocking check every loop)
  checkForDisplayPacket();

  // 3) "Xs ago" ticker + online/offline flip, once a second
  if (now - lastFreshnessTickAt >= FRESHNESS_TICK_MS) {
    lastFreshnessTickAt = now;
    bool wasOnline = isOnline;
    if (everReceivedPacket && (now - lastPacketAt) >= OFFLINE_TIMEOUT_MS) {
      isOnline = false;
    }
    if (wasOnline != isOnline) drawTitleBar();
    drawFreshness(now);
  }

  // 4) Keep WiFi alive without ever hard-halting the board
  static unsigned long lastReconnectAttempt = 0;
  if (WiFi.status() != WL_CONNECTED && now - lastReconnectAttempt > 10000) {
    lastReconnectAttempt = now;
    Serial.println("WiFi dropped - attempting reconnect...");
    WiFi.reconnect();
  }
}
