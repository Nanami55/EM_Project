/*
  ESP8266 Sensor + Pan/Tilt Node
  ================================
  Reads:  DHT22 (temp/humidity), digital hall-effect sensor, MPU6050 (I2C IMU)
  Drives: 2x MG996R servos (pan, tilt)

  Wiring (matches the pin plan already worked out for this board):
    DHT22 data   -> D7  (GPIO13)
    Hall sensor  -> D5  (GPIO14)
    MPU6050 SDA  -> D2  (GPIO4)
    MPU6050 SCL  -> D1  (GPIO5)
    Servo (pan)  -> D6  (GPIO12)
    Servo (tilt) -> D0  (GPIO16)

  Libraries needed (Arduino IDE Library Manager):
    - DHT sensor library (by Adafruit) + Adafruit Unified Sensor
    - Adafruit MPU6050 (pulls in Adafruit BusIO automatically)
    - ESP8266WiFi / ESP8266WebServer / Servo (bundled with the ESP8266
      board package, nothing extra to install)

  FALLBACK BEHAVIOR
  -------------------
  Every sensor is optional and independent:
    - DHT22: dht.readTemperature()/readHumidity() return NaN on a failed
      read (bad wiring, no pull-up, or not connected at all). Detected
      with isnan() and reported as {"connected": false, ...: null}.
    - MPU6050: mpu.begin() returns false at boot if nothing acks on the
      I2C bus. That's latched into mpuConnected and checked every loop —
      the board never blocks waiting for it, and /data reports
      {"connected": false} with no accel/gyro fields.
    - Hall sensor: it's a bare digital GPIO, so there's no handshake to
      detect "not connected" the way there is for DHT22/I2C — an
      unplugged pin just floats. INPUT_PULLUP is used so that an
      unconnected pin reads a stable HIGH (1) instead of noise, which is
      the best you can do in hardware for this sensor type. No
      "connected" flag is reported for it — that's a hardware
      limitation, not an oversight.
  None of the above ever halts setup() or loop() — a missing sensor just
  shows up as false/null in the JSON, and everything else keeps working.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Servo.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- WiFi ----------------
const char* WIFI_SSID = "Faisal";
const char* WIFI_PASS = "faisal09";

// ---------------- Pins ----------------
#define DHT_PIN         13   // D7
#define DHT_TYPE        DHT22
#define HALL_PIN        14   // D5
#define SERVO_PAN_PIN   12   // D6
#define SERVO_TILT_PIN  16   // D0
// MPU6050 uses I2C: SDA=GPIO4 (D2), SCL=GPIO5 (D1) — set explicitly in setup()

// ---------------- Globals ----------------
ESP8266WebServer server(80);
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;
Servo panServo;
Servo tiltServo;

bool mpuConnected = false;
int panAngle = 90;
int tiltAngle = 90;

const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- WiFi ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi NOT connected yet — will keep retrying in the background.");
  }

  // ---- Sensors (each independent; one failing doesn't block the rest) ----
  dht.begin();

  pinMode(HALL_PIN, INPUT_PULLUP);

  Wire.begin(4, 5); // SDA=D2, SCL=D1
  mpuConnected = mpu.begin();
  if (mpuConnected) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050: connected");
  } else {
    Serial.println("MPU6050: NOT found at boot — will report connected:false");
  }

  // ---- Servos ----
  panServo.attach(SERVO_PAN_PIN);
  tiltServo.attach(SERVO_TILT_PIN);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // ---- HTTP routes ----
  server.on("/data", HTTP_GET, handleData);
  server.on("/pan", HTTP_GET, handlePan);
  server.on("/tilt", HTTP_GET, handleTilt);
  server.onNotFound([]() {
    server.send(404, "application/json", "{\"error\":\"not found\"}");
  });
  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  server.handleClient();

  // Quiet WiFi reconnect if it drops — never blocks sensor/servo handling
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 10000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
  }
}

void handleData() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  bool dhtOk = !isnan(temp) && !isnan(hum);
  String tempStr = dhtOk ? String(temp, 1) : String("null");
  String humStr  = dhtOk ? String(hum, 1)  : String("null");

  String json = "{";

  json += "\"dht22\":{";
  json += "\"connected\":" + String(dhtOk ? "true" : "false") + ",";
  json += "\"temperature_c\":" + tempStr + ",";
  json += "\"humidity_pct\":" + humStr;
  json += "},";

  json += "\"hall\":{\"state\":" + String(digitalRead(HALL_PIN)) + "},";

  json += "\"mpu6050\":{\"connected\":" + String(mpuConnected ? "true" : "false");
  if (mpuConnected) {
    sensors_event_t a, g, tempEvt;
    mpu.getEvent(&a, &g, &tempEvt);
    json += ",\"accel\":{\"x\":" + String(a.acceleration.x, 3) +
            ",\"y\":" + String(a.acceleration.y, 3) +
            ",\"z\":" + String(a.acceleration.z, 3) + "}";
    json += ",\"gyro\":{\"x\":" + String(g.gyro.x, 3) +
            ",\"y\":" + String(g.gyro.y, 3) +
            ",\"z\":" + String(g.gyro.z, 3) + "}";
    json += ",\"temp_c\":" + String(tempEvt.temperature, 1);
  }
  json += "},";

  json += "\"servo\":{\"pan_deg\":" + String(panAngle) +
          ",\"tilt_deg\":" + String(tiltAngle) + "},";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";

  server.send(200, "application/json", json);
}

void handlePan()  { handleServo(true); }
void handleTilt() { handleServo(false); }

void handleServo(bool isPan) {
  if (!server.hasArg("angle")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing angle param\"}");
    return;
  }
  int a = constrain(server.arg("angle").toInt(), SERVO_MIN, SERVO_MAX);
  if (isPan) {
    panAngle = a;
    panServo.write(panAngle);
    server.send(200, "application/json", "{\"ok\":true,\"pan_deg\":" + String(panAngle) + "}");
  } else {
    tiltAngle = a;
    tiltServo.write(tiltAngle);
    server.send(200, "application/json", "{\"ok\":true,\"tilt_deg\":" + String(tiltAngle) + "}");
  }
}
