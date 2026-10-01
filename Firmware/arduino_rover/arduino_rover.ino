/*
  arduino_rover.ino
  ==================
  Control-unit firmware for the defect-inspection UGV.

  RESPONSIBILITIES (and only these — keep this file dumb and reliable):
    1. Drive 4 motors via 2x BTS7960 (differential/skid-steer).
    2. Receive short text commands from the server over HC-05 Bluetooth.
    3. Own the STOP -> wait -> auto-RESUME timing for defect events, so this
       keeps working even if the Bluetooth link or the server hiccups.
    4. Monitor battery voltage and sound the buzzer on low voltage.
    5. Report battery + state back to the server once a second.
    6. Light the onboard LED whenever motor PWM is active (pins 5,6,9,10).

  It does NOT decide what a "defect" is — that's the server's job. This
  file only reacts to the STOP / ESTOP / RESUME / M: / D: commands it
  receives.

  --------------------------------------------------------------------------
  SERIAL COMMAND PROTOCOL (server -> Arduino, one line each, newline-terminated)
  --------------------------------------------------------------------------
    STOP          Defect detected by the server. Stop immediately, then
                  auto-resume forward driving after DEFECT_PAUSE_MS, but
                  only if mode is AUTO. Ignored while already paused.
    ESTOP         Hard stop (dashboard Emergency Stop). Latches — the robot
                  will NOT auto-resume. Needs an explicit RESUME command.
    RESUME        Clears an ESTOP or a stuck pause and returns to normal
                  driving for the current mode.
    M:AUTO        Switch to autonomous patrol mode (drives forward by
                  itself, subject to STOP/ESTOP).
    M:MANUAL      Switch to manual/steering mode. Motors stop immediately
                  and wait for D: commands.
    D:F / D:B     Manual drive: forward / backward. Only obeyed in MANUAL
    D:L / D:R     mode. Ignored otherwise.
    D:S
    PING          Health check -> Arduino replies "PONG"

  --------------------------------------------------------------------------
  TELEMETRY (Arduino -> server, once a second)
  --------------------------------------------------------------------------
    BATT:<volts>       e.g. "BATT:11.42"
    STATE:<state>      one of PATROL / DEFECT_PAUSE / ESTOPPED / MANUAL
*/

#include <SoftwareSerial.h>

// ---------------------------------------------------------------------------
// Pin map — matches the wiring walkthrough
// ---------------------------------------------------------------------------
const uint8_t PIN_LEFT_RPWM   = 5;   // BTS7960 #1 (left side motors)
const uint8_t PIN_LEFT_LPWM   = 6;
const uint8_t PIN_RIGHT_RPWM  = 9;   // BTS7960 #2 (right side motors)
const uint8_t PIN_RIGHT_LPWM  = 10;

const uint8_t PIN_BT_RX       = 2;   // <- HC-05 TXD
const uint8_t PIN_BT_TX       = 3;   // -> HC-05 RXD (through voltage divider!)

const uint8_t PIN_BUZZER      = 4;
const uint8_t PIN_BATT_SENSE  = A0;

// Onboard LED — lights up whenever left/right motor PWM is nonzero.
// NOTE: on classic AVR boards (Uno/Nano/Mega) LED_BUILTIN is active-HIGH
// (HIGH = lit). If you move this firmware to an ESP8266/ESP32 board,
// LED_BUILTIN is often active-LOW (lit on LOW) — flip the logic in
// updateMotorLED() below if the LED behaves backwards on your board.
const uint8_t PIN_LED         = LED_BUILTIN;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
const int      DRIVE_SPEED         = 180;   // 0-255 PWM, patrol/manual speed
const int      TURN_SPEED          = 160;
const unsigned long DEFECT_PAUSE_MS = 4000;  // how long to sit at a logged defect
const unsigned long TELEMETRY_MS    = 1000;  // telemetry push interval

// Voltage divider: battery+ -> R1 -> A0 -> R2 -> GND. With R1=10k, R2=3.9k,
// the divider ratio is (R1+R2)/R2. Adjust these two if your resistors differ.
const float DIVIDER_R1        = 10000.0;
const float DIVIDER_R2        = 3900.0;
const float ADC_REF_VOLTAGE   = 5.0;
const float LOW_BATTERY_V     = 10.8;   // ~3.6V/cell for a 3S pack, tune to taste
const unsigned long BATT_CHECK_MS = 2000;
const unsigned long BUZZER_BEEP_MS = 150;
const unsigned long BUZZER_GAP_MS  = 850;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum Mode { MODE_AUTO, MODE_MANUAL };
enum RunState { STATE_PATROL, STATE_DEFECT_PAUSE, STATE_ESTOPPED, STATE_MANUAL_IDLE, STATE_MANUAL_DRIVING };

// SAFETY DEFAULT: the robot boots into MANUAL / idle, motors off. It will
// NOT start patrolling on its own just because it has power — it only
// starts driving once the dashboard explicitly sends M:AUTO. This matters
// because HC-05 pairing takes a few seconds after power-on; defaulting to
// AUTO/PATROL here would mean the robot drives off before Bluetooth is
// even connected, with no way to stop it in that window.
Mode currentMode = MODE_MANUAL;
RunState currentState = STATE_MANUAL_IDLE;

unsigned long pauseStartedAt = 0;
unsigned long lastTelemetryAt = 0;
unsigned long lastBattCheckAt = 0;
unsigned long buzzerToggledAt = 0;
bool buzzerOn = false;
bool lowBatteryLatched = false;

// Tracks the last commanded PWM for each side so the LED indicator can
// tell "motors active" from "motors stopped" without re-reading pins.
int lastLeftSpeed  = 0;
int lastRightSpeed = 0;

SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);

// ---------------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------------
void setMotor(uint8_t rpwmPin, uint8_t lpwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  }
}

// Reflects current motor activity on the onboard LED. Called any time
// lastLeftSpeed/lastRightSpeed change (i.e. from setLeft/setRight), so
// every code path that drives pins 5/6/9/10 keeps the LED in sync
// automatically — no need to touch the LED anywhere else in the file.
void updateMotorLED() {
  bool active = (lastLeftSpeed != 0) || (lastRightSpeed != 0);
  digitalWrite(PIN_LED, active ? HIGH : LOW);
}

void setLeft(int speed) {
  lastLeftSpeed = constrain(speed, -255, 255);
  setMotor(PIN_LEFT_RPWM, PIN_LEFT_LPWM, speed);
  updateMotorLED();
}

void setRight(int speed) {
  lastRightSpeed = constrain(speed, -255, 255);
  setMotor(PIN_RIGHT_RPWM, PIN_RIGHT_LPWM, speed);
  updateMotorLED();
}

void motorsStop()     { setLeft(0); setRight(0); }
void motorsForward()  { setLeft(DRIVE_SPEED);  setRight(DRIVE_SPEED); }
void motorsBackward() { setLeft(-DRIVE_SPEED); setRight(-DRIVE_SPEED); }
void motorsLeft()     { setLeft(-TURN_SPEED);  setRight(TURN_SPEED); }
void motorsRight()    { setLeft(TURN_SPEED);   setRight(-TURN_SPEED); }

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------
void enterDefectPause() {
  if (currentState == STATE_DEFECT_PAUSE || currentState == STATE_ESTOPPED) return; // already stopped
  motorsStop();
  currentState = STATE_DEFECT_PAUSE;
  pauseStartedAt = millis();
}

void enterEstop() {
  motorsStop();
  currentState = STATE_ESTOPPED;
}

void resumeFromPauseOrEstop() {
  if (currentMode == MODE_AUTO) {
    currentState = STATE_PATROL;
  } else {
    currentState = STATE_MANUAL_IDLE;
    motorsStop();
  }
}

void setMode(Mode m) {
  currentMode = m;
  motorsStop();
  currentState = (m == MODE_AUTO) ? STATE_PATROL : STATE_MANUAL_IDLE;
}

void handleManualDrive(char dir) {
  if (currentMode != MODE_MANUAL) return;          // ignore drive cmds outside manual mode
  if (currentState == STATE_ESTOPPED) return;       // ignore while hard-stopped
  currentState = STATE_MANUAL_DRIVING;
  switch (dir) {
    case 'F': motorsForward();  break;
    case 'B': motorsBackward(); break;
    case 'L': motorsLeft();     break;
    case 'R': motorsRight();    break;
    case 'S': motorsStop(); currentState = STATE_MANUAL_IDLE; break;
  }
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "STOP") {
    enterDefectPause();
  } else if (line == "ESTOP") {
    enterEstop();
  } else if (line == "RESUME") {
    resumeFromPauseOrEstop();
  } else if (line == "PING") {
    btSerial.println("PONG");
  } else if (line == "M:AUTO") {
    setMode(MODE_AUTO);
  } else if (line == "M:MANUAL") {
    setMode(MODE_MANUAL);
  } else if (line.startsWith("D:") && line.length() == 3) {
    handleManualDrive(line.charAt(2));
  }
  // Unknown commands are silently ignored — keeps this forward-compatible
  // with new server features without needing a firmware update every time.
}

// ---------------------------------------------------------------------------
// Battery + buzzer (non-blocking)
// ---------------------------------------------------------------------------
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATT_SENSE);
  float vAtPin = (raw / 1023.0) * ADC_REF_VOLTAGE;
  float ratio = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2;
  return vAtPin * ratio;
}

void serviceBuzzer(unsigned long now) {
  if (!lowBatteryLatched) {
    if (buzzerOn) { digitalWrite(PIN_BUZZER, LOW); buzzerOn = false; }
    return;
  }
  unsigned long elapsed = now - buzzerToggledAt;
  if (buzzerOn && elapsed >= BUZZER_BEEP_MS) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOn = false;
    buzzerToggledAt = now;
  } else if (!buzzerOn && elapsed >= BUZZER_GAP_MS) {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerOn = true;
    buzzerToggledAt = now;
  }
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
const char* stateName() {
  switch (currentState) {
    case STATE_PATROL:         return "PATROL";
    case STATE_DEFECT_PAUSE:   return "DEFECT_PAUSE";
    case STATE_ESTOPPED:       return "ESTOPPED";
    case STATE_MANUAL_IDLE:    return "MANUAL";
    case STATE_MANUAL_DRIVING: return "MANUAL";
    default:                   return "UNKNOWN";
  }
}

void sendTelemetry(float battV) {
  btSerial.print("BATT:");
  btSerial.println(battV, 2);
  btSerial.print("STATE:");
  btSerial.println(stateName());
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_LEFT_RPWM, OUTPUT);
  pinMode(PIN_LEFT_LPWM, OUTPUT);
  pinMode(PIN_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_RIGHT_LPWM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  motorsStop();

  Serial.begin(9600);        // USB — debugging only
  btSerial.begin(9600);      // HC-05 default baud

  Serial.println("Rover firmware ready.");
}

void loop() {
  unsigned long now = millis();

  // ---- read one command line from Bluetooth, if available -------------
  static String inbound = "";
  while (btSerial.available()) {
    char c = btSerial.read();
    if (c == '\n') {
      handleLine(inbound);
      inbound = "";
    } else if (c != '\r') {
      inbound += c;
    }
  }

  // ---- auto-resume after a defect pause --------------------------------
  if (currentState == STATE_DEFECT_PAUSE && (now - pauseStartedAt >= DEFECT_PAUSE_MS)) {
    resumeFromPauseOrEstop();
  }

  // ---- drive motors according to current state --------------------------
  if (currentState == STATE_PATROL) {
    motorsForward();
  }
  // DEFECT_PAUSE / ESTOPPED / MANUAL_IDLE: motors already stopped by the
  // handler that entered that state — nothing to do here every loop.

  // ---- battery check + buzzer -------------------------------------------
  if (now - lastBattCheckAt >= BATT_CHECK_MS) {
    lastBattCheckAt = now;
    float v = readBatteryVoltage();
    lowBatteryLatched = (v < LOW_BATTERY_V);
  }
  serviceBuzzer(now);

  // ---- telemetry ----------------------------------------------------------
  if (now - lastTelemetryAt >= TELEMETRY_MS) {
    lastTelemetryAt = now;
    sendTelemetry(readBatteryVoltage());
  }
}