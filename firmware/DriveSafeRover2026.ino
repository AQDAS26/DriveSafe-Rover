/*
  =====================================================================
   DRIVESAFE ROVER 2026
   Autonomous Smart Road Safety Rover - Main Controller (ESP32 Dev Module)
  =====================================================================
   Features:
     - Differential drive (2x L298N motor drivers)
     - Front obstacle avoidance (ultrasonic + servo scan)
     - Pothole detection (second ultrasonic sensor)
     - GPS tracking (TinyGPS++)
     - Headlight / Taillight / Hazard lights
     - Manual + Automatic drive modes
     - WiFi Access Point + AJAX dashboard (no page reloads)
     - ESP32-CAM handles video streaming independently (power only,
       no UART link to this board)

   NOTE ON STRAPPING PINS:
     GPIO2 and GPIO5 are ESP32 boot-strapping pins. They are used here
     as requested (IN4 / ENB) - this is safe as long as nothing external
     pulls them LOW/HIGH at power-on/reset. If you see random reboots
     tied to motor wiring, move these two lines to spare GPIOs
     (e.g. 33 for GPIO2, 25 for GPIO5).
  =====================================================================
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <ArduinoJson.h>

// =====================================================================
// WIFI ACCESS POINT CONFIG
// =====================================================================
const char *AP_SSID     = "DriveSafe_Rover";
const char *AP_PASSWORD = "12345678";

AsyncWebServer server(80);

// =====================================================================
// GPIO PIN DEFINITIONS
// =====================================================================
// ---- Left Motor Driver (L298N #1) ----
#define ENA_PIN   18   // PWM - Left side speed
#define IN1_PIN   19   // Left forward
#define IN2_PIN   23   // Left backward

// ---- Right Motor Driver (L298N #2) ----
#define ENB_PIN   5    // PWM - Right side speed (strapping pin, see note above)
#define IN3_PIN   4    // Right forward
#define IN4_PIN   2    // Right backward (strapping pin, see note above)

// ---- Camera Servo ----
#define SERVO_PIN 13

// ---- Ultrasonic - Obstacle (front) ----
#define TRIG_PIN         26
#define ECHO_OBSTACLE_PIN 27

// ---- Ultrasonic - Pothole ----
// Shares the same TRIG pin as the obstacle sensor, separate ECHO pin
#define ECHO_POTHOLE_PIN 14

// ---- GPS (NEO-6M etc.) ----
#define GPS_RX_PIN 16   // ESP32 RX2  <- GPS TX
#define GPS_TX_PIN 17   // ESP32 TX2  -> GPS RX

// ---- Lights ----
#define HEADLIGHT_PIN 21
#define TAILLIGHT_PIN 22
#define HAZARD_PIN    32

// =====================================================================
// CONSTANTS
// =====================================================================
#define MAX_DISTANCE_CM     300     // NewPing max range
#define OBSTACLE_THRESHOLD  30      // cm - stop & scan if closer than this
#define POTHOLE_THRESHOLD   15      // cm - deeper reading than this = pothole
#define DEFAULT_SPEED       200     // 0-255 PWM
#define SLOW_SPEED          110     // used while crossing a pothole
#define TURN_DURATION_MS    600     // how long to turn after deciding direction
#define SCAN_SETTLE_MS      400     // servo settle time per scan position
#define HAZARD_BLINK_MS     400
#define ULTRASONIC_INTERVAL_MS 100
#define GPS_READ_INTERVAL_MS   50
#define STATUS_BROADCAST_MS    500
#define WIFI_CHECK_INTERVAL_MS 2000

// =====================================================================
// GLOBAL OBJECTS
// =====================================================================
Servo camServo;
NewPing sonarObstacle(TRIG_PIN, ECHO_OBSTACLE_PIN, MAX_DISTANCE_CM);
NewPing sonarPothole(TRIG_PIN, ECHO_POTHOLE_PIN, MAX_DISTANCE_CM);

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// =====================================================================
// STATE VARIABLES
// =====================================================================
enum DriveMode { MODE_MANUAL, MODE_AUTO };
DriveMode currentMode = MODE_MANUAL;

String motorStatus = "STOPPED";
int currentSpeed   = DEFAULT_SPEED;
int servoCurrentAngle = 90;

bool headlightState = false;
bool taillightState = false;
bool hazardState    = false;
bool hazardBlinkOn  = false;

long obstacleDistance = MAX_DISTANCE_CM;
long potholeDistance  = MAX_DISTANCE_CM;

// Timers (millis based - no delay() in main loop)
unsigned long lastUltrasonicRead = 0;
unsigned long lastGpsRead        = 0;
unsigned long lastStatusBroadcast = 0;
unsigned long lastHazardBlink    = 0;
unsigned long lastWifiCheck      = 0;

// Automatic mode state machine
enum AutoState {
  AUTO_FORWARD,
  AUTO_STOPPED,
  AUTO_SCAN_LEFT,
  AUTO_SCAN_CENTER,
  AUTO_SCAN_RIGHT,
  AUTO_DECIDE,
  AUTO_TURNING,
  AUTO_POTHOLE_SLOW
};
AutoState autoState = AUTO_FORWARD;
unsigned long autoStateTimer = 0;
long distLeft = 0, distCenter = 0, distRight = 0;

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================
void forward();
void backward();
void left();
void right();
void stopMotors();
void setSpeed(int speed);

void headlightOn();  void headlightOff();
void tailLightOn();  void tailLightOff();
void hazardOn();     void hazardOff();
void hazardBlink();

void servoLeft();  void servoCenter();  void servoRight();
void servoAngle(int angle);

void readUltrasonicSensors();
void readGPS();
void automaticModeLogic();
void broadcastStatusIfDue();
String buildStatusJSON();
void setupWebServer();

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  // ESP32 core 3.x supports analogWrite() natively (PWM via LEDC under the hood)
  analogWriteResolution(ENA_PIN, 8);
  analogWriteResolution(ENB_PIN, 8);
  stopMotors();

  // Light pins
  pinMode(HEADLIGHT_PIN, OUTPUT);
  pinMode(TAILLIGHT_PIN, OUTPUT);
  pinMode(HAZARD_PIN, OUTPUT);
  headlightOff();
  tailLightOff();
  hazardOff();

  // Servo
  camServo.setPeriodHertz(50);
  camServo.attach(SERVO_PIN, 500, 2400);
  servoCenter();

  // GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  setupWebServer();
  server.begin();

  Serial.println("DriveSafe Rover 2026 - Ready.");
}

// =====================================================================
// MAIN LOOP - fully non-blocking (millis based)
// =====================================================================
void loop() {
  unsigned long now = millis();

  // ---- Ultrasonic sensors ----
  if (now - lastUltrasonicRead >= ULTRASONIC_INTERVAL_MS) {
    lastUltrasonicRead = now;
    readUltrasonicSensors();
  }

  // ---- GPS ----
  if (now - lastGpsRead >= GPS_READ_INTERVAL_MS) {
    lastGpsRead = now;
    readGPS();
  }

  // ---- Hazard blink (runs independently of hazard mode source) ----
  if (hazardState && (now - lastHazardBlink >= HAZARD_BLINK_MS)) {
    lastHazardBlink = now;
    hazardBlinkOn = !hazardBlinkOn;
    digitalWrite(HAZARD_PIN, hazardBlinkOn ? HIGH : LOW);
  }

  // ---- Safety: WiFi disconnect check (AP mode -> check connected clients) ----
  if (now - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheck = now;
    if (WiFi.softAPgetStationNum() == 0 && currentMode == MODE_MANUAL) {
      // No dashboard connected in manual mode -> stop for safety
      stopMotors();
    }
  }

  // ---- Automatic mode ----
  if (currentMode == MODE_AUTO) {
    automaticModeLogic();
  }

  // ---- Push status periodically (dashboard also polls /status via AJAX) ----
  broadcastStatusIfDue();
}

// =====================================================================
// MOTOR FUNCTIONS
// =====================================================================
void setSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  currentSpeed = speed;
  analogWrite(ENA_PIN, currentSpeed);
  analogWrite(ENB_PIN, currentSpeed);
}

void forward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, currentSpeed);
  analogWrite(ENB_PIN, currentSpeed);
  motorStatus = "FORWARD";
}

void backward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENA_PIN, currentSpeed);
  analogWrite(ENB_PIN, currentSpeed);
  motorStatus = "BACKWARD";
}

void left() {
  // Left side reverse, right side forward -> turn left in place
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, currentSpeed);
  analogWrite(ENB_PIN, currentSpeed);
  motorStatus = "LEFT";
}

void right() {
  // Left side forward, right side reverse -> turn right in place
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENA_PIN, currentSpeed);
  analogWrite(ENB_PIN, currentSpeed);
  motorStatus = "RIGHT";
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  motorStatus = "STOPPED";
}

// =====================================================================
// LIGHT FUNCTIONS
// =====================================================================
void headlightOn()  { headlightState = true;  digitalWrite(HEADLIGHT_PIN, HIGH); }
void headlightOff() { headlightState = false; digitalWrite(HEADLIGHT_PIN, LOW);  }

void tailLightOn()  { taillightState = true;  digitalWrite(TAILLIGHT_PIN, HIGH); }
void tailLightOff() { taillightState = false; digitalWrite(TAILLIGHT_PIN, LOW);  }

void hazardOn() {
  hazardState = true;
  lastHazardBlink = millis();
}
void hazardOff() {
  hazardState = false;
  hazardBlinkOn = false;
  digitalWrite(HAZARD_PIN, LOW);
}
// hazardBlink() kept for API completeness - actual blinking is handled
// non-blockingly inside loop() using millis()
void hazardBlink() { hazardOn(); }

// =====================================================================
// SERVO FUNCTIONS
// =====================================================================
void servoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servoCurrentAngle = angle;
  camServo.write(angle);
}
void servoLeft()   { servoAngle(30);  }
void servoCenter() { servoAngle(90);  }
void servoRight()  { servoAngle(150); }

// =====================================================================
// ULTRASONIC SENSORS
// =====================================================================
void readUltrasonicSensors() {
  unsigned int obs = sonarObstacle.ping_cm();
  unsigned int pot = sonarPothole.ping_cm();

  // ping_cm() returns 0 on timeout / out-of-range -> treat as "max distance"
  obstacleDistance = (obs == 0) ? MAX_DISTANCE_CM : obs;
  potholeDistance  = (pot == 0) ? MAX_DISTANCE_CM : pot;

  // Manual-mode safety stop on obstacle
  if (currentMode == MODE_MANUAL && obstacleDistance < OBSTACLE_THRESHOLD &&
      (motorStatus == "FORWARD")) {
    stopMotors();
  }
}

// =====================================================================
// GPS
// =====================================================================
void readGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// =====================================================================
// AUTOMATIC MODE - non-blocking obstacle scan & avoidance state machine
// =====================================================================
void automaticModeLogic() {
  unsigned long now = millis();

  // ---- Pothole handling takes priority ----
  if (potholeDistance > POTHOLE_THRESHOLD && autoState == AUTO_FORWARD) {
    stopMotors();
    hazardOn();
    autoState = AUTO_POTHOLE_SLOW;
    autoStateTimer = now;
    return;
  }

  switch (autoState) {

    case AUTO_FORWARD:
      setSpeed(DEFAULT_SPEED);
      forward();
      if (obstacleDistance < OBSTACLE_THRESHOLD) {
        stopMotors();
        servoAngle(30);
        autoState = AUTO_SCAN_LEFT;
        autoStateTimer = now;
      }
      break;

    case AUTO_SCAN_LEFT:
      if (now - autoStateTimer >= SCAN_SETTLE_MS) {
        distLeft = obstacleDistance;
        servoAngle(90);
        autoState = AUTO_SCAN_CENTER;
        autoStateTimer = now;
      }
      break;

    case AUTO_SCAN_CENTER:
      if (now - autoStateTimer >= SCAN_SETTLE_MS) {
        distCenter = obstacleDistance;
        servoAngle(150);
        autoState = AUTO_SCAN_RIGHT;
        autoStateTimer = now;
      }
      break;

    case AUTO_SCAN_RIGHT:
      if (now - autoStateTimer >= SCAN_SETTLE_MS) {
        distRight = obstacleDistance;
        autoState = AUTO_DECIDE;
      }
      break;

    case AUTO_DECIDE:
      servoCenter();
      // Choose direction with maximum measured distance
      if (distLeft >= distCenter && distLeft >= distRight) {
        left();
      } else if (distRight >= distCenter && distRight >= distLeft) {
        right();
      } else {
        // Center is clearest - reverse briefly then re-scan
        backward();
      }
      autoState = AUTO_TURNING;
      autoStateTimer = now;
      break;

    case AUTO_TURNING:
      if (now - autoStateTimer >= TURN_DURATION_MS) {
        stopMotors();
        autoState = AUTO_FORWARD;
      }
      break;

    case AUTO_POTHOLE_SLOW:
      setSpeed(SLOW_SPEED);
      forward();
      if (now - autoStateTimer >= 1500) {
        hazardOff();
        autoState = AUTO_FORWARD;
      }
      break;

    default:
      autoState = AUTO_FORWARD;
      break;
  }
}

// =====================================================================
// STATUS JSON (used by /status endpoint and periodic broadcast)
// =====================================================================
String buildStatusJSON() {
  StaticJsonDocument<512> doc;

  doc["latitude"]   = gps.location.isValid() ? gps.location.lat() : 0.0;
  doc["longitude"]  = gps.location.isValid() ? gps.location.lng() : 0.0;
  doc["satellites"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["speed"]      = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  doc["gpsFix"]     = gps.location.isValid();

  doc["obstacleDistance"] = obstacleDistance;
  doc["potholeDistance"]  = potholeDistance;
  doc["servoAngle"]       = servoCurrentAngle;

  doc["mode"]        = (currentMode == MODE_MANUAL) ? "MANUAL" : "AUTOMATIC";
  doc["motorStatus"]  = motorStatus;
  doc["speedValue"]   = currentSpeed;

  doc["headlight"] = headlightState;
  doc["taillight"] = taillightState;
  doc["hazard"]    = hazardState;

  String output;
  serializeJson(doc, output);
  return output;
}

void broadcastStatusIfDue() {
  unsigned long now = millis();
  if (now - lastStatusBroadcast >= STATUS_BROADCAST_MS) {
    lastStatusBroadcast = now;
    // Status is pulled by the dashboard via AJAX (/status) - this timer
    // is reserved for future WebSocket/event-source push if needed.
  }
}

// =====================================================================
// EMBEDDED DASHBOARD (dark theme, responsive, AJAX - no page reloads)
// =====================================================================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>DriveSafe Rover 2026</title>
<style>
  :root{
    --bg:#0d1117; --card:#161b22; --border:#30363d;
    --text:#e6edf3; --muted:#8b949e;
    --accent:#2ea043; --danger:#da3633; --warn:#d29922; --blue:#1f6feb;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; font-family:'Segoe UI',Roboto,Arial,sans-serif;
    background:var(--bg); color:var(--text); padding:16px;
  }
  h1{text-align:center; font-size:1.4rem; margin-bottom:4px;}
  .sub{text-align:center; color:var(--muted); margin-bottom:18px; font-size:0.85rem;}
  .grid{
    display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr));
    gap:14px; max-width:900px; margin:0 auto;
  }
  .card{
    background:var(--card); border:1px solid var(--border); border-radius:12px;
    padding:16px;
  }
  .card h2{margin:0 0 10px 0; font-size:0.95rem; color:var(--muted); text-transform:uppercase; letter-spacing:.05em;}
  .row{display:flex; justify-content:space-between; padding:4px 0; font-size:0.92rem; border-bottom:1px solid #21262d;}
  .row:last-child{border-bottom:none;}
  .row span:last-child{font-weight:600;}
  .badge{padding:2px 8px; border-radius:20px; font-size:0.75rem; font-weight:700;}
  .badge.on{background:var(--accent); color:#fff;}
  .badge.off{background:#30363d; color:var(--muted);}
  .badge.mode{background:var(--blue); color:#fff;}
  .btn-grid{display:grid; grid-template-columns:repeat(3,1fr); gap:8px;}
  button{
    background:#21262d; color:var(--text); border:1px solid var(--border);
    border-radius:8px; padding:12px 6px; font-size:0.85rem; cursor:pointer;
  }
  button:active{background:var(--blue);}
  button.stop{background:var(--danger); border-color:var(--danger); color:#fff; grid-column:span 3;}
  button.toggle-on{background:var(--accent); border-color:var(--accent); color:#fff;}
  .mode-switch{display:flex; gap:8px;}
  .mode-switch button{flex:1;}
  .mode-switch button.active{background:var(--blue); border-color:var(--blue); color:#fff;}
  @media(max-width:480px){ h1{font-size:1.15rem;} }
</style>
</head>
<body>
  <h1>🚗 DriveSafe Rover 2026</h1>
  <div class="sub">Autonomous Smart Road Safety Rover</div>

  <div class="grid">

    <div class="card">
      <h2>Live Status</h2>
      <div class="row"><span>GPS Latitude</span><span id="lat">--</span></div>
      <div class="row"><span>GPS Longitude</span><span id="lng">--</span></div>
      <div class="row"><span>GPS Fix</span><span id="fix">--</span></div>
      <div class="row"><span>Speed (km/h)</span><span id="spd">--</span></div>
      <div class="row"><span>Obstacle Dist (cm)</span><span id="obs">--</span></div>
      <div class="row"><span>Pothole Dist (cm)</span><span id="pot">--</span></div>
      <div class="row"><span>Servo Angle</span><span id="ang">--</span></div>
      <div class="row"><span>Vehicle Mode</span><span id="mode" class="badge mode">--</span></div>
      <div class="row"><span>Motor Status</span><span id="motor">--</span></div>
    </div>

    <div class="card">
      <h2>Mode</h2>
      <div class="mode-switch">
        <button id="btnManual" onclick="setMode('modeManual')">Manual</button>
        <button id="btnAuto" onclick="setMode('modeAuto')">Automatic</button>
      </div>

      <h2 style="margin-top:16px;">Manual Controls</h2>
      <div class="btn-grid">
        <div></div><button onclick="cmd('forward')">▲ Forward</button><div></div>
        <button onclick="cmd('left')">◀ Left</button>
        <button onclick="cmd('stop')" style="background:var(--warn);border-color:var(--warn);color:#000;">■ Stop</button>
        <button onclick="cmd('right')">▶ Right</button>
        <div></div><button onclick="cmd('backward')">▼ Backward</button><div></div>
      </div>

      <h2 style="margin-top:16px;">Camera Servo</h2>
      <div class="btn-grid">
        <button onclick="cmd('servoLeft')">◀ Left</button>
        <button onclick="cmd('servoCenter')">● Center</button>
        <button onclick="cmd('servoRight')">▶ Right</button>
      </div>
    </div>

    <div class="card">
      <h2>Lights</h2>
      <div class="row"><span>Headlight</span><span id="hlBadge" class="badge off">OFF</span></div>
      <div class="btn-grid" style="grid-template-columns:1fr 1fr;">
        <button onclick="cmd('headlightOn')">Head ON</button>
        <button onclick="cmd('headlightOff')">Head OFF</button>
      </div>
      <div class="row" style="margin-top:10px;"><span>Tail Light</span><span id="tlBadge" class="badge off">OFF</span></div>
      <div class="btn-grid" style="grid-template-columns:1fr 1fr;">
        <button onclick="cmd('tailLightOn')">Tail ON</button>
        <button onclick="cmd('tailLightOff')">Tail OFF</button>
      </div>
      <div class="row" style="margin-top:10px;"><span>Hazard</span><span id="hzBadge" class="badge off">OFF</span></div>
      <div class="btn-grid" style="grid-template-columns:1fr 1fr;">
        <button onclick="cmd('hazardOn')">Hazard ON</button>
        <button onclick="cmd('hazardOff')">Hazard OFF</button>
      </div>
    </div>

  </div>

<script>
function cmd(endpoint){
  fetch('/' + endpoint).then(r=>r.json()).then(updateUI).catch(()=>{});
}
function setMode(endpoint){
  fetch('/' + endpoint).then(r=>r.json()).then(updateUI).catch(()=>{});
}
function updateUI(d){
  document.getElementById('lat').innerText = d.latitude.toFixed(6);
  document.getElementById('lng').innerText = d.longitude.toFixed(6);
  document.getElementById('fix').innerText = d.gpsFix ? 'Fixed (' + '✓' + ')' : 'No Fix';
  document.getElementById('spd').innerText = d.speed.toFixed(1);
  document.getElementById('obs').innerText = d.obstacleDistance;
  document.getElementById('pot').innerText = d.potholeDistance;
  document.getElementById('ang').innerText = d.servoAngle + '°';
  document.getElementById('mode').innerText = d.mode;
  document.getElementById('motor').innerText = d.motorStatus;

  setBadge('hlBadge', d.headlight);
  setBadge('tlBadge', d.taillight);
  setBadge('hzBadge', d.hazard);

  document.getElementById('btnManual').classList.toggle('active', d.mode === 'MANUAL');
  document.getElementById('btnAuto').classList.toggle('active', d.mode === 'AUTOMATIC');
}
function setBadge(id, on){
  const el = document.getElementById(id);
  el.innerText = on ? 'ON' : 'OFF';
  el.className = 'badge ' + (on ? 'on' : 'off');
}
function poll(){
  fetch('/status').then(r=>r.json()).then(updateUI).catch(()=>{});
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)rawliteral";

// =====================================================================
// WEB SERVER SETUP - REST endpoints + embedded dashboard
// =====================================================================
void setupWebServer() {

  // ---- Dashboard page ----
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", DASHBOARD_HTML);
  });

  // ---- Status ----
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildStatusJSON());
  });

  // ---- Motor control (manual mode only) ----
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentMode == MODE_MANUAL) { setSpeed(DEFAULT_SPEED); forward(); }
    request->send(200, "application/json", buildStatusJSON());
  });
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentMode == MODE_MANUAL) { setSpeed(DEFAULT_SPEED); backward(); }
    request->send(200, "application/json", buildStatusJSON());
  });
  server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentMode == MODE_MANUAL) { setSpeed(DEFAULT_SPEED); left(); }
    request->send(200, "application/json", buildStatusJSON());
  });
  server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentMode == MODE_MANUAL) { setSpeed(DEFAULT_SPEED); right(); }
    request->send(200, "application/json", buildStatusJSON());
  });
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    stopMotors();
    request->send(200, "application/json", buildStatusJSON());
  });

  // ---- Lights ----
  server.on("/headlightOn",  HTTP_GET, [](AsyncWebServerRequest *request) { headlightOn();  request->send(200, "application/json", buildStatusJSON()); });
  server.on("/headlightOff", HTTP_GET, [](AsyncWebServerRequest *request) { headlightOff(); request->send(200, "application/json", buildStatusJSON()); });
  server.on("/tailLightOn",  HTTP_GET, [](AsyncWebServerRequest *request) { tailLightOn();  request->send(200, "application/json", buildStatusJSON()); });
  server.on("/tailLightOff", HTTP_GET, [](AsyncWebServerRequest *request) { tailLightOff(); request->send(200, "application/json", buildStatusJSON()); });
  server.on("/hazardOn",     HTTP_GET, [](AsyncWebServerRequest *request) { hazardOn();     request->send(200, "application/json", buildStatusJSON()); });
  server.on("/hazardOff",    HTTP_GET, [](AsyncWebServerRequest *request) { hazardOff();    request->send(200, "application/json", buildStatusJSON()); });

  // ---- Servo ----
  server.on("/servoLeft",   HTTP_GET, [](AsyncWebServerRequest *request) { servoLeft();   request->send(200, "application/json", buildStatusJSON()); });
  server.on("/servoCenter", HTTP_GET, [](AsyncWebServerRequest *request) { servoCenter(); request->send(200, "application/json", buildStatusJSON()); });
  server.on("/servoRight",  HTTP_GET, [](AsyncWebServerRequest *request) { servoRight();  request->send(200, "application/json", buildStatusJSON()); });

  // ---- Mode switch ----
  server.on("/modeManual", HTTP_GET, [](AsyncWebServerRequest *request) {
    currentMode = MODE_MANUAL;
    stopMotors();
    servoCenter();
    autoState = AUTO_FORWARD;
    request->send(200, "application/json", buildStatusJSON());
  });
  server.on("/modeAuto", HTTP_GET, [](AsyncWebServerRequest *request) {
    currentMode = MODE_AUTO;
    autoState = AUTO_FORWARD;
    request->send(200, "application/json", buildStatusJSON());
  });

  // ---- 404 ----
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not Found");
  });
}
