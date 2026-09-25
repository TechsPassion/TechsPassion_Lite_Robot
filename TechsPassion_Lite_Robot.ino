#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "secrets.h"  // WIFI_SSID / WIFI_PASSWORD

// -----------------------------------------
// PINS (see "Wiring" in README.md before changing any of these)
// -----------------------------------------
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

#define LEFT_DIR1  38
#define LEFT_DIR2  39
#define LEFT_PWM   40
#define RIGHT_DIR1 47
#define RIGHT_DIR2 48
#define RIGHT_PWM  21

#define OLED_SDA_PIN 1
#define OLED_SCL_PIN 2
#define TRIG_PIN     41
#define ECHO_PIN     42
#define SERVO_PIN    14

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -----------------------------------------
// TUNING
// -----------------------------------------
#define HOSTNAME "techspassion-lite-robot"

// 1 = servo angles above 90 point the sensor/camera LEFT (the convention the
// autopilot has always used). If the radar map or pan slider look mirrored on
// the real robot, set this to 0 - firmware and UI both follow it.
#define PAN_LEFT_IS_HIGH 1

const int   PAN_CENTER        = 90;
const int   PAN_LEFT_SIGN     = PAN_LEFT_IS_HIGH ? 1 : -1;
const int   RADAR_START       = 30;   // servo sweep 30..150 in 10 deg steps
const int   RADAR_STEP        = 10;
const int   RADAR_POINTS      = 13;

const float MANUAL_STOP_CM    = 15;   // manual forward is blocked closer than this
const float SONAR_MAX_CM      = 400;  // HC-SR04 useful range; beyond = "clear"

// Autopilot tuning
const float AUTO_OBSTACLE_CM  = 25;   // stop and sweep for a new heading
const float AUTO_SLOW_CM      = 70;   // start slowing down (and stop glancing)
const float AUTO_SIDE_CM      = 25;   // side glance closer than this -> veer away
const float AUTO_MIN_OPEN_CM  = 35;   // a gap must be this deep to drive into it
const float AUTO_CLEAR_CM     = 60;   // "clear enough" when turning toward a gap
const int   AUTO_MIN_SPEED    = 120;  // slowest PWM that still moves the robot
const int   AUTO_TURN_SPEED   = 180;  // skid-steer turns need torque; never spin slower
const int   VEER_INNER_PCT    = 45;   // inner-side power when curving away from a wall
const int   GLANCE_DEG        = 25;   // side glances while driving
const float AP_MS_PER_DEG     = 8.0;  // rough spin rate at full PWM (only a first guess;
                                      // turns are closed-loop on the sensor anyway)

// Wi-Fi: home network first, own hotspot (AP_SSID in secrets.h) as fallback
const uint32_t STA_CONNECT_MS      = 15000; // boot: how long to try the home Wi-Fi
const uint32_t STA_LOST_TO_AP_MS   = 30000; // running: home Wi-Fi gone this long -> hotspot
const uint32_t STA_RETRY_EVERY_MS  = 60000; // hotspot: retry home Wi-Fi (only with no phones connected)
const uint32_t STA_RETRY_WINDOW_MS = 15000;

const uint32_t CMD_TIMEOUT_MS    = 600;  // manual motion stops if the UI goes quiet
const uint32_t SONAR_INTERVAL_MS = 60;   // HC-SR04 datasheet minimum cycle
const uint32_t OLED_INTERVAL_MS  = 300;
const uint32_t SWEEP_STEP_MAX_MS = 400;  // per-step cap if the sensor stops answering
const uint32_t AP_VEER_MS        = 400;
const uint32_t AP_TURN_BURST_MS  = 100;
const uint32_t AP_REVERSE_MS     = 500;
const uint32_t AP_STUCK_MS       = 2500; // front distance unchanged this long = stuck
const float    AP_STUCK_TOL_CM   = 4;
const uint32_t AP_OSC_WINDOW_MS  = 10000;// 4 turns inside this window = trapped in a corner

// -----------------------------------------
// STATE
// Written by the HTTP server task, read by loop() (and vice versa). Mode
// changes are requested via pendingMode and applied by loop() so the state
// machines are only ever mutated from one task.
// -----------------------------------------
enum { MODE_MANUAL, MODE_AUTO, MODE_RADAR };
enum { AP_DRIVE, AP_SCAN, AP_TURN, AP_TURN_CHECK, AP_REVERSE };
enum { GL_CENTER1, GL_LEFT, GL_CENTER2, GL_RIGHT };

volatile uint8_t  mode          = MODE_MANUAL;
volatile char     pendingMode   = 0;      // 'M', 'A', 'R' or 0
volatile char     manualCmd     = 'S';
volatile uint32_t lastCmdMs     = 0;
volatile int      motorSpeed    = 255;
volatile int      panTarget     = PAN_CENTER;
volatile float    distanceCm    = 0;      // median-filtered; 0 = nothing in range
volatile bool     forwardBlocked = false;
volatile char     appliedCmd    = 'S';
bool camOk  = false;
bool oledOk = false;
volatile bool apMode = false;             // true while running our own hotspot

// Radar
int radarData[RADAR_POINTS];
volatile int      radarCount = 0;         // points captured in the current/last scan
volatile uint16_t radarSeq   = 0;         // completed scans, lets the UI spot new data
uint32_t sweepSeq = 0;                    // sonarSeq when the servo last moved
uint32_t sweepMoveMs = 0;

// Autopilot
int apState = AP_DRIVE;
uint32_t apTimer = 0;
uint32_t apSeq = 0;                       // sonarSeq when the servo last moved
uint32_t frontSeq = 0;                    // last ping judged as "straight ahead"
int glancePhase = GL_CENTER1;
float frontCm = 0;                        // latest straight-ahead reading (0 = clear)
char veerCmd = 'F';
uint32_t veerUntil = 0;
float stuckRef = 0;
uint32_t stuckSince = 0;
char apTurnDir = 'L';
uint32_t apTurnBudget = 0, apTurnUsed = 0, apBurstMs = 0;
float apNeedCm = AUTO_CLEAR_CM;
bool apTurnAfterReverse = false;          // after backing up: turn (boxed in) or re-sweep (stuck)
uint32_t turnTimes[4] = {0, 0, 0, 0};
uint8_t turnIdx = 0;

// -----------------------------------------
// ULTRASONIC (interrupt driven, never blocks loop)
// -----------------------------------------
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;
volatile bool echoReady = false;
bool pingPending = false;
uint32_t lastPingMs = 0;
float samples[3] = {SONAR_MAX_CM, SONAR_MAX_CM, SONAR_MAX_CM};
uint8_t sampleIdx = 0;
// Unfiltered latest reading + a counter, for code that moves the servo and
// needs a reading taken *after* the move (the median would mix angles).
float lastRawCm = 0;
uint32_t sonarSeq = 0;

void onEcho() {
  uint32_t t = micros();
  if (digitalRead(ECHO_PIN)) {
    echoRiseUs = t;
  } else if (echoRiseUs) {
    echoWidthUs = t - echoRiseUs;
    echoRiseUs = 0;
    echoReady = true;
  }
}

void addSample(float cm) {
  lastRawCm = (cm <= 0 || cm >= SONAR_MAX_CM) ? 0 : cm;
  sonarSeq++;
  samples[sampleIdx] = (cm <= 0 || cm >= SONAR_MAX_CM) ? SONAR_MAX_CM : cm;
  sampleIdx = (sampleIdx + 1) % 3;
  float a = samples[0], b = samples[1], c = samples[2];
  float med = max(min(a, b), min(max(a, b), c));  // median of 3 rejects single spikes
  distanceCm = (med >= SONAR_MAX_CM) ? 0 : med;
}

void updateSonar(uint32_t now) {
  if (echoReady) {
    echoReady = false;
    pingPending = false;
    float cm = echoWidthUs / 58.0f;
    addSample(cm >= 2 ? cm : 0);
  }
  if (now - lastPingMs >= SONAR_INTERVAL_MS) {
    if (pingPending) addSample(0);  // no echo at all: treat as clear
    lastPingMs = now;
    pingPending = true;
    echoRiseUs = 0;
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}

bool obstacleWithin(float cm) { float d = distanceCm; return d > 0 && d < cm; }
float orMax(float cm) { return cm > 0 ? cm : SONAR_MAX_CM; }

// -----------------------------------------
// SERVO (native LEDC; ESP32Servo conflicts with the camera's timer)
// -----------------------------------------
void setServoAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  int pulse_us = map(angle, 0, 180, 500, 2400);
  int duty = (pulse_us * 16384) / 20000;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(4, duty);
#endif
}

// -----------------------------------------
// MOTORS (right side is wired inverted relative to left)
// -----------------------------------------
void driveMotors(char cmd, int speed) {
  if (cmd == 'F') {
    digitalWrite(LEFT_DIR1, HIGH); digitalWrite(LEFT_DIR2, LOW); analogWrite(LEFT_PWM, speed);
    digitalWrite(RIGHT_DIR1, LOW); digitalWrite(RIGHT_DIR2, HIGH); analogWrite(RIGHT_PWM, speed);
  } else if (cmd == 'B') {
    digitalWrite(LEFT_DIR1, LOW); digitalWrite(LEFT_DIR2, HIGH); analogWrite(LEFT_PWM, speed);
    digitalWrite(RIGHT_DIR1, HIGH); digitalWrite(RIGHT_DIR2, LOW); analogWrite(RIGHT_PWM, speed);
  } else if (cmd == 'L') {
    digitalWrite(LEFT_DIR1, LOW); digitalWrite(LEFT_DIR2, HIGH); analogWrite(LEFT_PWM, speed);
    digitalWrite(RIGHT_DIR1, LOW); digitalWrite(RIGHT_DIR2, HIGH); analogWrite(RIGHT_PWM, speed);
  } else if (cmd == 'R') {
    digitalWrite(LEFT_DIR1, HIGH); digitalWrite(LEFT_DIR2, LOW); analogWrite(LEFT_PWM, speed);
    digitalWrite(RIGHT_DIR1, HIGH); digitalWrite(RIGHT_DIR2, LOW); analogWrite(RIGHT_PWM, speed);
  } else if (cmd == 'l') {  // forward, curving left (autopilot veer)
    digitalWrite(LEFT_DIR1, HIGH); digitalWrite(LEFT_DIR2, LOW); analogWrite(LEFT_PWM, speed * VEER_INNER_PCT / 100);
    digitalWrite(RIGHT_DIR1, LOW); digitalWrite(RIGHT_DIR2, HIGH); analogWrite(RIGHT_PWM, speed);
  } else if (cmd == 'r') {  // forward, curving right
    digitalWrite(LEFT_DIR1, HIGH); digitalWrite(LEFT_DIR2, LOW); analogWrite(LEFT_PWM, speed);
    digitalWrite(RIGHT_DIR1, LOW); digitalWrite(RIGHT_DIR2, HIGH); analogWrite(RIGHT_PWM, speed * VEER_INNER_PCT / 100);
  } else {
    digitalWrite(LEFT_DIR1, LOW); digitalWrite(LEFT_DIR2, LOW); analogWrite(LEFT_PWM, 0);
    digitalWrite(RIGHT_DIR1, LOW); digitalWrite(RIGHT_DIR2, LOW); analogWrite(RIGHT_PWM, 0);
  }
}

// Only touches the pins when the command or speed actually changes.
int appliedSpeed = -1;
void applyDriveAt(char cmd, int spd) {
  if (cmd == appliedCmd && spd == appliedSpeed) return;
  appliedCmd = cmd;
  appliedSpeed = spd;
  driveMotors(cmd, spd);
}
void applyDrive(char cmd) { applyDriveAt(cmd, motorSpeed); }

const char* modeName(uint8_t m) {
  return m == MODE_AUTO ? "auto" : m == MODE_RADAR ? "radar" : "manual";
}

// -----------------------------------------
// HTML UI (self-contained: no CDN fonts/icons, works without internet)
// -----------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b1120">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="Robot">
<title>TechsPassion Robot</title>
<link rel="icon" href="data:,">
<style>
:root{--bg:#0b1120;--panel:rgba(22,32,50,.72);--line:rgba(255,255,255,.08);--text:#e8eef6;--muted:#8a9ab0;--brand:#2ecc71;--warn:#f5b301;--danger:#ef4444;--blue:#3b82f6;--btn:#243044}
*{box-sizing:border-box}
html,body{margin:0}
body{background:var(--bg) radial-gradient(circle at 50% -10%,rgba(46,204,113,.16),transparent 55%) no-repeat;color:var(--text);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;min-height:100vh;padding:12px 16px calc(24px + env(safe-area-inset-bottom));-webkit-tap-highlight-color:transparent}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;max-width:1100px;margin:0 auto 12px}
h1{margin:0;font-size:20px;font-weight:800;letter-spacing:.5px;color:var(--brand);text-shadow:0 0 12px rgba(46,204,113,.45)}
h1 small{display:block;font-size:11px;font-weight:500;color:var(--muted);letter-spacing:1.5px;text-transform:uppercase;text-shadow:none}
.pill{display:flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;background:var(--panel);border:1px solid var(--line);font-size:12px;color:var(--muted);white-space:nowrap}
.dot{width:8px;height:8px;border-radius:50%;background:var(--danger);box-shadow:0 0 8px var(--danger)}
.pill.on .dot{background:var(--brand);box-shadow:0 0 8px var(--brand)}
.layout{display:flex;flex-direction:column;gap:14px;max-width:1100px;margin:0 auto}
.col{display:contents}
/* Phone order: camera, then the D-pad right under it, then the rest */
.o1{order:1}.o2{order:2}.o3{order:3}.o4{order:4}.o5{order:5}.o6{order:6}.o7{order:7}
@media(min-width:900px),(orientation:landscape) and (min-width:560px){.layout{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(0,1fr);align-items:start}.col{display:flex;flex-direction:column;gap:14px;min-width:0}}
@media(min-width:900px) and (min-height:560px){.col>*{order:0}}
button,input{touch-action:manipulation}
.pad,.actions,.seg,.pan,header{user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
.card{background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:14px;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);box-shadow:0 10px 25px -8px rgba(0,0,0,.6)}
.label{font-size:11px;text-transform:uppercase;letter-spacing:1.2px;color:var(--muted);margin:0 0 10px;display:flex;justify-content:space-between;align-items:center;gap:8px}
.cam{padding:8px}
.cam-box{position:relative;aspect-ratio:4/3;background:#05080f;border-radius:12px;overflow:hidden}
.cam-box img{width:100%;height:100%;object-fit:contain;display:block}
.overlay{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;color:var(--muted);font-size:13px;background:#05080f}
.hide{display:none!important}
.hud{position:absolute;left:10px;bottom:10px;padding:5px 10px;border-radius:10px;background:rgba(0,0,0,.6);font-weight:700;font-variant-numeric:tabular-nums}
.icon-btn{position:absolute;right:10px;top:10px;width:38px;height:38px;border-radius:10px;border:1px solid var(--line);background:rgba(0,0,0,.55);color:#fff;display:grid;place-items:center;cursor:pointer}
svg{width:22px;height:22px;fill:none;stroke:currentColor;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round;flex:none}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.stat span{font-size:11px;text-transform:uppercase;letter-spacing:1.2px;color:var(--muted)}
.stat b{display:block;font-size:20px;margin-top:4px;color:var(--brand);font-variant-numeric:tabular-nums;white-space:nowrap}
.bar{height:6px;border-radius:3px;background:#1c2638;margin-top:12px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--brand);transition:width .25s,background .25s}
.ok{color:var(--brand)!important}.warn{color:var(--warn)!important}.danger{color:var(--danger)!important}
.actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.act{border:0;border-radius:14px;padding:14px 10px;font:600 15px system-ui,sans-serif;display:flex;gap:8px;align-items:center;justify-content:center;cursor:pointer;color:#111;background:var(--warn);box-shadow:0 0 14px rgba(245,179,1,.25);transition:transform .1s,background .2s,box-shadow .2s}
.act:active{transform:scale(.97)}
.act.on{background:var(--brand);box-shadow:0 0 18px rgba(46,204,113,.55)}
.act.radar{background:var(--blue);color:#fff;box-shadow:0 0 14px rgba(59,130,246,.3)}
.act:disabled{opacity:.65;cursor:default}
canvas{width:100%;display:block}
.seg{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.seg button{background:var(--btn);color:var(--text);border:1px solid var(--line);border-radius:12px;padding:10px 0;font:600 14px system-ui,sans-serif;cursor:pointer;transition:background .15s}
.seg button small{display:block;font-size:11px;color:var(--muted);font-weight:500}
.seg button.active{background:var(--brand);color:#06240f;border-color:var(--brand);box-shadow:0 0 14px rgba(46,204,113,.5)}
.seg button.active small{color:#06240f}
.pan{display:flex;gap:10px;align-items:center}
input[type=range]{-webkit-appearance:none;appearance:none;flex:1;min-width:0;height:36px;background:transparent;margin:0}
input[type=range]:focus{outline:none}
input[type=range]::-webkit-slider-runnable-track{height:6px;background:#2a364b;border-radius:3px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;border-radius:50%;background:var(--brand);margin-top:-10px;box-shadow:0 0 10px var(--brand);border:0}
input[type=range]::-moz-range-track{height:6px;background:#2a364b;border-radius:3px}
input[type=range]::-moz-range-thumb{width:26px;height:26px;border-radius:50%;background:var(--brand);border:0;box-shadow:0 0 10px var(--brand)}
.small-btn{background:var(--btn);color:var(--text);border:1px solid var(--line);border-radius:10px;padding:8px 12px;font:600 12px system-ui,sans-serif;cursor:pointer}
.pad{display:grid;grid-template-columns:repeat(3,76px);grid-template-rows:repeat(3,76px);gap:12px;justify-content:center;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
.pad button{background:var(--btn);color:#fff;border:1px solid var(--line);border-radius:18px;display:grid;place-items:center;cursor:pointer;touch-action:none;box-shadow:0 4px 8px rgba(0,0,0,.4),inset 0 1px 1px rgba(255,255,255,.08);transition:background .12s,transform .12s,box-shadow .12s}
.pad button svg{width:30px;height:30px}
.pad button.held{background:var(--brand);border-color:var(--brand);box-shadow:0 0 18px var(--brand);transform:scale(.94)}
.pad button.blocked{background:var(--danger);border-color:var(--danger);box-shadow:0 0 18px var(--danger)}
.pad .stop{background:#3a1f25;color:#ff8a8a}
#bF{grid-area:1/2}#bL{grid-area:2/1}#bS{grid-area:2/2}#bR{grid-area:2/3}#bB{grid-area:3/2}
.hint{text-align:center;color:var(--muted);font-size:12px;margin:12px 0 0}
@media(hover:none){.hint{display:none}}
@media(max-width:360px){.pad{grid-template-columns:repeat(3,66px);grid-template-rows:repeat(3,66px)}}
.toast{position:fixed;left:50%;bottom:calc(20px + env(safe-area-inset-bottom));transform:translate(-50%,20px);background:#1d2738;border:1px solid var(--line);padding:10px 16px;border-radius:12px;font-size:14px;opacity:0;transition:.25s;pointer-events:none;z-index:9;max-width:calc(100% - 32px)}
.toast.show{opacity:1;transform:translate(-50%,0)}
/* Drive view: full-screen camera with the D-pad floating over it */
body.drive{overflow:hidden}
body.drive .cam{backdrop-filter:none;-webkit-backdrop-filter:none}
body.drive .toast{z-index:30;top:calc(12px + env(safe-area-inset-top));bottom:auto}
body.drive .cam-box{position:fixed;inset:0;z-index:20;border-radius:0;aspect-ratio:auto;background:#000}
body.drive .cam-box .pad{position:absolute;right:max(16px,env(safe-area-inset-right));bottom:max(16px,env(safe-area-inset-bottom));grid-template-columns:repeat(3,68px);grid-template-rows:repeat(3,68px);gap:8px}
body.drive .cam-box .pad button{background:rgba(20,28,40,.5);backdrop-filter:blur(4px);-webkit-backdrop-filter:blur(4px);border-color:rgba(255,255,255,.18)}
body.drive .cam-box .pad .stop{background:rgba(90,25,35,.6)}
body.drive .cam-box .pad button.held{background:var(--brand)}
body.drive .cam-box .pad button.blocked{background:var(--danger)}
body.drive .hud{top:max(10px,env(safe-area-inset-top));bottom:auto;left:max(10px,env(safe-area-inset-left));font-size:18px}
body.drive .icon-btn{top:max(10px,env(safe-area-inset-top));right:max(10px,env(safe-area-inset-right))}
@media(orientation:portrait){body.drive .cam-box .pad{right:auto;left:50%;transform:translateX(-50%)}}
</style></head><body>
<header>
  <h1>TechsPassion Robot<small id="host">lite</small></h1>
  <div class="pill" id="link"><span class="dot"></span><span id="linkTxt">Connecting&hellip;</span></div>
</header>
<main class="layout">
<div class="col">
  <section class="card cam o1">
    <div class="cam-box" id="camBox">
      <img id="stream" alt="Robot camera">
      <div class="overlay" id="camMsg"><svg viewBox="0 0 24 24"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg><span id="camTxt">Connecting to camera&hellip;</span></div>
      <div class="hud" id="hud">-- cm</div>
      <button class="icon-btn" id="fs" title="Drive view" aria-label="Drive view"><svg viewBox="0 0 24 24"><path d="M8 3H5a2 2 0 0 0-2 2v3M21 8V5a2 2 0 0 0-2-2h-3M16 21h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3"/></svg></button>
    </div>
  </section>
  <section class="card o7">
    <p class="label"><span>Radar map</span><span id="radarInfo">No scan yet</span></p>
    <canvas id="radar"></canvas>
  </section>
</div>
<div class="col">
  <section class="card o5">
    <div class="stats">
      <div class="stat"><span>Distance</span><b id="dist">--</b></div>
      <div class="stat"><span>Mode</span><b id="mode">--</b></div>
      <div class="stat"><span>Wi-Fi</span><b id="rssi">--</b></div>
    </div>
    <div class="bar"><i id="prox"></i></div>
  </section>
  <section class="actions o4">
    <button class="act" id="btnAuto"><svg viewBox="0 0 24 24"><rect x="4" y="8" width="16" height="12" rx="3"/><path d="M12 4v4M9 13h.01M15 13h.01M9 17h6"/></svg><span>Auto Pilot</span></button>
    <button class="act radar" id="btnRadar"><svg viewBox="0 0 24 24"><path d="M12 12l7-7M21 12a9 9 0 1 1-9-9M16 12a4 4 0 1 1-4-4"/></svg><span>Radar Scan</span></button>
  </section>
  <section class="card o6">
    <p class="label"><span>Camera pan</span><span id="panVal">Center</span></p>
    <div class="pan"><input type="range" id="pan" min="0" max="180" value="90" aria-label="Camera pan"><button class="small-btn" id="panC">Center</button></div>
  </section>
  <section class="card o3">
    <p class="label"><span>Speed</span></p>
    <div class="seg" id="speed"><button data-v="120">Slow<small>120</small></button><button data-v="180">Medium<small>180</small></button><button data-v="255">Fast<small>255</small></button></div>
  </section>
  <section class="card o2" id="padCard">
    <div class="pad" id="pad">
      <button id="bF" aria-label="Forward"><svg viewBox="0 0 24 24"><path d="M6 15l6-6 6 6"/></svg></button>
      <button id="bL" aria-label="Left"><svg viewBox="0 0 24 24"><path d="M15 6l-6 6 6 6"/></svg></button>
      <button id="bS" class="stop" aria-label="Stop"><svg viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="2" fill="currentColor"/></svg></button>
      <button id="bR" aria-label="Right"><svg viewBox="0 0 24 24"><path d="M9 6l6 6-6 6"/></svg></button>
      <button id="bB" aria-label="Back"><svg viewBox="0 0 24 24"><path d="M6 9l6 6 6-6"/></svg></button>
    </div>
    <p class="hint">Keyboard: arrows / WASD to drive &middot; Space to stop</p>
  </section>
</div>
</main>
<div class="toast" id="toast"></div>
<script>
(()=>{
const $=id=>document.getElementById(id);
const H=location.hostname;
let lh=1,mode='manual',held=null,holdT=null,fails=0,wasBlk=false,lastScan=-1,radar=[],rc=0,toastT;
let modeLock=0,speedLock=0,panLock=0,panPending=null,panTimer=null;

const api=p=>{const c=new AbortController(),t=setTimeout(()=>c.abort(),1500);
  return fetch(p+(p.includes('?')?'&':'?')+'t='+Date.now(),{cache:'no-store',signal:c.signal}).finally(()=>clearTimeout(t))};
const send=c=>api('/control?cmd='+c).catch(()=>{});
function toast(m){const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(toastT);toastT=setTimeout(()=>t.classList.remove('show'),2400)}
$('host').textContent=H;let net='';

/* Camera stream with auto-retry */
const img=$('stream');let camRetry;
function startCam(){img.src=location.protocol+'//'+H+':81/stream?r='+Date.now()}
img.onload=()=>$('camMsg').classList.add('hide');
img.onerror=()=>{$('camMsg').classList.remove('hide');$('camTxt').textContent='Camera offline — retrying…';clearTimeout(camRetry);camRetry=setTimeout(startCam,2500)};
startCam();
/* Drive view: camera fills the screen and the D-pad moves on top of it.
   Uses real fullscreen where the browser allows it (Android/desktop);
   on iPhone it still fills the page (add to Home Screen to hide Safari's bars). */
const padEl=$('pad'),padHome=$('padCard');
function setDrive(on){if(document.body.classList.contains('drive')===on)return;
  document.body.classList.toggle('drive',on);
  if(on){$('camBox').appendChild(padEl);
    const d=document.documentElement;
    if(d.requestFullscreen&&!document.fullscreenElement)d.requestFullscreen().then(()=>{try{screen.orientation.lock('landscape').catch(()=>{})}catch(_){}}).catch(()=>{});
  }else{padHome.insertBefore(padEl,padHome.firstChild);
    if(document.fullscreenElement)document.exitFullscreen().catch(()=>{})}
  release();drawRadar()}
$('fs').onclick=()=>setDrive(!document.body.classList.contains('drive'));
document.addEventListener('fullscreenchange',()=>{if(!document.fullscreenElement)setDrive(false)});
addEventListener('keydown',e=>{if(e.code==='Escape')setDrive(false)});

/* Driving: held buttons re-send every 200 ms; the robot stops by itself
   if it hears nothing for 600 ms (lost Wi-Fi, closed tab, etc.) */
const pad={F:$('bF'),B:$('bB'),L:$('bL'),R:$('bR')};
function press(c){if(held===c)return;if(held)pad[held].classList.remove('held');
  held=c;pad[c].classList.add('held');if(navigator.vibrate)navigator.vibrate(12);if(mode!=='manual'){setMode('manual');modeLock=Date.now()+800}
  send(c);clearInterval(holdT);holdT=setInterval(()=>send(c),200)}
function release(c){if(!held||(c&&held!==c))return;pad[held].classList.remove('held');held=null;clearInterval(holdT);send('S')}
function stopAll(){if(held){pad[held].classList.remove('held');held=null;clearInterval(holdT)}send('S');setMode('manual');modeLock=Date.now()+800}
for(const c in pad){const b=pad[c];
  b.addEventListener('pointerdown',e=>{e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(_){}press(c)});
  ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>release(c)))}
$('bS').addEventListener('pointerdown',e=>{e.preventDefault();stopAll()});
$('pad').addEventListener('contextmenu',e=>e.preventDefault());
const keys={ArrowUp:'F',KeyW:'F',ArrowDown:'B',KeyS:'B',ArrowLeft:'L',KeyA:'L',ArrowRight:'R',KeyD:'R'};
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
  if(e.code==='Space'){e.preventDefault();stopAll();return}
  const c=keys[e.code];if(c){e.preventDefault();if(!e.repeat)press(c)}});
addEventListener('keyup',e=>{const c=keys[e.code];if(c)release(c)});
addEventListener('blur',()=>release());
document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});

/* Modes */
function setMode(m){mode=m;const a=$('btnAuto'),r=$('btnRadar'),md=$('mode');
  a.classList.toggle('on',m==='auto');a.querySelector('span').textContent=m==='auto'?'Auto Pilot ON':'Auto Pilot';
  r.disabled=m==='radar';r.querySelector('span').textContent=m==='radar'?'Scanning…':'Radar Scan';
  md.textContent={manual:'Manual',auto:'Auto',radar:'Radar'}[m]||m;md.className=m==='manual'?'':'warn'}
$('btnAuto').onclick=()=>{modeLock=Date.now()+800;
  if(mode==='auto'){send('S');setMode('manual')}
  else{release();send('A');setMode('auto');toast('Auto Pilot on — press Stop or any arrow to take over')}};
$('btnRadar').onclick=()=>{release();modeLock=Date.now()+800;send('RDR');setMode('radar');rc=0;drawRadar()};

/* Speed */
const segs=[...document.querySelectorAll('#speed button')];
function markSpeed(v){segs.forEach(b=>b.classList.toggle('active',+b.dataset.v===v))}
segs.forEach(b=>b.onclick=()=>{const v=+b.dataset.v;markSpeed(v);speedLock=Date.now()+1000;api('/speed?val='+v).catch(()=>toast('Could not change speed'))});

/* Camera pan: slider left = look left, throttled to ~12 requests/s */
const pan=$('pan');
const toServo=v=>lh?180-v:v;
function panLabel(v){const d=v-90;return Math.abs(d)<3?'Center':Math.abs(d)+'° '+(d<0?'left':'right')}
function setPan(v){pan.value=v;$('panVal').textContent=panLabel(v);panLock=Date.now()+1200;panPending=v;
  if(!panTimer)panTimer=setTimeout(()=>{panTimer=null;if(panPending!==null){api('/servo?pos='+toServo(panPending)).catch(()=>{});panPending=null}},80)}
pan.addEventListener('input',()=>setPan(+pan.value));
$('panC').onclick=()=>setPan(90);

/* Radar map */
function drawRadar(){const c=$('radar'),w=c.clientWidth||300,h=Math.round(w*.56),dpr=devicePixelRatio||1;
  c.width=w*dpr;c.height=h*dpr;c.style.height=h+'px';const x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);
  const cx=w/2,cy=h-16,R=Math.min(cx-16,cy-8),MAX=200,rad=a=>a*Math.PI/180,
        pt=(a,r)=>[cx+Math.cos(rad(a))*r,cy-Math.sin(rad(a))*r];
  x.clearRect(0,0,w,h);
  x.fillStyle='rgba(46,204,113,.05)';x.beginPath();x.moveTo(cx,cy);x.arc(cx,cy,R,-rad(150),-rad(30));x.closePath();x.fill();
  x.font='10px system-ui,sans-serif';x.textAlign='center';
  for(const d of [50,100,150,200]){const r=d/MAX*R;x.strokeStyle='rgba(46,204,113,.22)';x.lineWidth=1;
    x.beginPath();x.arc(cx,cy,r,Math.PI,2*Math.PI);x.stroke();x.fillStyle='#5f7188';x.fillText(d+'',cx+r,cy+12)}
  x.setLineDash([3,4]);x.strokeStyle='rgba(46,204,113,.15)';
  for(const a of [30,60,90,120,150]){const[p,q]=pt(a,R);x.beginPath();x.moveTo(cx,cy);x.lineTo(p,q);x.stroke()}
  x.setLineDash([]);x.fillStyle='#8a9ab0';x.font='600 11px system-ui,sans-serif';
  x.fillText('L',10,cy-2);x.fillText('R',w-10,cy-2);
  const pts=[];for(let i=0;i<rc&&i<radar.length;i++){const a=30+i*10,ca=lh?a:180-a,d=radar[i],clear=d<=0||d>=MAX;
    pts.push({ca,d,clear,r:(clear?MAX:d)/MAX*R})}
  pts.sort((a,b)=>b.ca-a.ca);
  if(pts.length){x.beginPath();x.moveTo(cx,cy);for(const p of pts){const[u,v]=pt(p.ca,p.r);x.lineTo(u,v)}x.closePath();
    x.fillStyle='rgba(46,204,113,.16)';x.fill();x.strokeStyle='rgba(46,204,113,.5)';x.stroke();
    for(const p of pts){if(p.clear)continue;const[u,v]=pt(p.ca,p.r);x.fillStyle=p.d<30?'#ef4444':'#f5b301';
      x.beginPath();x.arc(u,v,5,0,2*Math.PI);x.fill()}}
  if(mode==='radar'&&rc<13){const a=30+rc*10,[u,v]=pt(lh?a:180-a,R);x.strokeStyle='rgba(46,204,113,.9)';x.lineWidth=2;
    x.beginPath();x.moveTo(cx,cy);x.lineTo(u,v);x.stroke()}
  x.fillStyle='#2ecc71';x.beginPath();x.moveTo(cx,cy-9);x.lineTo(cx-6,cy+3);x.lineTo(cx+6,cy+3);x.closePath();x.fill()}
addEventListener('resize',drawRadar);

/* Status polling */
function link(ok,ms){$('link').classList.toggle('on',ok);$('linkTxt').textContent=ok?'Online · '+ms+' ms':'Offline'}
function update(d){const now=Date.now();lh=d.lh;
  const cls=d.dist>0&&d.dist<15?'danger':d.dist>0&&d.dist<35?'warn':'ok',txt=d.dist>0?Math.round(d.dist)+' cm':'Clear';
  $('dist').textContent=txt;$('dist').className=cls;$('hud').textContent=txt;$('hud').className='hud '+cls;
  const p=$('prox');p.style.width=(d.dist>0?Math.max(4,100-Math.min(d.dist,200)/2):0)+'%';
  p.style.background=cls==='danger'?'var(--danger)':cls==='warn'?'var(--warn)':'var(--brand)';
  const r=$('rssi');
  if(d.net==='ap'){r.textContent='Hotspot';r.className='ok';r.title='Connected directly to the robot'}
  else{r.textContent=d.rssi>=-60?'Strong':d.rssi>=-72?'Fair':'Weak';r.className=d.rssi>=-60?'ok':d.rssi>=-72?'warn':'danger';r.title=d.rssi+' dBm'}
  if(d.net!==net){net=d.net;$('host').textContent=d.ip+(net==='ap'?' · hotspot':'')}
  if(!d.cam&&!$('camMsg').classList.contains('hide'))$('camTxt').textContent='Camera failed to start on the robot';
  const prev=mode;if(now>modeLock&&d.mode!==mode){setMode(d.mode);if(prev==='auto'&&d.mode==='manual'&&!held)toast('Auto Pilot stopped')}
  if(now>speedLock)markSpeed(d.spd);
  if(now>panLock){const v=lh?180-d.pan:d.pan;if(+pan.value!==v){pan.value=v;$('panVal').textContent=panLabel(v)}}
  pad.F.classList.toggle('blocked',!!d.blk);if(d.blk&&!wasBlk)toast('Obstacle ahead — forward blocked');wasBlk=!!d.blk;
  if(d.scan!==lastScan||d.mode==='radar'||rc!==d.rc){if(lastScan>=0&&d.scan!==lastScan&&prev==='radar')toast('Radar scan complete');
    lastScan=d.scan;radar=d.radar;rc=d.rc;drawRadar()}
  $('radarInfo').textContent=d.mode==='radar'?'Scanning '+d.rc+'/13':d.scan>0?'Scan #'+d.scan:'No scan yet'}
async function poll(){const t0=performance.now();
  try{const d=await(await api('/status')).json();fails=0;link(true,Math.round(performance.now()-t0));update(d)}
  catch(e){if(++fails>=2)link(false)}
  setTimeout(poll,fails?1000:400)}
setMode('manual');drawRadar();poll();
})();
</script></body></html>
)rawliteral";

// -----------------------------------------
// WEB HANDLERS
// -----------------------------------------
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static bool queryParam(httpd_req_t *req, const char *key, char *out, size_t len) {
  char query[96];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  return httpd_query_key_value(query, key, out, len) == ESP_OK;
}

static esp_err_t sendOk(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, "OK", 2);
}

esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, index_html, strlen(index_html));
}

esp_err_t control_handler(httpd_req_t *req) {
  char val[8];
  if (!queryParam(req, "cmd", val, sizeof(val))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing cmd");
    return ESP_FAIL;
  }
  if (strcmp(val, "A") == 0) {
    manualCmd = 'S';
    pendingMode = 'A';
  } else if (strcmp(val, "RDR") == 0) {
    manualCmd = 'S';
    pendingMode = 'R';
  } else if (strchr("FBLRS", val[0]) && val[1] == '\0') {
    manualCmd = val[0];
    lastCmdMs = millis();
    pendingMode = 'M';  // any manual command takes over from auto/radar
  } else {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown cmd");
    return ESP_FAIL;
  }
  return sendOk(req);
}

esp_err_t servo_handler(httpd_req_t *req) {
  char val[8];
  if (queryParam(req, "pos", val, sizeof(val)) && mode == MODE_MANUAL) {
    panTarget = constrain(atoi(val), 0, 180);
  }
  return sendOk(req);
}

esp_err_t speed_handler(httpd_req_t *req) {
  char val[8];
  if (queryParam(req, "val", val, sizeof(val))) {
    motorSpeed = constrain(atoi(val), 0, 255);
  }
  return sendOk(req);
}

// Hidden test hook: /wifi?mode=ap switches to the hotspot now (it hops back to
// the home Wi-Fi by itself within ~1 min if no phone joins).
volatile bool forceApRequest = false;
esp_err_t wifi_handler(httpd_req_t *req) {
  char val[8];
  if (queryParam(req, "mode", val, sizeof(val)) && strcmp(val, "ap") == 0) forceApRequest = true;
  return sendOk(req);
}

esp_err_t status_handler(httpd_req_t *req) {
  char json[480];
  bool ap = apMode;
  IPAddress ip = ap ? WiFi.softAPIP() : WiFi.localIP();
  int n = snprintf(json, sizeof(json),
    "{\"dist\":%.1f,\"mode\":\"%s\",\"cmd\":\"%c\",\"spd\":%d,\"pan\":%d,\"lh\":%d,\"blk\":%d,"
    "\"net\":\"%s\",\"ip\":\"%u.%u.%u.%u\",\"rssi\":%d,\"up\":%lu,\"cam\":%d,\"scan\":%u,\"rc\":%d,\"radar\":[",
    (float)distanceCm, modeName(mode), (char)appliedCmd, (int)motorSpeed, (int)panTarget, PAN_LEFT_IS_HIGH,
    forwardBlocked ? 1 : 0, ap ? "ap" : "sta", ip[0], ip[1], ip[2], ip[3],
    ap ? 0 : (int)WiFi.RSSI(), (unsigned long)(millis() / 1000), camOk ? 1 : 0,
    (unsigned)radarSeq, (int)radarCount);
  for (int i = 0; i < RADAR_POINTS; i++) {
    n += snprintf(json + n, sizeof(json) - n, i ? ",%d" : "%d", radarData[i]);
  }
  n += snprintf(json + n, sizeof(json) - n, "]}");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, n);
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
  if (!camOk) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not initialised");
    return ESP_FAIL;
  }
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted) res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned)_jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) break;
  }
  return res;
}

void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;
  config.lru_purge_enable = true;  // phones leave idle sockets open; recycle them

  httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler,   .user_ctx = NULL };
  httpd_uri_t control_uri = { .uri = "/control", .method = HTTP_GET, .handler = control_handler, .user_ctx = NULL };
  httpd_uri_t servo_uri   = { .uri = "/servo",   .method = HTTP_GET, .handler = servo_handler,   .user_ctx = NULL };
  httpd_uri_t status_uri  = { .uri = "/status",  .method = HTTP_GET, .handler = status_handler,  .user_ctx = NULL };
  httpd_uri_t speed_uri   = { .uri = "/speed",   .method = HTTP_GET, .handler = speed_handler,   .user_ctx = NULL };
  httpd_uri_t wifi_uri    = { .uri = "/wifi",    .method = HTTP_GET, .handler = wifi_handler,    .user_ctx = NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &control_uri);
    httpd_register_uri_handler(camera_httpd, &servo_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &speed_uri);
    httpd_register_uri_handler(camera_httpd, &wifi_uri);
  }

  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 32769;
  config_stream.lru_purge_enable = true;
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&stream_httpd, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// -----------------------------------------
// OLED
// -----------------------------------------
void oledMessage(const char *line1, const char *line2) {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.println("TechsPassion Robot");
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 20); display.println(line1);
  display.setCursor(0, 34); display.println(line2);
  display.display();
}

void drawOled() {
  if (!oledOk) return;
  float d = distanceCm;
  const char *modeTxt = mode == MODE_AUTO ? "AUTO" : mode == MODE_RADAR ? "RADAR" : "MANUAL";
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.println("TechsPassion Robot");
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
  if (apMode) {
    // Hotspot: everything needed to connect a phone
    display.setCursor(0, 12); display.print("WiFi "); display.println(AP_SSID);
    display.setCursor(0, 22); display.print("Pass "); display.println(AP_PASSWORD);
    display.setCursor(0, 32); display.print("Open "); display.println(WiFi.softAPIP());
    display.setCursor(0, 44); display.print("Dist ");
    if (d > 0) { display.print(d, 0); display.print("cm"); } else display.print("clear");
    display.setCursor(0, 54); display.print(modeTxt);
    display.print("  Phones "); display.print(WiFi.softAPgetStationNum());
  } else {
    display.setCursor(0, 12); display.print("IP   ");
    if (WiFi.isConnected()) display.println(WiFi.localIP()); else display.println("reconnecting...");
    display.setCursor(0, 23); display.print("Dist ");
    if (d > 0) { display.print(d, 0); display.println(" cm"); } else display.println("clear");
    display.setCursor(0, 34); display.print("Mode "); display.println(modeTxt);
    display.setCursor(0, 45); display.print("Spd  "); display.print((int)motorSpeed);
    display.print(camOk ? "   Cam OK" : "   Cam ERR");
  }
  display.display();
}

// -----------------------------------------
// WI-FI: home network, falling back to our own hotspot
// -----------------------------------------
uint32_t staDownSince = 0;
uint32_t staRetryAt = 0;
uint32_t staRetryStarted = 0;
bool staRetrying = false;

void startAccessPoint() {
  // Stop the home-network reconnect attempts: their channel scans would keep
  // knocking the phone off the hotspot.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apMode = true;
  staRetrying = false;
  staRetryAt = millis() + STA_RETRY_EVERY_MS;
  Serial.printf("Hotspot %s started, open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void stopAccessPoint() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  apMode = false;
  staDownSince = 0;
  Serial.print("Back on home Wi-Fi: "); Serial.println(WiFi.localIP());
}

void manageWifi(uint32_t now) {
  if (forceApRequest) {
    forceApRequest = false;
    if (!apMode) startAccessPoint();
    return;
  }
  if (!apMode) {
    if (WiFi.isConnected()) { staDownSince = 0; return; }
    if (!staDownSince) staDownSince = now;
    else if (now - staDownSince > STA_LOST_TO_AP_MS) startAccessPoint();
    return;
  }
  // Hotspot mode
  int phones = WiFi.softAPgetStationNum();
  if (WiFi.isConnected()) {
    staRetrying = false;
    if (phones == 0) stopAccessPoint();  // home Wi-Fi is back and nobody is using the hotspot
    return;
  }
  if (staRetrying) {
    if (now - staRetryStarted > STA_RETRY_WINDOW_MS) {
      WiFi.disconnect(false);
      staRetrying = false;
      staRetryAt = now + STA_RETRY_EVERY_MS;
    }
  } else if ((int32_t)(now - staRetryAt) >= 0) {
    if (phones == 0) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      staRetrying = true;
      staRetryStarted = now;
    } else {
      staRetryAt = now + STA_RETRY_EVERY_MS;  // someone is driving: don't disturb them
    }
  }
}

// -----------------------------------------
// SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), onEcho, CHANGE);

  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // The servo channel is attached here but gets no pulses until loop() runs,
  // i.e. after camera + Wi-Fi have started (avoids the boot brownout).
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, 50, 14);
#else
  ledcSetup(4, 50, 14);
  ledcAttachPin(SERVO_PIN, 4);
#endif

  driveMotors('S', 0);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (oledOk) display.setTextColor(SSD1306_WHITE);
  oledMessage("Booting...", "Starting camera");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 12;
  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;  // always stream the newest frame (lower lag)
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
    config.frame_size = FRAMESIZE_QVGA;     // DRAM is tiny without PSRAM
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }
  esp_err_t camErr = esp_camera_init(&config);
  camOk = (camErr == ESP_OK);
  if (!camOk) Serial.printf("Camera init failed: 0x%x\n", camErr);

  oledMessage("Connecting Wi-Fi", WIFI_SSID);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.waitForConnectResult(STA_CONNECT_MS) == WL_CONNECTED) {
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    oledMessage("Home Wi-Fi not found", "Starting hotspot...");
    startAccessPoint();
  }

  startServers();

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.onStart([]() {
    // Make sure nothing moves while the flash is being rewritten
    mode = MODE_MANUAL;
    manualCmd = 'S';
    driveMotors('S', 0);
    appliedCmd = 'S';
    oledMessage("OTA update", "Starting...");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int lastPct = -1;
    int pct = total ? (done * 100) / total : 0;
    if (pct / 10 == lastPct / 10 || !oledOk) return;
    lastPct = pct;
    char line[20];
    snprintf(line, sizeof(line), "%d%%", pct);
    oledMessage("OTA update", line);
  });
  ArduinoOTA.onEnd([]() { oledMessage("OTA update", "Done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) {
    char line[20];
    snprintf(line, sizeof(line), "Error %u", (unsigned)e);
    oledMessage("OTA failed", line);
  });
  ArduinoOTA.begin();
  MDNS.addService("http", "tcp", 80);  // http://techspassion-lite-robot.local
}

// -----------------------------------------
// LOOP
// -----------------------------------------
// Servo sweep 30..150 deg shared by Radar Scan and the autopilot. Each step
// waits for fresh sonar pings taken after the servo moved (~120 ms/step,
// ~1.6 s per sweep) and stores the raw reading; the UI draws it live.
void startSweep(uint32_t now) {
  for (int i = 0; i < RADAR_POINTS; i++) radarData[i] = 0;
  radarCount = 0;
  panTarget = RADAR_START;
  sweepSeq = sonarSeq;
  sweepMoveMs = now;
}

bool stepSweep(uint32_t now) {
  if (radarCount >= RADAR_POINTS) return true;
  // The first step swings up to 60 deg from center, so let it settle longer.
  // One ping may already be in flight when the servo moves, hence +1.
  uint32_t need = radarCount == 0 ? 4 : 2;
  if (sonarSeq - sweepSeq >= need || now - sweepMoveMs > SWEEP_STEP_MAX_MS) {
    radarData[radarCount] = (int)lastRawCm;
    radarCount = radarCount + 1;
    if (radarCount >= RADAR_POINTS) {
      radarSeq = radarSeq + 1;
      return true;
    }
    panTarget = RADAR_START + radarCount * RADAR_STEP;
    sweepSeq = sonarSeq;
    sweepMoveMs = now;
  }
  return false;
}

void resetAutopilot(uint32_t now) {
  apState = AP_DRIVE;
  glancePhase = GL_CENTER1;
  panTarget = PAN_CENTER;
  apSeq = sonarSeq;
  frontSeq = sonarSeq;
  frontCm = distanceCm;
  veerUntil = 0;
  stuckRef = frontCm;
  stuckSince = now;
  for (int i = 0; i < 4; i++) turnTimes[i] = 0;
}

void applyModeRequest(uint32_t now) {
  char req = pendingMode;
  if (!req) return;
  pendingMode = 0;
  if (req == 'A') {
    mode = MODE_AUTO;
    resetAutopilot(now);
  } else if (req == 'R') {
    mode = MODE_RADAR;
    startSweep(now);
  } else if (req == 'M' && mode != MODE_MANUAL) {
    mode = MODE_MANUAL;
    panTarget = PAN_CENTER;
  }
}

void runRadar(uint32_t now) {
  applyDrive('S');
  if (stepSweep(now)) {
    mode = MODE_MANUAL;
    panTarget = PAN_CENTER;
  }
}

// ----- Autopilot helpers -----

// Ease off from full speed at AUTO_SLOW_CM down to AUTO_MIN_SPEED at the stop line.
int cruiseSpeed() {
  int top = motorSpeed;
  if (frontCm <= 0 || frontCm >= AUTO_SLOW_CM || top <= AUTO_MIN_SPEED) return top;
  float t = (frontCm - AUTO_OBSTACLE_CM) / (AUTO_SLOW_CM - AUTO_OBSTACLE_CM);
  t = constrain(t, 0.0f, 1.0f);
  return AUTO_MIN_SPEED + (int)(t * (top - AUTO_MIN_SPEED));
}

int turnSpeed() { return max((int)motorSpeed, AUTO_TURN_SPEED); }

// Turning left/right in robot terms for a servo angle, honoring PAN_LEFT_IS_HIGH.
char dirForAngle(int servoAngle) {
  int off = (servoAngle - PAN_CENTER) * PAN_LEFT_SIGN;
  return off > 0 ? 'L' : 'R';
}

void startTurn(uint32_t now, char dir, int degrees, float needCm) {
  apTurnDir = dir;
  apTurnBudget = (uint32_t)(degrees * AP_MS_PER_DEG * 255.0f / turnSpeed());
  apTurnUsed = 0;
  apBurstMs = max((uint32_t)(apTurnBudget * 0.7f), AP_TURN_BURST_MS);  // first burst undershoots on purpose
  apNeedCm = needCm;
  panTarget = PAN_CENTER;
  turnTimes[turnIdx] = now;
  turnIdx = (turnIdx + 1) % 4;
  applyDriveAt(dir, turnSpeed());
  apTimer = now;
  apState = AP_TURN;
}

void startReverse(uint32_t now, bool thenTurn) {
  apTurnAfterReverse = thenTurn;
  panTarget = PAN_CENTER;
  applyDriveAt('B', turnSpeed());
  apTimer = now;
  apState = AP_REVERSE;
}

void startScan(uint32_t now) {
  applyDrive('S');
  startSweep(now);
  apState = AP_SCAN;
}

bool trappedInCorner(uint32_t now) {
  // All of the last 4 turns happened recently -> we're ping-ponging
  for (int i = 0; i < 4; i++) {
    if (turnTimes[i] == 0 || now - turnTimes[i] > AP_OSC_WINDOW_MS) return false;
  }
  return true;
}

// Pick the heading with the most room, judged over a 30 deg window so the
// whole robot fits (not just the narrow sonar beam), preferring small turns.
void chooseHeading(uint32_t now) {
  int best = -1;
  float bestScore = -1e9, bestOpen = 0;
  for (int i = 0; i < RADAR_POINTS; i++) {
    int angle = RADAR_START + i * RADAR_STEP;
    int off = abs(angle - PAN_CENTER);
    if (off < 20) continue;  // straight ahead is what just blocked us
    float open = orMax(radarData[i]);
    if (i > 0) open = min(open, orMax(radarData[i - 1]));
    if (i < RADAR_POINTS - 1) open = min(open, orMax(radarData[i + 1]));
    float score = min(open, 200.0f) - off * 0.4f;
    if (score > bestScore) { bestScore = score; best = i; bestOpen = open; }
  }

  if (trappedInCorner(now)) {
    // Stop oscillating: back out and do a big turn one way
    for (int i = 0; i < 4; i++) turnTimes[i] = 0;
    apTurnDir = best >= 0 ? dirForAngle(RADAR_START + best * RADAR_STEP) : 'L';
    startReverse(now, true);
    return;
  }
  if (best < 0 || bestOpen < AUTO_MIN_OPEN_CM) {
    // Boxed in: back out, then turn around toward whichever side was more open
    apTurnDir = best >= 0 ? dirForAngle(RADAR_START + best * RADAR_STEP) : 'L';
    startReverse(now, true);
    return;
  }
  int angle = RADAR_START + best * RADAR_STEP;
  startTurn(now, dirForAngle(angle), abs(angle - PAN_CENTER), constrain(bestOpen * 0.8f, AUTO_MIN_OPEN_CM, AUTO_CLEAR_CM));
}

void autoDrive(uint32_t now) {
  bool settled = sonarSeq - apSeq >= 2;  // a full ping since the servo last moved
  bool atCenter = glancePhase == GL_CENTER1 || glancePhase == GL_CENTER2;

  // Straight ahead: judge every new ping while looking forward
  if (atCenter && settled && sonarSeq != frontSeq) {
    frontSeq = sonarSeq;
    frontCm = lastRawCm;
    if (frontCm > 0 && frontCm < AUTO_OBSTACLE_CM) {
      startScan(now);
      return;
    }
    // Stuck: we're driving but the wall ahead isn't getting any closer
    if (frontCm > 0 && frontCm < 200) {
      if (fabsf(frontCm - stuckRef) > AP_STUCK_TOL_CM) { stuckRef = frontCm; stuckSince = now; }
      else if (now - stuckSince > AP_STUCK_MS) { startReverse(now, false); return; }
    } else {
      stuckRef = frontCm;
      stuckSince = now;
    }
  }

  bool pathOpen = frontCm <= 0 || frontCm >= AUTO_SLOW_CM;
  if (!pathOpen && !atCenter) {
    // Something ahead: stop glancing and keep the sensor forward
    glancePhase = GL_CENTER1;
    panTarget = PAN_CENTER;
    apSeq = sonarSeq;
  } else if (settled && pathOpen) {
    // Finished this glance position: act on a side reading, then move on
    if (!atCenter) {
      float side = lastRawCm;
      if (side > 0 && side < AUTO_SIDE_CM) {
        veerCmd = glancePhase == GL_LEFT ? 'r' : 'l';  // curve away from it
        veerUntil = now + AP_VEER_MS;
      }
    }
    glancePhase = (glancePhase + 1) % 4;
    int off = glancePhase == GL_LEFT ? GLANCE_DEG : glancePhase == GL_RIGHT ? -GLANCE_DEG : 0;
    panTarget = PAN_CENTER + off * PAN_LEFT_SIGN;
    apSeq = sonarSeq;
  }

  char cmd = (int32_t)(veerUntil - now) > 0 ? veerCmd : 'F';
  applyDriveAt(cmd, cruiseSpeed());
}

void runAutopilot(uint32_t now) {
  switch (apState) {
    case AP_DRIVE:
      autoDrive(now);
      break;
    case AP_SCAN:
      applyDrive('S');
      if (stepSweep(now)) chooseHeading(now);
      break;
    case AP_TURN:
      if (now - apTimer >= apBurstMs) {
        apTurnUsed += apBurstMs;
        applyDrive('S');
        apSeq = sonarSeq;
        apState = AP_TURN_CHECK;
      }
      break;
    case AP_TURN_CHECK: {
      // Closed loop: pause, take a fresh reading, keep turning until the way is clear
      if (sonarSeq - apSeq < 2) break;
      float d = lastRawCm;
      bool clear = d <= 0 || d >= apNeedCm;
      if (clear || apTurnUsed >= apTurnBudget * 2) {
        resetAutopilot(now);
      } else {
        apBurstMs = (uint32_t)(AP_TURN_BURST_MS * 255.0f / turnSpeed());
        applyDriveAt(apTurnDir, turnSpeed());
        apTimer = now;
        apState = AP_TURN;
      }
      break;
    }
    case AP_REVERSE:
      if (now - apTimer >= AP_REVERSE_MS) {
        applyDrive('S');
        if (apTurnAfterReverse) {
          apTurnAfterReverse = false;
          startTurn(now, apTurnDir, 150, AUTO_CLEAR_CM);  // turn around
        } else {
          startScan(now);
        }
      }
      break;
  }
}

void runManual() {
  char cmd = manualCmd;
  // Dead-man: the UI re-sends held commands every 200 ms. Signed compare
  // because lastCmdMs is written by the HTTP task and can be "ahead" of now.
  if (cmd != 'S' && (int32_t)(millis() - lastCmdMs) > (int32_t)CMD_TIMEOUT_MS) {
    manualCmd = 'S';
    cmd = 'S';
  }
  bool blocked = cmd == 'F' && obstacleWithin(MANUAL_STOP_CM);
  forwardBlocked = blocked;
  applyDrive(blocked ? 'S' : cmd);
}

int servoWritten = -1;
uint32_t lastOledMs = 0;

void loop() {
  ArduinoOTA.handle();
  uint32_t now = millis();

  applyModeRequest(now);
  updateSonar(now);

  if (mode == MODE_RADAR) runRadar(now);
  else if (mode == MODE_AUTO) runAutopilot(now);
  if (mode == MODE_MANUAL) runManual();
  else forwardBlocked = false;

  int pan = panTarget;
  if (pan != servoWritten) {
    setServoAngle(pan);
    servoWritten = pan;
  }

  if (now - lastOledMs >= OLED_INTERVAL_MS) {
    lastOledMs = now;
    manageWifi(now);
    drawOled();
  }

  delay(1);
}
