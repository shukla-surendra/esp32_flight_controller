/*
========================================================
ESP32 DRONE — MULTI-SENSOR VERSION
MPU6050 + BMP280 + LSM303DLHC | Live PID Tuning
========================================================
SENSORS:
  MPU6050    — roll/pitch via complementary filter
  BMP280     — altitude hold PID (skipped gracefully if absent)
  LSM303DLHC — compass yaw heading hold (skipped if absent)

WEB ENDPOINTS:
  /        — control page with PID tuning panel
  /status  — JSON telemetry (200 ms poll)
  /control — throttle/attitude/mode commands
  /pid     — GET current PID gains as JSON
  /setpid  — GET with query params to update gains live

TUNING ORDER (bench, NO props):
  1. Roll/Pitch P — raise until oscillation, back off 30%
  2. Roll/Pitch D — dampen oscillation; stop before buzz
  3. Roll/Pitch I — very small; corrects steady lean only
  4. Yaw P/I      — stop yaw drift
  5. Alt P/I/D    — only with alt hold enabled + BMP280 present

WIRING (all sensors share same I2C bus):
  SDA->GPIO18   SCL->GPIO19   VCC->3.3V   GND->GND
  MPU6050 0x68 | BMP280 0x76 | LSM303 mag 0x1E

NOTE — motor magnetic interference:
  ESC/motor fields corrupt the magnetometer at close range.
  Yaw heading mode is experimental. Calibrate with motors off.
  Rate mode (default) needs no magnetometer.
========================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// ======================================================
// WIFI
// ======================================================

const char* ssid     = "ESP32-DRONE";
const char* password = "12345678";

WebServer server(80);

// ======================================================
// I2C
// ======================================================

#define SDA_PIN 18
#define SCL_PIN 19

// ======================================================
// SENSOR ADDRESSES
// ======================================================

#define BMP_ADDR     0x76   // try 0x77 if not detected (SDO HIGH)
#define LSM_MAG_ADDR 0x1E   // LSM303DLHC magnetometer, driven via Wire directly

// ======================================================
// SENSOR OBJECTS
// ======================================================

Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp;

bool bmpOk    = false;
bool lsmMagOk = false;

// ======================================================
// ESC PINS
// ======================================================

#define FL_PIN 25
#define FR_PIN 26
#define RL_PIN 27
#define RR_PIN 14

// ======================================================
// ESC SETTINGS
// ======================================================

#define MIN_THROTTLE 1000
#define ARM_THROTTLE 1050
#define MAX_THROTTLE 2000

// ======================================================
// MOTOR OBJECTS
// ======================================================

Servo motorFL, motorFR, motorRL, motorRR;

// ======================================================
// CONTROL SETPOINTS
// ======================================================

int   throttle      = 1000;
float pitchSetpoint = 0.0f;
float rollSetpoint  = 0.0f;
float yawSetpoint   = 0.0f;   // rate mode: deg/s | heading mode: 0-360 deg
float altSetpoint   = 0.0f;   // target relative altitude (m)

bool altHoldEnabled    = false;
bool yawHeadingEnabled = false;

// ======================================================
// PID GAINS — ROLL / PITCH (angle mode)
// P acts on angle error (deg)
// D acts on filtered gyro rate (deg/s) — no derivative kick
// ======================================================

float pid_p_gain_roll  = 2.0f;
float pid_i_gain_roll  = 0.01f;
float pid_d_gain_roll  = 0.5f;

float pid_p_gain_pitch = 2.0f;
float pid_i_gain_pitch = 0.01f;
float pid_d_gain_pitch = 0.5f;

// ======================================================
// PID GAINS — YAW
// Rate mode (default): P/I on gyro yaw rate error (deg/s)
// Heading mode: P/I on compass heading error (deg)
// No D — differentiating yaw gyro amplifies noise.
// ======================================================

float pid_p_gain_yaw = 2.0f;
float pid_i_gain_yaw = 0.02f;

// ======================================================
// PID GAINS — ALTITUDE HOLD (requires BMP280)
// P/I/D on relative altitude error (meters).
// Output is a throttle trim added to base throttle (µs).
// ======================================================

float pid_p_gain_alt = 1.5f;
float pid_i_gain_alt = 0.05f;
float pid_d_gain_alt = 0.8f;

// ======================================================
// FILTER COEFFICIENTS (tunable via /setpid)
// ======================================================

float gyro_lpf_alpha = 0.7f;   // 0.0=max filter (lag), 1.0=no filter (noisy)
float comp_alpha     = 0.98f;  // 0.95–0.99: higher=trust gyro more

// ======================================================
// PID STATE
// ======================================================

float pid_i_mem_roll  = 0.0f, pid_output_roll  = 0.0f;
float pid_i_mem_pitch = 0.0f, pid_output_pitch = 0.0f;
float pid_i_mem_yaw   = 0.0f, pid_output_yaw   = 0.0f;
float pid_i_mem_alt   = 0.0f, pid_output_alt   = 0.0f;
float alt_prev_error  = 0.0f;

// ======================================================
// GYRO / ANGLE ESTIMATION
// ======================================================

float gyro_roll_input  = 0.0f;
float gyro_pitch_input = 0.0f;
float gyro_yaw_input   = 0.0f;

float gyro_roll_filtered  = 0.0f;
float gyro_pitch_filtered = 0.0f;
float gyro_yaw_filtered   = 0.0f;

float angle_roll  = 0.0f;
float angle_pitch = 0.0f;

float gyro_roll_cal  = 0.0f;
float gyro_pitch_cal = 0.0f;
float gyro_yaw_cal   = 0.0f;

float accel_roll_offset  = 0.0f;   // measured at rest, subtracted every loop
float accel_pitch_offset = 0.0f;

// ======================================================
// ALTITUDE — BMP280
// ======================================================

float altitude_abs  = 0.0f;
float altitude_home = 0.0f;
float altitude_rel  = 0.0f;
float pressure_hpa  = 0.0f;

#define SEA_LEVEL_HPA   1013.25f
#define BMP_INTERVAL_MS 100
unsigned long last_bmp_time = 0;

// ======================================================
// HEADING — LSM303DLHC MAGNETOMETER (direct I2C)
// Register order on this chip is X, Z, Y — not X, Y, Z.
// Gain: XY = 1100 LSB/G, Z = 980 LSB/G  (CRB=0x20, ±1.3G)
// ======================================================

float   heading = 0.0f;
int16_t rawMagX = 0;
int16_t rawMagY = 0;
int16_t rawMagZ = 0;

#define MAG_INTERVAL_MS 100
unsigned long last_mag_time = 0;

// ======================================================
// MOTOR SPEEDS (global so /status can read them)
// ======================================================

int motorFL_speed = MIN_THROTTLE;
int motorFR_speed = MIN_THROTTLE;
int motorRL_speed = MIN_THROTTLE;
int motorRR_speed = MIN_THROTTLE;

// ======================================================
// LOOP TIMING
// ======================================================

unsigned long last_time = 0;
float dt = 0.01f;

// ======================================================
// TAKEOFF DETECTION
// ======================================================

bool          isFlying    = false;
unsigned long flyingTimer = 0;

#define TAKEOFF_THROTTLE        1200
#define TAKEOFF_CONFIRM_MS      500
#define TAKEOFF_ACCEL_THRESHOLD 1.5f

// ======================================================
// HTML PAGE
// ======================================================

const char webpage[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{text-align:center;font-family:Arial;background:#1a1a1a;color:#eee;padding-bottom:20px}
h2{color:#4af;margin:10px 0}
button{width:120px;height:50px;font-size:15px;margin:5px;background:#333;color:#eee;border:1px solid #555;border-radius:6px;cursor:pointer}
button:active{background:#555}
#stop-btn{background:#700;color:#fff;border-color:#f00;width:140px}
#status-bar{font-size:22px;font-weight:bold;margin:8px auto;padding:8px;border-radius:6px;background:#222;width:220px}
.flying{color:#0f0}.grounded{color:#fa0}
#info-bar{font-size:13px;color:#aaa;margin:4px}
.row{margin:5px}
details{margin:10px auto;max-width:580px;text-align:left;background:#222;border-radius:6px;padding:10px}
summary{cursor:pointer;color:#4af;font-size:15px;font-weight:bold}
.pid-row{margin:6px 0;font-size:13px}
input[type=number]{width:62px;background:#333;color:#eee;border:1px solid #555;border-radius:4px;padding:3px;font-size:13px;margin:0 2px}
.mode-btn{width:155px;height:44px;font-size:13px;background:#334;border-color:#44f}
.mode-btn.on{background:#050;border-color:#0f0;color:#0f0}
#pid-msg{color:#0f0;font-size:13px;margin-left:8px;vertical-align:middle}
.warn{color:#fa0;font-size:12px;margin:4px 0}
#log{text-align:left;background:#111;color:#0f0;padding:12px;margin:10px auto;width:90%;max-width:560px;border-radius:6px;font-size:13px;min-height:200px;white-space:pre-wrap}
.guide-text{font-size:13px;color:#ccc;line-height:1.7;padding:4px 0}
.guide-text b{color:#eee}
</style>
</head>
<body>
<h2>ESP32 Drone</h2>
<div id="status-bar" class="grounded">GROUNDED</div>
<div id="info-bar">Alt: -- m &nbsp;|&nbsp; Heading: -- deg</div>

<div class="row">
  <button onclick="send('throttle_up')">Throttle +</button>
  <button onclick="send('throttle_down')">Throttle -</button>
</div>
<div class="row"><button onclick="send('forward')">Forward</button></div>
<div class="row">
  <button onclick="send('yaw_left')">Yaw L</button>
  <button onclick="send('left')">Left</button>
  <button onclick="send('center')">Center</button>
  <button onclick="send('right')">Right</button>
  <button onclick="send('yaw_right')">Yaw R</button>
</div>
<div class="row"><button onclick="send('backward')">Backward</button></div>
<div class="row"><button id="stop-btn" onclick="send('stop')">STOP</button></div>

<div class="row">
  <button class="mode-btn" id="alt-btn" onclick="toggleMode('alt_hold')">Alt Hold: OFF</button>
  <button class="mode-btn" id="yaw-btn" onclick="toggleMode('yaw_heading')">Yaw Lock: OFF</button>
</div>
<div class="row" id="alt-adj" style="display:none">
  <button onclick="send('alt_up')" style="width:110px">Alt +0.5m</button>
  <button onclick="send('alt_down')" style="width:110px">Alt -0.5m</button>
</div>

<details>
<summary>PID Tuning</summary>
<div style="padding:6px">
<p class="warn">Values apply instantly and reset on power cycle. Zero throttle before large changes.</p>
<div class="pid-row"><b style="display:inline-block;width:50px">Roll</b>
  P:<input type="number" id="rp" step="0.1" min="0">
  I:<input type="number" id="ri" step="0.001" min="0">
  D:<input type="number" id="rd" step="0.1" min="0">
</div>
<div class="pid-row"><b style="display:inline-block;width:50px">Pitch</b>
  P:<input type="number" id="pp" step="0.1" min="0">
  I:<input type="number" id="pi" step="0.001" min="0">
  D:<input type="number" id="pd" step="0.1" min="0">
</div>
<div class="pid-row"><b style="display:inline-block;width:50px">Yaw</b>
  P:<input type="number" id="yp" step="0.1" min="0">
  I:<input type="number" id="yi" step="0.001" min="0">
</div>
<div class="pid-row"><b style="display:inline-block;width:50px">Alt</b>
  P:<input type="number" id="ap" step="0.1" min="0">
  I:<input type="number" id="ai" step="0.001" min="0">
  D:<input type="number" id="ad" step="0.1" min="0">
</div>
<div class="pid-row"><b style="display:inline-block;width:50px">Filters</b>
  Gyro LPF:<input type="number" id="lpf" step="0.05" min="0.1" max="1.0">
  Comp:<input type="number" id="comp" step="0.01" min="0.9" max="0.99">
</div>
<button onclick="loadPID()" style="width:90px;height:36px;font-size:13px">Load</button>
<button onclick="applyPID()" style="width:90px;height:36px;font-size:13px">Apply</button>
<span id="pid-msg"></span>
</div>
</details>

<details>
<summary>Tuning Guide</summary>
<div class="guide-text" style="padding:6px">
<p><b>STEP 1 — Roll/Pitch P (start ~1.5)</b><br>
No props. Raise P until drone body twitches or oscillates rapidly. Back off 30%. Typical: 1.0 – 4.0</p>
<p><b>STEP 2 — Roll/Pitch D (start ~0.3)</b><br>
Raise D until oscillations after a tap are damped. Stop if high-pitch buzz appears (too much D). Typical: 0.1 – 1.5</p>
<p><b>STEP 3 — Roll/Pitch I (start ~0.005)</b><br>
Add last. Corrects slow steady lean. Too high = slow wobbly oscillation. Typical: 0.001 – 0.05</p>
<p><b>STEP 4 — Yaw P/I</b><br>
P stops spin drift. I corrects slow residual rotation. Keep small. Typical P: 1–3, I: 0.01–0.05</p>
<p><b>STEP 5 — Altitude P/I/D (with Alt Hold ON + BMP280)</b><br>
P=1.5 is safe start. D smooths bouncing. I corrects slow climb or sink.</p>
<p><b>Gyro LPF Alpha (default 0.7)</b><br>
0.4 = heavy filter, more lag. 0.9 = light filter, more noise. Adjust if D term causes buzz.</p>
<p><b>Complementary Filter (default 0.98)</b><br>
Higher = trusts gyro more (faster response, more drift). Lower = more accel correction. Range: 0.95–0.99</p>
<p class="warn">ALWAYS test on bench without propellers first!</p>
<p class="warn">If drone corrects in wrong direction: negate the pid_output_roll or pid_output_pitch sign in the motor mixer.</p>
</div>
</details>

<div class="row">
  <button onclick="downloadLog()" style="width:155px;height:38px;font-size:13px">Download CSV</button>
  <button onclick="clearLog()" style="width:120px;height:38px;font-size:13px">Clear Log</button>
</div>

<pre id="log">Waiting for data...</pre>

<script>
const MAX_ENTRIES = 5000;
let logs = JSON.parse(localStorage.getItem("drone_log") || "[]");

function send(cmd) { fetch("/control?cmd=" + cmd); }

function toggleMode(mode) { fetch("/control?cmd=" + mode); }

function clearLog() {
  logs = [];
  localStorage.removeItem("drone_log");
  document.getElementById("log").textContent = "Log cleared.";
}

function downloadLog() {
  if (!logs.length) { alert("No log data."); return; }
  const hdr = "time,throttle,angle_roll,angle_pitch,gyro_roll,gyro_pitch,yaw_rate,altitude,heading,pid_roll,pid_pitch,pid_yaw,pid_alt,fl,fr,rl,rr,flying,alt_hold,yaw_heading\n";
  const rows = logs.map(e =>
    [e.t,e.thr,e.ar,e.ap,e.gr,e.gp,e.yr,e.alt,e.hdg,
     e.pr,e.pp,e.py,e.pa,e.fl,e.fr,e.rl,e.rr,
     e.fly,e.ah,e.yh].join(",")
  ).join("\n");
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([hdr+rows],{type:"text/csv"}));
  a.download = "drone_" + new Date().toISOString().slice(0,19).replace(/:/g,"-") + ".csv";
  a.click();
}

function loadPID() {
  fetch("/pid").then(r=>r.json()).then(d=>{
    ["rp","ri","rd","pp","pi","pd","yp","yi","ap","ai","ad","lpf","comp"]
      .forEach(k=>{ if(d[k]!==undefined) document.getElementById(k).value=d[k]; });
    showMsg("Loaded");
  });
}

function applyPID() {
  const ids = ["rp","ri","rd","pp","pi","pd","yp","yi","ap","ai","ad","lpf","comp"];
  const qs = ids.map(k=>k+"="+document.getElementById(k).value).join("&");
  fetch("/setpid?"+qs).then(()=>showMsg("Applied!"));
}

function showMsg(txt) {
  const el = document.getElementById("pid-msg");
  el.textContent = txt;
  setTimeout(()=>el.textContent="", 2500);
}

setInterval(function() {
  fetch("/status").then(r=>r.json()).then(d=>{
    const e = {
      t:new Date().toISOString(),
      thr:d.throttle, ar:d.angle_roll, ap:d.angle_pitch,
      gr:d.gyro_roll, gp:d.gyro_pitch, yr:d.yaw_rate,
      alt:d.altitude, hdg:d.heading,
      pr:d.pid_roll, pp:d.pid_pitch, py:d.pid_yaw, pa:d.pid_alt,
      fl:d.fl, fr:d.fr, rl:d.rl, rr:d.rr,
      fly:d.flying, ah:d.alt_hold, yh:d.yaw_heading
    };
    logs.push(e);
    if(logs.length>MAX_ENTRIES) logs.shift();
    localStorage.setItem("drone_log", JSON.stringify(logs));

    const sb = document.getElementById("status-bar");
    sb.textContent = d.flying ? "FLYING" : "GROUNDED";
    sb.className   = d.flying ? "flying" : "grounded";

    document.getElementById("info-bar").textContent =
      "Alt: " + d.altitude.toFixed(1) + " m  |  Heading: " + d.heading.toFixed(0) + " deg";

    const ab = document.getElementById("alt-btn");
    ab.textContent = "Alt Hold: " + (d.alt_hold?"ON":"OFF");
    ab.className   = "mode-btn" + (d.alt_hold?" on":"");
    document.getElementById("alt-adj").style.display = d.alt_hold?"block":"none";

    const yb = document.getElementById("yaw-btn");
    yb.textContent = "Yaw Lock: " + (d.yaw_heading?"ON":"OFF");
    yb.className   = "mode-btn" + (d.yaw_heading?" on":"");

    document.getElementById("log").textContent =
      "Time      : " + e.t + "\n" +
      "Flying    : " + (d.flying?"YES":"NO") + "\n" +
      "Throttle  : " + d.throttle + "\n" +
      "Roll      : " + d.angle_roll.toFixed(1)  + " deg   Pitch: " + d.angle_pitch.toFixed(1) + " deg\n" +
      "Gyro Roll : " + d.gyro_roll.toFixed(1)   + "       Pitch: " + d.gyro_pitch.toFixed(1) + "\n" +
      "Yaw Rate  : " + d.yaw_rate.toFixed(1)    + " deg/s\n" +
      "Altitude  : " + d.altitude.toFixed(2)    + " m (rel)  AltHold: " + (d.alt_hold?"ON":"OFF") + "\n" +
      "Heading   : " + d.heading.toFixed(1)     + " deg      YawLock: " + (d.yaw_heading?"ON":"OFF") + "\n" +
      "PID Roll  : " + d.pid_roll.toFixed(2)    + "  Pitch: " + d.pid_pitch.toFixed(2) + "\n" +
      "PID Yaw   : " + d.pid_yaw.toFixed(2)     + "  Alt:   " + d.pid_alt.toFixed(2) + "\n" +
      "FL: " + d.fl + "  FR: " + d.fr + "\n" +
      "RL: " + d.rl + "  RR: " + d.rr + "\n" +
      "Entries   : " + logs.length + " / 5000";
  }).catch(()=>{
    document.getElementById("log").textContent = "Connection lost...";
  });
}, 200);
</script>
</body>
</html>
)rawhtml";

// ======================================================
// MOTOR WRITE
// ======================================================

void writeMotor(Servo &motor, int us)
{
  us = constrain(us, MIN_THROTTLE, MAX_THROTTLE);
  motor.writeMicroseconds(us);
}

// ======================================================
// GYRO CALIBRATION
// ======================================================

void calibrateGyro()
{
  Serial.println("================================");
  Serial.println("KEEP DRONE STILL — CALIBRATING");
  Serial.println("================================");

  sensors_event_t a, g, temp;
  float roll_sum = 0, pitch_sum = 0, yaw_sum = 0;

  for (int i = 0; i < 2000; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    roll_sum  += g.gyro.x;
    pitch_sum += g.gyro.y;
    yaw_sum   += g.gyro.z;
    delay(2);
  }

  gyro_roll_cal  = roll_sum  / 2000.0f;
  gyro_pitch_cal = pitch_sum / 2000.0f;
  gyro_yaw_cal   = yaw_sum   / 2000.0f;

  // Measure resting accel angles so the complementary filter starts at true level.
  // The drone MUST be sitting flat and still for this to be accurate.
  float ar_sum = 0, ap_sum = 0;
  for (int i = 0; i < 500; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    ar_sum += atan2f(a.acceleration.y, a.acceleration.z) * 57.2958f;
    ap_sum += atan2f(-a.acceleration.x, a.acceleration.z) * 57.2958f;
    delay(2);
  }
  accel_roll_offset  = ar_sum / 500.0f;
  accel_pitch_offset = ap_sum / 500.0f;

  Serial.println("CALIBRATION COMPLETE");
  Serial.print("Gyro roll offset:   "); Serial.println(gyro_roll_cal,   6);
  Serial.print("Gyro pitch offset:  "); Serial.println(gyro_pitch_cal,  6);
  Serial.print("Gyro yaw offset:    "); Serial.println(gyro_yaw_cal,    6);
  Serial.print("Accel roll offset:  "); Serial.println(accel_roll_offset,  2);
  Serial.print("Accel pitch offset: "); Serial.println(accel_pitch_offset, 2);
}

// ======================================================
// TAKEOFF DETECTION
// ======================================================

void detectTakeoff(float accel_z)
{
  bool condMet = (throttle > TAKEOFF_THROTTLE) &&
                 (fabsf(accel_z - 9.81f) > TAKEOFF_ACCEL_THRESHOLD);

  if (condMet)
  {
    if (flyingTimer == 0) flyingTimer = millis();
    if (millis() - flyingTimer > TAKEOFF_CONFIRM_MS) isFlying = true;
  }
  else
  {
    flyingTimer = 0;
    if (throttle <= ARM_THROTTLE) isFlying = false;
  }
}

// ======================================================
// LSM303DLHC — INIT MAGNETOMETER ONLY (direct I2C)
// CRA_REG_M 0x00 = 0x18 -> 75 Hz output rate
// CRB_REG_M 0x01 = 0x20 -> ±1.3 G gain (XY:1100, Z:980 LSB/G)
// MR_REG_M  0x02 = 0x00 -> continuous measurement
// ======================================================

bool initLSM303Mag()
{
  Wire.beginTransmission(LSM_MAG_ADDR);
  Wire.write(0x00); Wire.write(0x18);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(LSM_MAG_ADDR);
  Wire.write(0x01); Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(LSM_MAG_ADDR);
  Wire.write(0x02); Wire.write(0x00);
  Wire.endTransmission();

  return true;
}

// ======================================================
// LSM303DLHC — READ MAG REGISTERS
// Registers 0x03-0x08. IMPORTANT: chip order is X, Z, Y.
// ======================================================

void readLsmMagRaw()
{
  Wire.beginTransmission(LSM_MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)LSM_MAG_ADDR, (uint8_t)6, (uint8_t)true);

  if (Wire.available() >= 6)
  {
    rawMagX = (int16_t)((Wire.read() << 8) | Wire.read());
    rawMagZ = (int16_t)((Wire.read() << 8) | Wire.read()); // Z before Y — chip quirk
    rawMagY = (int16_t)((Wire.read() << 8) | Wire.read());
  }
}

// ======================================================
// TILT-COMPENSATED COMPASS HEADING
// Uses current roll/pitch from complementary filter
// to correct mag readings when sensor is not flat.
// ======================================================

float computeHeading(float rollDeg, float pitchDeg)
{
  float magX = rawMagX / 1100.0f;
  float magY = rawMagY / 1100.0f;
  float magZ = rawMagZ /  980.0f;

  float roll_r  = rollDeg  * (PI / 180.0f);
  float pitch_r = pitchDeg * (PI / 180.0f);

  float xh = magX * cosf(pitch_r) + magZ * sinf(pitch_r);
  float yh = magX * sinf(roll_r) * sinf(pitch_r)
           + magY * cosf(roll_r)
           - magZ * sinf(roll_r) * cosf(pitch_r);

  float h = atan2f(yh, xh) * (180.0f / PI);
  if (h < 0.0f) h += 360.0f;
  return h;
}

// ======================================================
// HEADING ERROR WITH WRAPAROUND
// Shortest angular path between current and target.
// ======================================================

float wrapHeadingError(float current, float target)
{
  float err = current - target;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ======================================================
// ROOT PAGE
// ======================================================

void handleRoot()
{
  server.send_P(200, "text/html", webpage);
}

// ======================================================
// STATUS — JSON telemetry
// ======================================================

void handleStatus()
{
  String json = "{";
  json += "\"throttle\":"     + String(throttle)              + ",";
  json += "\"angle_roll\":"   + String(angle_roll,        2)  + ",";
  json += "\"angle_pitch\":"  + String(angle_pitch,       2)  + ",";
  json += "\"gyro_roll\":"    + String(gyro_roll_input,   2)  + ",";
  json += "\"gyro_pitch\":"   + String(gyro_pitch_input,  2)  + ",";
  json += "\"yaw_rate\":"     + String(gyro_yaw_input,    2)  + ",";
  json += "\"altitude\":"     + String(altitude_rel,      2)  + ",";
  json += "\"heading\":"      + String(heading,           1)  + ",";
  json += "\"pid_roll\":"     + String(pid_output_roll,   2)  + ",";
  json += "\"pid_pitch\":"    + String(pid_output_pitch,  2)  + ",";
  json += "\"pid_yaw\":"      + String(pid_output_yaw,    2)  + ",";
  json += "\"pid_alt\":"      + String(pid_output_alt,    2)  + ",";
  json += "\"fl\":"           + String(motorFL_speed)         + ",";
  json += "\"fr\":"           + String(motorFR_speed)         + ",";
  json += "\"rl\":"           + String(motorRL_speed)         + ",";
  json += "\"rr\":"           + String(motorRR_speed)         + ",";
  json += "\"flying\":"       + String(isFlying          ? "true" : "false") + ",";
  json += "\"alt_hold\":"     + String(altHoldEnabled    ? "true" : "false") + ",";
  json += "\"yaw_heading\":"  + String(yawHeadingEnabled ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

// ======================================================
// CONTROL — accepts ?cmd=...
// ======================================================

void handleControl()
{
  String cmd = server.arg("cmd");

  if (cmd == "throttle_up")   throttle += 20;
  if (cmd == "throttle_down") throttle -= 20;
  if (cmd == "forward")       pitchSetpoint =  15.0f;
  if (cmd == "backward")      pitchSetpoint = -15.0f;
  if (cmd == "left")          rollSetpoint  = -15.0f;
  if (cmd == "right")         rollSetpoint  =  15.0f;
  if (cmd == "yaw_left")      yawSetpoint   = -30.0f;
  if (cmd == "yaw_right")     yawSetpoint   =  30.0f;

  if (cmd == "center")
  {
    pitchSetpoint = 0.0f;
    rollSetpoint  = 0.0f;
    yawSetpoint   = yawHeadingEnabled ? heading : 0.0f;
  }

  if (cmd == "stop")
  {
    throttle      = MIN_THROTTLE;
    pitchSetpoint = 0.0f;
    rollSetpoint  = 0.0f;
    yawSetpoint   = 0.0f;
    altHoldEnabled    = false;
    pid_output_alt    = 0.0f;
    pid_i_mem_alt     = 0.0f;
  }

  // Altitude hold toggle — locks current altitude as target
  if (cmd == "alt_hold")
  {
    if (!bmpOk)
    {
      server.send(200, "text/plain", "NO BMP280");
      return;
    }
    altHoldEnabled = !altHoldEnabled;
    if (altHoldEnabled)
    {
      altSetpoint   = altitude_rel;
      pid_i_mem_alt = 0.0f;
      alt_prev_error = 0.0f;
    }
    else
    {
      pid_output_alt = 0.0f;
      pid_i_mem_alt  = 0.0f;
    }
  }

  // Yaw heading lock toggle — locks current compass heading
  if (cmd == "yaw_heading")
  {
    if (!lsmMagOk)
    {
      server.send(200, "text/plain", "NO LSM303");
      return;
    }
    yawHeadingEnabled = !yawHeadingEnabled;
    if (yawHeadingEnabled)
    {
      yawSetpoint  = heading;
      pid_i_mem_yaw = 0.0f;
    }
    else
    {
      yawSetpoint = 0.0f;
    }
  }

  // Altitude setpoint adjustment (only when alt hold active)
  if (cmd == "alt_up")   altSetpoint += 0.5f;
  if (cmd == "alt_down") altSetpoint -= 0.5f;

  throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);

  server.send(200, "text/plain", "OK");
}

// ======================================================
// PID — return all gains as JSON
// ======================================================

void handlePID()
{
  String json = "{";
  json += "\"rp\":"   + String(pid_p_gain_roll,  4) + ",";
  json += "\"ri\":"   + String(pid_i_gain_roll,  4) + ",";
  json += "\"rd\":"   + String(pid_d_gain_roll,  4) + ",";
  json += "\"pp\":"   + String(pid_p_gain_pitch, 4) + ",";
  json += "\"pi\":"   + String(pid_i_gain_pitch, 4) + ",";
  json += "\"pd\":"   + String(pid_d_gain_pitch, 4) + ",";
  json += "\"yp\":"   + String(pid_p_gain_yaw,   4) + ",";
  json += "\"yi\":"   + String(pid_i_gain_yaw,   4) + ",";
  json += "\"ap\":"   + String(pid_p_gain_alt,   4) + ",";
  json += "\"ai\":"   + String(pid_i_gain_alt,   4) + ",";
  json += "\"ad\":"   + String(pid_d_gain_alt,   4) + ",";
  json += "\"lpf\":"  + String(gyro_lpf_alpha,   4) + ",";
  json += "\"comp\":" + String(comp_alpha,        4);
  json += "}";
  server.send(200, "application/json", json);
}

// ======================================================
// SET PID — update gains via query params, then reset I-terms
// Example: /setpid?rp=2.5&rd=0.8&ri=0.01 ...
// ======================================================

void handleSetPID()
{
  if (server.hasArg("rp"))   pid_p_gain_roll  = server.arg("rp").toFloat();
  if (server.hasArg("ri"))   pid_i_gain_roll  = server.arg("ri").toFloat();
  if (server.hasArg("rd"))   pid_d_gain_roll  = server.arg("rd").toFloat();
  if (server.hasArg("pp"))   pid_p_gain_pitch = server.arg("pp").toFloat();
  if (server.hasArg("pi"))   pid_i_gain_pitch = server.arg("pi").toFloat();
  if (server.hasArg("pd"))   pid_d_gain_pitch = server.arg("pd").toFloat();
  if (server.hasArg("yp"))   pid_p_gain_yaw   = server.arg("yp").toFloat();
  if (server.hasArg("yi"))   pid_i_gain_yaw   = server.arg("yi").toFloat();
  if (server.hasArg("ap"))   pid_p_gain_alt   = server.arg("ap").toFloat();
  if (server.hasArg("ai"))   pid_i_gain_alt   = server.arg("ai").toFloat();
  if (server.hasArg("ad"))   pid_d_gain_alt   = server.arg("ad").toFloat();
  if (server.hasArg("lpf"))  gyro_lpf_alpha   = constrain(server.arg("lpf").toFloat(),  0.1f, 1.0f);
  if (server.hasArg("comp")) comp_alpha        = constrain(server.arg("comp").toFloat(), 0.90f, 0.99f);

  // Clear I-terms so old accumulated integral doesn't jump with new gain
  pid_i_mem_roll  = 0.0f;
  pid_i_mem_pitch = 0.0f;
  pid_i_mem_yaw   = 0.0f;
  pid_i_mem_alt   = 0.0f;

  server.send(200, "text/plain", "OK");
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(500);

  // ---- MPU6050 (required) ----
  if (!mpu.begin())
  {
    Serial.println("MPU6050 FAILED — check wiring!");
    while (1);
  }
  Serial.println("MPU6050  OK  (0x68)");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1500);
  calibrateGyro();

  // ---- BMP280 (optional — altitude hold) ----
  bmpOk = bmp.begin(BMP_ADDR);
  if (bmpOk)
  {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_63);
    altitude_home = bmp.readAltitude(SEA_LEVEL_HPA);
    altitude_abs  = altitude_home;
    Serial.println("BMP280   OK  (0x76) — alt hold available");
    Serial.print  ("  Home altitude: "); Serial.print(altitude_home, 1); Serial.println(" m");
  }
  else
  {
    Serial.println("BMP280   NOT FOUND — altitude hold disabled");
  }

  // ---- LSM303DLHC magnetometer (optional — yaw heading hold) ----
  lsmMagOk = initLSM303Mag();
  if (lsmMagOk)
  {
    Serial.println("LSM303   OK  (0x1E) — yaw heading lock available");
    // First reading
    readLsmMagRaw();
    heading = computeHeading(0.0f, 0.0f);
  }
  else
  {
    Serial.println("LSM303   NOT FOUND — yaw heading lock disabled");
  }

  // ---- PWM timers ----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ---- ESC setup ----
  motorFL.setPeriodHertz(50); motorFL.attach(FL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorFR.setPeriodHertz(50); motorFR.attach(FR_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRL.setPeriodHertz(50); motorRL.attach(RL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRR.setPeriodHertz(50); motorRR.attach(RR_PIN, MIN_THROTTLE, MAX_THROTTLE);

  writeMotor(motorFL, MIN_THROTTLE);
  writeMotor(motorFR, MIN_THROTTLE);
  writeMotor(motorRL, MIN_THROTTLE);
  writeMotor(motorRR, MIN_THROTTLE);

  delay(5000);

  // ---- WiFi AP ----
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP: "); Serial.println(WiFi.softAPIP());

  // ---- Web server routes ----
  server.on("/",        handleRoot);
  server.on("/status",  handleStatus);
  server.on("/control", handleControl);
  server.on("/pid",     handlePID);
  server.on("/setpid",  handleSetPID);
  server.begin();
  Serial.println("Web server ready");

  last_time     = micros();
  last_bmp_time = millis();
  last_mag_time = millis();
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  // ---- Timing ----
  unsigned long now_us = micros();
  dt = (now_us - last_time) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  last_time = now_us;

  server.handleClient();

  unsigned long now_ms = millis();

  // ---- Read MPU6050 every loop (~100 Hz) ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  detectTakeoff(a.acceleration.z);

  gyro_roll_input  = (g.gyro.x - gyro_roll_cal)  * 57.2958f;
  gyro_pitch_input = (g.gyro.y - gyro_pitch_cal)  * 57.2958f;
  gyro_yaw_input   = (g.gyro.z - gyro_yaw_cal)    * 57.2958f;

  gyro_roll_filtered  = gyro_lpf_alpha * gyro_roll_input  + (1.0f - gyro_lpf_alpha) * gyro_roll_filtered;
  gyro_pitch_filtered = gyro_lpf_alpha * gyro_pitch_input + (1.0f - gyro_lpf_alpha) * gyro_pitch_filtered;
  gyro_yaw_filtered   = gyro_lpf_alpha * gyro_yaw_input   + (1.0f - gyro_lpf_alpha) * gyro_yaw_filtered;

  float accel_roll  = atan2f(a.acceleration.y,  a.acceleration.z) * 57.2958f - accel_roll_offset;
  float accel_pitch = atan2f(-a.acceleration.x, a.acceleration.z) * 57.2958f - accel_pitch_offset;

  angle_roll  = comp_alpha * (angle_roll  + gyro_roll_filtered  * dt) + (1.0f - comp_alpha) * accel_roll;
  angle_pitch = comp_alpha * (angle_pitch + gyro_pitch_filtered * dt) + (1.0f - comp_alpha) * accel_pitch;

  // ---- Read BMP280 at 10 Hz ----
  if (bmpOk && (now_ms - last_bmp_time >= BMP_INTERVAL_MS))
  {
    float bmp_dt    = (now_ms - last_bmp_time) / 1000.0f;
    last_bmp_time   = now_ms;

    altitude_abs  = bmp.readAltitude(SEA_LEVEL_HPA);
    pressure_hpa  = bmp.readPressure() / 100.0f;
    altitude_rel  = altitude_abs - altitude_home;

    // Altitude PID — only runs when new barometer data arrives
    if (altHoldEnabled && throttle > ARM_THROTTLE)
    {
      float alt_err     = altSetpoint - altitude_rel;
      pid_i_mem_alt    += pid_i_gain_alt * alt_err * bmp_dt;
      pid_i_mem_alt     = constrain(pid_i_mem_alt, -100.0f, 100.0f);
      float alt_d       = (alt_err - alt_prev_error) / bmp_dt;
      alt_prev_error    = alt_err;

      pid_output_alt = pid_p_gain_alt * alt_err
                     + pid_i_mem_alt
                     + pid_d_gain_alt * alt_d;
      pid_output_alt = constrain(pid_output_alt, -200.0f, 200.0f);
    }
    else
    {
      pid_output_alt = 0.0f;
      pid_i_mem_alt  = 0.0f;
      alt_prev_error = 0.0f;
    }
  }

  // ---- Read LSM303 mag at 10 Hz ----
  if (lsmMagOk && (now_ms - last_mag_time >= MAG_INTERVAL_MS))
  {
    last_mag_time = now_ms;
    readLsmMagRaw();
    heading = computeHeading(angle_roll, angle_pitch);
  }

  // ---- PID ROLL (angle mode) ----
  float roll_error    = angle_roll - rollSetpoint;
  pid_i_mem_roll     += pid_i_gain_roll * roll_error * dt;
  pid_i_mem_roll      = constrain(pid_i_mem_roll, -50.0f, 50.0f);
  pid_output_roll     = pid_p_gain_roll * roll_error
                      + pid_i_mem_roll
                      - pid_d_gain_roll * gyro_roll_filtered;
  pid_output_roll     = constrain(pid_output_roll, -200.0f, 200.0f);

  // ---- PID PITCH (angle mode) ----
  float pitch_error   = angle_pitch - pitchSetpoint;
  pid_i_mem_pitch    += pid_i_gain_pitch * pitch_error * dt;
  pid_i_mem_pitch     = constrain(pid_i_mem_pitch, -50.0f, 50.0f);
  pid_output_pitch    = pid_p_gain_pitch * pitch_error
                      + pid_i_mem_pitch
                      - pid_d_gain_pitch * gyro_pitch_filtered;
  pid_output_pitch    = constrain(pid_output_pitch, -200.0f, 200.0f);

  // ---- PID YAW — rate mode or heading mode ----
  float yaw_error;
  if (yawHeadingEnabled)
    yaw_error = wrapHeadingError(heading, yawSetpoint);
  else
    yaw_error = gyro_yaw_input - yawSetpoint;

  pid_i_mem_yaw  += pid_i_gain_yaw * yaw_error * dt;
  pid_i_mem_yaw   = constrain(pid_i_mem_yaw, -50.0f, 50.0f);
  pid_output_yaw  = pid_p_gain_yaw * yaw_error + pid_i_mem_yaw;
  pid_output_yaw  = constrain(pid_output_yaw, -200.0f, 200.0f);

  // ---- Safety: zero motors below arm throttle ----
  if (throttle <= ARM_THROTTLE)
  {
    motorFL_speed = MIN_THROTTLE;
    motorFR_speed = MIN_THROTTLE;
    motorRL_speed = MIN_THROTTLE;
    motorRR_speed = MIN_THROTTLE;

    pid_i_mem_roll  = 0.0f;
    pid_i_mem_pitch = 0.0f;
    pid_i_mem_yaw   = 0.0f;
    pid_i_mem_alt   = 0.0f;
    pid_output_alt  = 0.0f;
  }
  else
  {
    // ---- Motor mixer — X-frame ----
    // FL=CCW, FR=CW, RL=CW, RR=CCW (standard X)
    // Altitude PID trims the base throttle when alt hold is active.
    // If drone corrects in the wrong direction on any axis, negate
    // the corresponding pid_output term below.
    int base = constrain(throttle + (int)pid_output_alt, MIN_THROTTLE, MAX_THROTTLE);

    motorFL_speed = base - pid_output_pitch + pid_output_roll + pid_output_yaw;
    motorFR_speed = base - pid_output_pitch - pid_output_roll - pid_output_yaw;
    motorRL_speed = base + pid_output_pitch + pid_output_roll - pid_output_yaw;
    motorRR_speed = base + pid_output_pitch - pid_output_roll + pid_output_yaw;

    motorFL_speed = constrain(motorFL_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorFR_speed = constrain(motorFR_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorRL_speed = constrain(motorRL_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorRR_speed = constrain(motorRR_speed, MIN_THROTTLE, MAX_THROTTLE);
  }

  writeMotor(motorFL, motorFL_speed);
  writeMotor(motorFR, motorFR_speed);
  writeMotor(motorRL, motorRL_speed);
  writeMotor(motorRR, motorRR_speed);

  delay(10);
}
