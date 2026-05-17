/*
========================================================
ESP32 DRONE - SAFE TEST VERSION
MPU6050 + WiFi Control + SAFE PID
========================================================

FIXES INCLUDED:
- PID disabled at low throttle
- Safer startup
- Gyro calibration
- dt-corrected I and D terms
- Complementary filter for angle estimation (self-leveling)
- Gyro low-pass filter before D term (noise suppression)
- D term on measurement — no derivative kick on setpoint change
- Yaw rate PID with motor mixer
- Browser telemetry logging with localStorage
- Takeoff detection via throttle + accelerometer Z

NOTE ON SIGNS:
If the drone corrects in the wrong direction on roll or pitch,
negate pid_output_roll or pid_output_pitch in the motor mixer.
Yaw direction depends on motor rotation order — flip pid_output_yaw
signs in the mixer if yaw correction goes the wrong way.

IMPORTANT:
REMOVE PROPELLERS FIRST
========================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
// MPU6050
// ======================================================

Adafruit_MPU6050 mpu;

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

Servo motorFL;
Servo motorFR;
Servo motorRL;
Servo motorRR;

// ======================================================
// CONTROL VARIABLES
// ======================================================

int   throttle      = 1000;
float pitchSetpoint = 0;     // target angle in degrees
float rollSetpoint  = 0;     // target angle in degrees
float yawSetpoint   = 0;     // target yaw rate in deg/s

// ======================================================
// PID GAINS
// Tuning order: P first, then D, then I.
// Start bench test (no props) before flying.
// ======================================================

// Roll / Pitch — angle mode
// P acts on angle error (deg), D acts on gyro rate (deg/s)
float pid_p_gain_roll  = 2.0;
float pid_i_gain_roll  = 0.01;
float pid_d_gain_roll  = 0.5;

float pid_p_gain_pitch = 2.0;
float pid_i_gain_pitch = 0.01;
float pid_d_gain_pitch = 0.5;

// Yaw — rate mode (no angle estimation, no magnetometer)
float pid_p_gain_yaw   = 2.0;
float pid_i_gain_yaw   = 0.02;

// ======================================================
// PID STATE
// ======================================================

float pid_i_mem_roll    = 0;
float pid_output_roll   = 0;

float pid_i_mem_pitch   = 0;
float pid_output_pitch  = 0;

float pid_i_mem_yaw     = 0;
float pid_output_yaw    = 0;

// ======================================================
// GYRO VALUES
// ======================================================

float gyro_roll_input  = 0;
float gyro_pitch_input = 0;
float gyro_yaw_input   = 0;

// ======================================================
// GYRO LOW-PASS FILTER
// Filters raw gyro before D term to suppress motor vibration noise.
// Lower ALPHA = stronger filter (more lag).
// Higher ALPHA = less filter (more responsive but noisier).
// ======================================================

#define GYRO_LPF_ALPHA 0.7f

float gyro_roll_filtered  = 0;
float gyro_pitch_filtered = 0;
float gyro_yaw_filtered   = 0;

// ======================================================
// COMPLEMENTARY FILTER
// Fuses gyro (accurate short-term) with accel (absolute reference).
// COMP_ALPHA = 0.98 → 98% gyro, 2% accel per step.
// Corrects gyro drift over ~0.5 s time constant at 100 Hz.
// ======================================================

#define COMP_ALPHA 0.98f

float angle_roll  = 0;
float angle_pitch = 0;

// ======================================================
// GYRO CALIBRATION OFFSETS
// ======================================================

float gyro_roll_cal  = 0;
float gyro_pitch_cal = 0;
float gyro_yaw_cal   = 0;

// ======================================================
// MOTOR SPEEDS (global so status endpoint can read them)
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

bool          isFlying   = false;
unsigned long flyingTimer = 0;

#define TAKEOFF_THROTTLE        1200
#define TAKEOFF_CONFIRM_MS      500
#define TAKEOFF_ACCEL_THRESHOLD 1.5f

// ======================================================
// HTML PAGE
// ======================================================

String webpage = R"====(
<!DOCTYPE html>
<html>

<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>

body {
  text-align: center;
  font-family: Arial;
  background: #1a1a1a;
  color: #eee;
}

h2 { color: #4af; }

button {
  width: 120px;
  height: 60px;
  font-size: 18px;
  margin: 10px;
  background: #333;
  color: #eee;
  border: 1px solid #555;
  border-radius: 6px;
  cursor: pointer;
}

button:active { background: #555; }

#stop-btn {
  background: #700;
  color: #fff;
  border-color: #f00;
}

#status-bar {
  font-size: 22px;
  font-weight: bold;
  margin: 12px auto;
  padding: 8px;
  border-radius: 6px;
  background: #222;
  width: 200px;
}

.flying   { color: #0f0; }
.grounded { color: #fa0; }

#log {
  text-align: left;
  background: #111;
  color: #0f0;
  padding: 12px;
  margin: 10px auto;
  width: 90%;
  max-width: 540px;
  border-radius: 6px;
  font-size: 14px;
  min-height: 220px;
}

.log-btns { margin: 8px; }

.log-btns button {
  width: 160px;
  height: 40px;
  font-size: 14px;
}

</style>

</head>

<body>

<h2>ESP32 Drone</h2>

<div id="status-bar" class="grounded">GROUNDED</div>

<button onclick="send('throttle_up')">Throttle +</button>
<button onclick="send('throttle_down')">Throttle -</button>

<br>

<button onclick="send('forward')">Forward</button>

<br>

<button onclick="send('yaw_left')">Yaw L</button>
<button onclick="send('left')">Left</button>
<button onclick="send('center')">Center</button>
<button onclick="send('right')">Right</button>
<button onclick="send('yaw_right')">Yaw R</button>

<br>

<button onclick="send('backward')">Backward</button>

<br><br>

<button id="stop-btn" onclick="send('stop')">STOP</button>

<br>

<div class="log-btns">
  <button onclick="downloadLog()">Download CSV</button>
  <button onclick="clearLog()">Clear Log</button>
</div>

<pre id="log">Waiting for data...</pre>

<script>

const MAX_ENTRIES = 5000;
let logs = JSON.parse(localStorage.getItem("drone_log") || "[]");

function send(cmd)
{
  fetch("/control?cmd=" + cmd);
}

function clearLog()
{
  logs = [];
  localStorage.removeItem("drone_log");
  document.getElementById("log").textContent = "Log cleared.";
}

function downloadLog()
{
  if(logs.length === 0)
  {
    alert("No log data to download.");
    return;
  }

  const header = "time,throttle,angle_roll,angle_pitch,gyro_roll,gyro_pitch,yaw_rate,pid_roll,pid_pitch,pid_yaw,fl,fr,rl,rr,flying\n";

  const rows = logs.map(e =>
    [e.t, e.throttle, e.angle_roll, e.angle_pitch,
     e.gyro_roll, e.gyro_pitch, e.yaw_rate,
     e.pid_roll, e.pid_pitch, e.pid_yaw,
     e.fl, e.fr, e.rl, e.rr, e.flying].join(",")
  ).join("\n");

  const blob = new Blob([header + rows], { type: "text/csv" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "drone_log_" + new Date().toISOString().slice(0,19).replace(/:/g,"-") + ".csv";
  a.click();
}

setInterval(function()
{
  fetch("/status")
    .then(r => r.json())
    .then(d =>
    {
      const entry = {
        t:           new Date().toISOString(),
        throttle:    d.throttle,
        angle_roll:  d.angle_roll,
        angle_pitch: d.angle_pitch,
        gyro_roll:   d.gyro_roll,
        gyro_pitch:  d.gyro_pitch,
        yaw_rate:    d.yaw_rate,
        pid_roll:    d.pid_roll,
        pid_pitch:   d.pid_pitch,
        pid_yaw:     d.pid_yaw,
        fl:          d.fl,
        fr:          d.fr,
        rl:          d.rl,
        rr:          d.rr,
        flying:      d.flying
      };

      logs.push(entry);
      if(logs.length > MAX_ENTRIES) logs.shift();
      localStorage.setItem("drone_log", JSON.stringify(logs));

      const statusEl = document.getElementById("status-bar");
      if(d.flying)
      {
        statusEl.textContent = "FLYING";
        statusEl.className = "flying";
      }
      else
      {
        statusEl.textContent = "GROUNDED";
        statusEl.className = "grounded";
      }

      document.getElementById("log").textContent =
        "Time       : " + entry.t + "\n" +
        "Flying     : " + (d.flying ? "YES" : "NO") + "\n" +
        "Throttle   : " + d.throttle + "\n" +
        "Angle Roll : " + d.angle_roll  + "  Pitch: " + d.angle_pitch + "\n" +
        "Gyro  Roll : " + d.gyro_roll   + "  Pitch: " + d.gyro_pitch  + "\n" +
        "Yaw Rate   : " + d.yaw_rate    + " deg/s\n" +
        "PID   Roll : " + d.pid_roll    + "  Pitch: " + d.pid_pitch + "  Yaw: " + d.pid_yaw + "\n" +
        "FL: " + d.fl + "  FR: " + d.fr + "\n" +
        "RL: " + d.rl + "  RR: " + d.rr + "\n" +
        "Entries    : " + logs.length + " / 5000";
    })
    .catch(function()
    {
      document.getElementById("log").textContent = "Connection lost...";
    });

}, 200);

</script>

</body>
</html>
)====";

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
  Serial.println("KEEP DRONE STILL");
  Serial.println("GYRO CALIBRATION");
  Serial.println("================================");

  sensors_event_t a, g, temp;

  float roll_sum  = 0;
  float pitch_sum = 0;
  float yaw_sum   = 0;

  for(int i = 0; i < 2000; i++)
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

  Serial.println("GYRO CALIBRATION COMPLETE");
  Serial.print("ROLL OFFSET: ");  Serial.println(gyro_roll_cal,  6);
  Serial.print("PITCH OFFSET: "); Serial.println(gyro_pitch_cal, 6);
  Serial.print("YAW OFFSET: ");   Serial.println(gyro_yaw_cal,   6);
}

// ======================================================
// TAKEOFF DETECTION
// ======================================================

void detectTakeoff(float accel_z)
{
  bool conditionsMet = (throttle > TAKEOFF_THROTTLE) &&
                       (fabsf(accel_z - 9.81f) > TAKEOFF_ACCEL_THRESHOLD);

  if(conditionsMet)
  {
    if(flyingTimer == 0) flyingTimer = millis();
    if(millis() - flyingTimer > TAKEOFF_CONFIRM_MS) isFlying = true;
  }
  else
  {
    flyingTimer = 0;
    if(throttle <= ARM_THROTTLE) isFlying = false;
  }
}

// ======================================================
// ROOT PAGE
// ======================================================

void handleRoot()
{
  server.send(200, "text/html", webpage);
}

// ======================================================
// STATUS
// ======================================================

void handleStatus()
{
  String json = "{";
  json += "\"throttle\":"   + String(throttle)              + ",";
  json += "\"angle_roll\":" + String(angle_roll,        2)  + ",";
  json += "\"angle_pitch\":" + String(angle_pitch,      2)  + ",";
  json += "\"gyro_roll\":"  + String(gyro_roll_input,   2)  + ",";
  json += "\"gyro_pitch\":" + String(gyro_pitch_input,  2)  + ",";
  json += "\"yaw_rate\":"   + String(gyro_yaw_input,    2)  + ",";
  json += "\"pid_roll\":"   + String(pid_output_roll,   2)  + ",";
  json += "\"pid_pitch\":"  + String(pid_output_pitch,  2)  + ",";
  json += "\"pid_yaw\":"    + String(pid_output_yaw,    2)  + ",";
  json += "\"fl\":"         + String(motorFL_speed)         + ",";
  json += "\"fr\":"         + String(motorFR_speed)         + ",";
  json += "\"rl\":"         + String(motorRL_speed)         + ",";
  json += "\"rr\":"         + String(motorRR_speed)         + ",";
  json += "\"flying\":"     + String(isFlying ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

// ======================================================
// CONTROL
// ======================================================

void handleControl()
{
  String cmd = server.arg("cmd");

  if(cmd == "throttle_up")   throttle += 20;
  if(cmd == "throttle_down") throttle -= 20;
  if(cmd == "forward")       pitchSetpoint =  15;
  if(cmd == "backward")      pitchSetpoint = -15;
  if(cmd == "left")          rollSetpoint  = -15;
  if(cmd == "right")         rollSetpoint  =  15;
  if(cmd == "yaw_left")      yawSetpoint   = -30;
  if(cmd == "yaw_right")     yawSetpoint   =  30;

  if(cmd == "center")
  {
    pitchSetpoint = 0;
    rollSetpoint  = 0;
    yawSetpoint   = 0;
  }

  if(cmd == "stop")
  {
    throttle      = MIN_THROTTLE;
    pitchSetpoint = 0;
    rollSetpoint  = 0;
    yawSetpoint   = 0;
  }

  throttle = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);

  server.send(200, "text/plain", "OK");
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(1000);

  // ====================================================
  // MPU6050
  // ====================================================

  if(!mpu.begin())
  {
    Serial.println("MPU6050 FAILED");
    while(1);
  }

  Serial.println("MPU6050 OK");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(2000);

  calibrateGyro();

  // ====================================================
  // PWM TIMERS
  // ====================================================

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ====================================================
  // ESC SETUP
  // ====================================================

  motorFL.setPeriodHertz(50);
  motorFR.setPeriodHertz(50);
  motorRL.setPeriodHertz(50);
  motorRR.setPeriodHertz(50);

  motorFL.attach(FL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorFR.attach(FR_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRL.attach(RL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRR.attach(RR_PIN, MIN_THROTTLE, MAX_THROTTLE);

  // ====================================================
  // SAFE MOTOR START
  // ====================================================

  writeMotor(motorFL, MIN_THROTTLE);
  writeMotor(motorFR, MIN_THROTTLE);
  writeMotor(motorRL, MIN_THROTTLE);
  writeMotor(motorRR, MIN_THROTTLE);

  delay(5000);

  // ====================================================
  // WIFI
  // ====================================================

  WiFi.softAP(ssid, password);

  Serial.println("================================");
  Serial.println("WIFI STARTED");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // ====================================================
  // WEB SERVER
  // ====================================================

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/status",  handleStatus);

  server.begin();

  Serial.println("WEB SERVER READY");

  last_time = micros();
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  // ====================================================
  // LOOP TIMING
  // ====================================================

  unsigned long now = micros();
  dt = (now - last_time) / 1000000.0f;
  if(dt <= 0 || dt > 0.1f) dt = 0.01f;
  last_time = now;

  server.handleClient();

  // ====================================================
  // READ SENSORS
  // ====================================================

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ====================================================
  // TAKEOFF DETECTION
  // ====================================================

  detectTakeoff(a.acceleration.z);

  // ====================================================
  // RAW GYRO (deg/s, calibration offset removed)
  // ====================================================

  gyro_roll_input  = (g.gyro.x - gyro_roll_cal)  * 57.2958f;
  gyro_pitch_input = (g.gyro.y - gyro_pitch_cal)  * 57.2958f;
  gyro_yaw_input   = (g.gyro.z - gyro_yaw_cal)    * 57.2958f;

  // ====================================================
  // GYRO LOW-PASS FILTER
  // Smooths gyro before use in D term.
  // Also used as input to complementary filter.
  // ====================================================

  gyro_roll_filtered  = GYRO_LPF_ALPHA * gyro_roll_input  + (1.0f - GYRO_LPF_ALPHA) * gyro_roll_filtered;
  gyro_pitch_filtered = GYRO_LPF_ALPHA * gyro_pitch_input + (1.0f - GYRO_LPF_ALPHA) * gyro_pitch_filtered;
  gyro_yaw_filtered   = GYRO_LPF_ALPHA * gyro_yaw_input   + (1.0f - GYRO_LPF_ALPHA) * gyro_yaw_filtered;

  // ====================================================
  // COMPLEMENTARY FILTER — ANGLE ESTIMATION
  // accel gives absolute roll/pitch but is noisy.
  // gyro integrates accurately short-term but drifts.
  // Filter combines both: gyro for fast response,
  // accel to correct slow drift.
  // ====================================================

  float accel_roll  = atan2f(a.acceleration.y,  a.acceleration.z) * 57.2958f;
  float accel_pitch = atan2f(-a.acceleration.x, a.acceleration.z) * 57.2958f;

  angle_roll  = COMP_ALPHA * (angle_roll  + gyro_roll_filtered  * dt) + (1.0f - COMP_ALPHA) * accel_roll;
  angle_pitch = COMP_ALPHA * (angle_pitch + gyro_pitch_filtered * dt) + (1.0f - COMP_ALPHA) * accel_pitch;

  // ====================================================
  // PID ROLL — ANGLE MODE
  // P + I act on angle error (degrees).
  // D acts directly on filtered gyro rate (deg/s) —
  //   no differentiation, no derivative kick, less noise.
  // ====================================================

  float roll_error = angle_roll - rollSetpoint;

  pid_i_mem_roll  += pid_i_gain_roll * roll_error * dt;
  pid_i_mem_roll   = constrain(pid_i_mem_roll, -300, 300);

  pid_output_roll  = pid_p_gain_roll * roll_error
                   + pid_i_mem_roll
                   - pid_d_gain_roll * gyro_roll_filtered;

  pid_output_roll  = constrain(pid_output_roll, -300, 300);

  // ====================================================
  // PID PITCH — ANGLE MODE
  // ====================================================

  float pitch_error = angle_pitch - pitchSetpoint;

  pid_i_mem_pitch  += pid_i_gain_pitch * pitch_error * dt;
  pid_i_mem_pitch   = constrain(pid_i_mem_pitch, -300, 300);

  pid_output_pitch  = pid_p_gain_pitch * pitch_error
                    + pid_i_mem_pitch
                    - pid_d_gain_pitch * gyro_pitch_filtered;

  pid_output_pitch  = constrain(pid_output_pitch, -300, 300);

  // ====================================================
  // PID YAW — RATE MODE
  // No angle estimation possible without magnetometer.
  // Holds zero rotation rate by default (yawSetpoint = 0).
  // Yaw L/R buttons set a target rotation rate in deg/s.
  // No D term — differentiating gyro yaw amplifies noise.
  // ====================================================

  float yaw_error  = gyro_yaw_input - yawSetpoint;

  pid_i_mem_yaw   += pid_i_gain_yaw * yaw_error * dt;
  pid_i_mem_yaw    = constrain(pid_i_mem_yaw, -300, 300);

  pid_output_yaw   = pid_p_gain_yaw * yaw_error + pid_i_mem_yaw;
  pid_output_yaw   = constrain(pid_output_yaw, -300, 300);

  // ====================================================
  // SAFETY: NO PID AT LOW THROTTLE
  // ====================================================

  if(throttle <= ARM_THROTTLE)
  {
    motorFL_speed = MIN_THROTTLE;
    motorFR_speed = MIN_THROTTLE;
    motorRL_speed = MIN_THROTTLE;
    motorRR_speed = MIN_THROTTLE;

    pid_i_mem_roll  = 0;
    pid_i_mem_pitch = 0;
    pid_i_mem_yaw   = 0;
  }
  else
  {
    // ==================================================
    // MOTOR MIXER — X-FRAME
    // Assumes standard rotation: FL=CCW, FR=CW, RL=CW, RR=CCW.
    // Yaw: CCW motors (FL, RR) oppose CW yaw torque.
    // If drone corrects wrong way on any axis, negate
    // that pid_output term below.
    // ==================================================

    motorFL_speed = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
    motorFR_speed = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
    motorRL_speed = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
    motorRR_speed = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;

    motorFL_speed = constrain(motorFL_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorFR_speed = constrain(motorFR_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorRL_speed = constrain(motorRL_speed, MIN_THROTTLE, MAX_THROTTLE);
    motorRR_speed = constrain(motorRR_speed, MIN_THROTTLE, MAX_THROTTLE);
  }

  // ====================================================
  // WRITE MOTORS
  // ====================================================

  writeMotor(motorFL, motorFL_speed);
  writeMotor(motorFR, motorFR_speed);
  writeMotor(motorRL, motorRL_speed);
  writeMotor(motorRR, motorRR_speed);

  delay(10);
}
