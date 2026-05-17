/*
========================================================
ESP32 DRONE - SAFE TEST VERSION
MPU6050 + WiFi Control + SAFE PID
========================================================

FIXES INCLUDED:
- PID disabled at low throttle
- Safer startup
- Gyro calibration
- Reduced PID aggressiveness
- Motor safety logic
- Better debugging

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

const char* ssid = "ESP32-DRONE";
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

int throttle = 1000;

float pitchSetpoint = 0;
float rollSetpoint = 0;

// ======================================================
// PID VALUES (SAFE START VALUES)
// ======================================================

float pid_p_gain_roll  = 0.5;
float pid_i_gain_roll  = 0.00;
float pid_d_gain_roll  = 0.00;

float pid_p_gain_pitch = 0.5;
float pid_i_gain_pitch = 0.00;
float pid_d_gain_pitch = 0.00;

// ======================================================
// PID VARIABLES
// ======================================================

float pid_i_mem_roll = 0;
float pid_output_roll = 0;
float pid_last_roll_d_error = 0;

float pid_i_mem_pitch = 0;
float pid_output_pitch = 0;
float pid_last_pitch_d_error = 0;

// ======================================================
// GYRO VALUES
// ======================================================

float gyro_roll_input;
float gyro_pitch_input;

// ======================================================
// GYRO CALIBRATION
// ======================================================

float gyro_roll_cal = 0;
float gyro_pitch_cal = 0;
float gyro_yaw_cal = 0;

// ======================================================
// HTML PAGE
// ======================================================

String webpage = R"====(
<!DOCTYPE html>
<html>

<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>

body{
  text-align:center;
  font-family:Arial;
}

button{
  width:120px;
  height:60px;
  font-size:18px;
  margin:10px;
}

</style>

</head>

<body>

<h2>ESP32 Drone</h2>

<button onclick="send('throttle_up')">Throttle +</button>
<button onclick="send('throttle_down')">Throttle -</button>

<br>

<button onclick="send('forward')">Forward</button>

<br>

<button onclick="send('left')">Left</button>
<button onclick="send('center')">Center</button>
<button onclick="send('right')">Right</button>

<br>

<button onclick="send('backward')">Backward</button>

<br><br>

<button onclick="send('stop')">STOP</button>

<script>

function send(cmd)
{
  fetch("/control?cmd=" + cmd);
}

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

  float roll_sum = 0;
  float pitch_sum = 0;
  float yaw_sum = 0;

  for(int i = 0; i < 2000; i++)
  {
    mpu.getEvent(&a, &g, &temp);

    roll_sum += g.gyro.x;
    pitch_sum += g.gyro.y;
    yaw_sum += g.gyro.z;

    delay(2);
  }

  gyro_roll_cal = roll_sum / 2000.0;
  gyro_pitch_cal = pitch_sum / 2000.0;
  gyro_yaw_cal = yaw_sum / 2000.0;

  Serial.println("GYRO CALIBRATION COMPLETE");

  Serial.print("ROLL OFFSET: ");
  Serial.println(gyro_roll_cal, 6);

  Serial.print("PITCH OFFSET: ");
  Serial.println(gyro_pitch_cal, 6);

  Serial.print("YAW OFFSET: ");
  Serial.println(gyro_yaw_cal, 6);
}

// ======================================================
// ROOT PAGE
// ======================================================

void handleRoot()
{
  server.send(200, "text/html", webpage);
}

// ======================================================
// CONTROL
// ======================================================

void handleControl()
{
  String cmd = server.arg("cmd");

  if(cmd == "throttle_up")
  {
    throttle += 20;
  }

  if(cmd == "throttle_down")
  {
    throttle -= 20;
  }

  if(cmd == "forward")
  {
    pitchSetpoint = 15;
  }

  if(cmd == "backward")
  {
    pitchSetpoint = -15;
  }

  if(cmd == "left")
  {
    rollSetpoint = -15;
  }

  if(cmd == "right")
  {
    rollSetpoint = 15;
  }

  if(cmd == "center")
  {
    pitchSetpoint = 0;
    rollSetpoint = 0;
  }

  if(cmd == "stop")
  {
    throttle = MIN_THROTTLE;

    pitchSetpoint = 0;
    rollSetpoint = 0;
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

  server.begin();

  Serial.println("WEB SERVER READY");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  server.handleClient();

  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  // ====================================================
  // APPLY GYRO CALIBRATION
  // ====================================================

  gyro_roll_input =
    (g.gyro.x - gyro_roll_cal) * 57.2958;

  gyro_pitch_input =
    (g.gyro.y - gyro_pitch_cal) * 57.2958;

  // ====================================================
  // PID ROLL
  // ====================================================

  float pid_error_temp;

  pid_error_temp = gyro_roll_input - rollSetpoint;

  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

  pid_i_mem_roll = constrain(pid_i_mem_roll, -300, 300);

  pid_output_roll =
      pid_p_gain_roll * pid_error_temp +
      pid_i_mem_roll +
      pid_d_gain_roll *
      (pid_error_temp - pid_last_roll_d_error);

  pid_output_roll = constrain(pid_output_roll, -300, 300);

  pid_last_roll_d_error = pid_error_temp;

  // ====================================================
  // PID PITCH
  // ====================================================

  pid_error_temp = gyro_pitch_input - pitchSetpoint;

  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -300, 300);

  pid_output_pitch =
      pid_p_gain_pitch * pid_error_temp +
      pid_i_mem_pitch +
      pid_d_gain_pitch *
      (pid_error_temp - pid_last_pitch_d_error);

  pid_output_pitch = constrain(pid_output_pitch, -300, 300);

  pid_last_pitch_d_error = pid_error_temp;

  // ====================================================
  // MOTOR OUTPUTS
  // ====================================================

  int motorFL_speed;
  int motorFR_speed;
  int motorRL_speed;
  int motorRR_speed;

  // ====================================================
  // SAFETY:
  // NO PID AT LOW THROTTLE
  // ====================================================

  if(throttle <= ARM_THROTTLE)
  {
    motorFL_speed = MIN_THROTTLE;
    motorFR_speed = MIN_THROTTLE;
    motorRL_speed = MIN_THROTTLE;
    motorRR_speed = MIN_THROTTLE;

    // reset I terms

    pid_i_mem_roll = 0;
    pid_i_mem_pitch = 0;
  }
  else
  {
    // ==================================================
    // MOTOR MIXER
    // ==================================================

    motorFL_speed =
      throttle - pid_output_pitch + pid_output_roll;

    motorFR_speed =
      throttle - pid_output_pitch - pid_output_roll;

    motorRL_speed =
      throttle + pid_output_pitch + pid_output_roll;

    motorRR_speed =
      throttle + pid_output_pitch - pid_output_roll;

    // ==================================================
    // LIMITS
    // ==================================================

    motorFL_speed =
      constrain(motorFL_speed,
      MIN_THROTTLE,
      MAX_THROTTLE);

    motorFR_speed =
      constrain(motorFR_speed,
      MIN_THROTTLE,
      MAX_THROTTLE);

    motorRL_speed =
      constrain(motorRL_speed,
      MIN_THROTTLE,
      MAX_THROTTLE);

    motorRR_speed =
      constrain(motorRR_speed,
      MIN_THROTTLE,
      MAX_THROTTLE);
  }

  // ====================================================
  // WRITE MOTORS
  // ====================================================

  writeMotor(motorFL, motorFL_speed);
  writeMotor(motorFR, motorFR_speed);
  writeMotor(motorRL, motorRL_speed);
  writeMotor(motorRR, motorRR_speed);

  // ====================================================
  // DEBUG
  // ====================================================

  Serial.println("======================");

  Serial.print("Throttle: ");
  Serial.println(throttle);

  Serial.print("Gyro Roll: ");
  Serial.println(gyro_roll_input);

  Serial.print("Gyro Pitch: ");
  Serial.println(gyro_pitch_input);

  Serial.print("PID Roll: ");
  Serial.println(pid_output_roll);

  Serial.print("PID Pitch: ");
  Serial.println(pid_output_pitch);

  Serial.print("FL: ");
  Serial.println(motorFL_speed);

  Serial.print("FR: ");
  Serial.println(motorFR_speed);

  Serial.print("RL: ");
  Serial.println(motorRL_speed);

  Serial.print("RR: ");
  Serial.println(motorRR_speed);

  delay(10);
}