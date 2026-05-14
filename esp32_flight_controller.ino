#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// ======================================================
// WIFI ACCESS POINT
// ======================================================
const char* ssid = "ESP32-DRONE";
const char* password = "12345678";

WebServer server(80);

// ======================================================
// I2C PINS
// ======================================================
#define SDA_PIN 19
#define SCL_PIN 18

// ======================================================
// SENSOR ADDRESSES
// ======================================================
#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

#define LSM303_ACC_ADDR 0x19
#define LSM303_MAG_ADDR 0x1E

// ======================================================
// SENSOR OBJECTS
// ======================================================
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// ======================================================
// BMP STATUS
// ======================================================
bool bmpFound = false;

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
#define MIN_US     1000
#define IDLE_US    1050
#define MAX_US     2000

// ======================================================
// CONTROL VARIABLES
// ======================================================
int throttle = MIN_US;
int pitch = 0;

float altitude = 0;
float groundAltitude = 0;

// ======================================================
// LSM303 RAW DATA
// ======================================================
int16_t lsmAccX, lsmAccY, lsmAccZ;
int16_t lsmMagX, lsmMagY, lsmMagZ;

// ======================================================
// SERVO OBJECTS
// ======================================================
Servo motorFL;
Servo motorFR;
Servo motorRL;
Servo motorRR;

// ======================================================
// WRITE MOTOR
// ======================================================
void writeMotor(Servo &motor, int us)
{
  us = constrain(us, MIN_US, MAX_US);
  motor.writeMicroseconds(us);
}

// ======================================================
// MOTOR MIXER
// ======================================================
void updateMotors()
{
  int FL_out = throttle - pitch;
  int FR_out = throttle - pitch;

  int RL_out = throttle + pitch;
  int RR_out = throttle + pitch;

  writeMotor(motorFL, FL_out);
  writeMotor(motorFR, FR_out);
  writeMotor(motorRL, RL_out);
  writeMotor(motorRR, RR_out);

  Serial.println("======================");

  Serial.print("FL: ");
  Serial.println(FL_out);

  Serial.print("FR: ");
  Serial.println(FR_out);

  Serial.print("RL: ");
  Serial.println(RL_out);

  Serial.print("RR: ");
  Serial.println(RR_out);
}

// ======================================================
// READ LSM303 ACCELEROMETER
// ======================================================
void readLSM303Accel()
{
  Wire.beginTransmission(LSM303_ACC_ADDR);
  Wire.write(0x28 | 0x80);

  if(Wire.endTransmission(false) != 0)
    return;

  Wire.requestFrom(LSM303_ACC_ADDR, 6);

  if(Wire.available() >= 6)
  {
    uint8_t xl = Wire.read();
    uint8_t xh = Wire.read();

    uint8_t yl = Wire.read();
    uint8_t yh = Wire.read();

    uint8_t zl = Wire.read();
    uint8_t zh = Wire.read();

    lsmAccX = (int16_t)(xh << 8 | xl);
    lsmAccY = (int16_t)(yh << 8 | yl);
    lsmAccZ = (int16_t)(zh << 8 | zl);
  }
}

// ======================================================
// READ LSM303 MAGNETOMETER
// ======================================================
void readLSM303Mag()
{
  Wire.beginTransmission(LSM303_MAG_ADDR);
  Wire.write(0x68);

  if(Wire.endTransmission(false) != 0)
    return;

  Wire.requestFrom(LSM303_MAG_ADDR, 6);

  if(Wire.available() >= 6)
  {
    uint8_t xh = Wire.read();
    uint8_t xl = Wire.read();

    uint8_t yh = Wire.read();
    uint8_t yl = Wire.read();

    uint8_t zh = Wire.read();
    uint8_t zl = Wire.read();

    lsmMagX = (int16_t)(xh << 8 | xl);
    lsmMagY = (int16_t)(yh << 8 | yl);
    lsmMagZ = (int16_t)(zh << 8 | zl);
  }
}

// ======================================================
// INIT LSM303
// ======================================================
void initLSM303()
{
  // Accelerometer enable
  Wire.beginTransmission(LSM303_ACC_ADDR);
  Wire.write(0x20);
  Wire.write(0x57);
  Wire.endTransmission();

  // Magnetometer continuous mode
  Wire.beginTransmission(LSM303_MAG_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ======================================================
// ESC CALIBRATION
// ======================================================
void calibrateESCs()
{
  Serial.println("================================");
  Serial.println("ESC CALIBRATION START");
  Serial.println("DISCONNECT BATTERY NOW");
  Serial.println("================================");

  delay(5000);

  // Send MAX throttle
  motorFL.writeMicroseconds(MAX_US);
  motorFR.writeMicroseconds(MAX_US);
  motorRL.writeMicroseconds(MAX_US);
  motorRR.writeMicroseconds(MAX_US);

  Serial.println("CONNECT BATTERY NOW");
  Serial.println("WAITING FOR BEEPS...");

  delay(8000);

  // Send MIN throttle
  motorFL.writeMicroseconds(MIN_US);
  motorFR.writeMicroseconds(MIN_US);
  motorRL.writeMicroseconds(MIN_US);
  motorRR.writeMicroseconds(MIN_US);

  Serial.println("WAITING FOR CONFIRMATION BEEPS");

  delay(8000);

  Serial.println("ESC CALIBRATION COMPLETE");
}

// ======================================================
// WEBPAGE
// ======================================================
String webpage = R"====(
<!DOCTYPE html>
<html>

<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
button{
  width:140px;
  height:60px;
  font-size:18px;
  margin:10px;
}
</style>

</head>

<body align="center">

<h2>ESP32 Drone Controller</h2>

<button onclick="send('throttle_up')">Throttle +</button>
<button onclick="send('throttle_down')">Throttle -</button>

<br><br>

<button onclick="send('forward')">Forward</button>
<button onclick="send('backward')">Backward</button>

<br><br>

<button onclick="send('center')">Center</button>

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
// HANDLE ROOT
// ======================================================
void handleRoot()
{
  server.send(200, "text/html", webpage);
}

// ======================================================
// HANDLE CONTROL
// ======================================================
void handleControl()
{
  String cmd = server.arg("cmd");

  if(cmd == "throttle_up")
    throttle += 20;

  if(cmd == "throttle_down")
    throttle -= 20;

  if(cmd == "forward")
    pitch = 40;

  if(cmd == "backward")
    pitch = -40;

  if(cmd == "center")
    pitch = 0;

  if(cmd == "stop")
  {
    throttle = MIN_US;
    pitch = 0;
  }

  throttle = constrain(throttle, MIN_US, MAX_US);

  updateMotors();

  server.send(200, "text/plain", "OK");
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Serial.begin(115200);

  // ====================================================
  // START I2C
  // ====================================================
  Wire.begin(SDA_PIN, SCL_PIN);

  delay(1000);

  Serial.println("I2C STARTED");

  // ====================================================
  // INIT LSM303
  // ====================================================
  initLSM303();

  Serial.println("LSM303 INITIALIZED");

  // ====================================================
  // MPU6050
  // ====================================================
  if(!mpu.begin(MPU6050_ADDR, &Wire))
  {
    Serial.println("MPU6050 FAILED");
  }
  else
  {
    Serial.println("MPU6050 OK");
  }

  // ====================================================
  // BMP280
  // ====================================================
  if(bmp.begin(BMP280_ADDR))
  {
    Serial.println("BMP280 OK");
    bmpFound = true;
  }
  else
  {
    Serial.println("BMP280 FAILED");
  }

  // ====================================================
  // ALTITUDE CALIBRATION
  // ====================================================
  if(bmpFound)
  {
    groundAltitude = bmp.readAltitude(1013.25);

    Serial.print("GROUND ALTITUDE: ");
    Serial.println(groundAltitude);
  }

  // ====================================================
  // ATTACH ESCs
  // ====================================================
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motorFL.setPeriodHertz(50);
  motorFR.setPeriodHertz(50);
  motorRL.setPeriodHertz(50);
  motorRR.setPeriodHertz(50);

  motorFL.attach(FL_PIN, MIN_US, MAX_US);
  motorFR.attach(FR_PIN, MIN_US, MAX_US);
  motorRL.attach(RL_PIN, MIN_US, MAX_US);
  motorRR.attach(RR_PIN, MIN_US, MAX_US);

  // ====================================================
  // INITIAL MIN THROTTLE
  // ====================================================
  motorFL.writeMicroseconds(MIN_US);
  motorFR.writeMicroseconds(MIN_US);
  motorRL.writeMicroseconds(MIN_US);
  motorRR.writeMicroseconds(MIN_US);

  delay(5000);

  // ====================================================
  // OPTIONAL ESC CALIBRATION
  // ====================================================
  // Uncomment ONLY ONCE when calibrating ESCs
  //
  // calibrateESCs();
  //

  Serial.println("ESC READY");

  // ====================================================
  // WIFI ACCESS POINT
  // ====================================================
  WiFi.softAP(ssid, password);

  Serial.print("CONNECT WIFI: ");
  Serial.println(ssid);

  Serial.print("OPEN BROWSER: ");
  Serial.println(WiFi.softAPIP());

  // ====================================================
  // SERVER ROUTES
  // ====================================================
  server.on("/", handleRoot);
  server.on("/control", handleControl);

  server.begin();

  Serial.println("WEB SERVER STARTED");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
  server.handleClient();

  // ====================================================
  // MPU6050
  // ====================================================
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  // ====================================================
  // LSM303
  // ====================================================
  readLSM303Accel();
  readLSM303Mag();

  // ====================================================
  // SENSOR OUTPUT
  // ====================================================
  Serial.println("======================");

  Serial.print("Throttle: ");
  Serial.println(throttle);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("MPU Accel X: ");
  Serial.println(a.acceleration.x);

  Serial.print("MPU Accel Y: ");
  Serial.println(a.acceleration.y);

  Serial.print("MPU Accel Z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Gyro X: ");
  Serial.println(g.gyro.x);

  Serial.print("Gyro Y: ");
  Serial.println(g.gyro.y);

  Serial.print("Gyro Z: ");
  Serial.println(g.gyro.z);

  Serial.print("LSM Accel X: ");
  Serial.println(lsmAccX);

  Serial.print("LSM Accel Y: ");
  Serial.println(lsmAccY);

  Serial.print("LSM Accel Z: ");
  Serial.println(lsmAccZ);

  Serial.print("Compass X: ");
  Serial.println(lsmMagX);

  Serial.print("Compass Y: ");
  Serial.println(lsmMagY);

  Serial.print("Compass Z: ");
  Serial.println(lsmMagZ);

  // ====================================================
  // BMP280 ALTITUDE
  // ====================================================
  if(bmpFound)
  {
    altitude = bmp.readAltitude(1013.25) - groundAltitude;

    Serial.print("Altitude: ");
    Serial.println(altitude);
  }

  delay(500);
}