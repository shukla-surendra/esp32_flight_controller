#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// WiFi access point
const char* ssid = "ESP32-DRONE";
const char* password = "12345678";

WebServer server(80);

// Sensors
Adafruit_MPU6050 mpu;
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP280 bmp;

// ESC pins
#define FL 18
#define FR 19
#define RL 27
#define RR 14

#define PWM_FREQ 50
#define PWM_RES 16

#define MIN_US 1000
#define MAX_US 1500   // safe test range

int throttle = 1000;
int pitch = 0;       // NEW control variable

float roll, yaw;
float altitude;
float groundAltitude;


// Convert ESC microseconds → PWM duty
uint32_t usToDuty(int us)
{
  return map(us, 1000, 2000, 3276, 6553);
}


// Write motor safely
void writeMotor(int pin, int us)
{
  us = constrain(us, MIN_US, MAX_US);
  ledcWrite(pin, usToDuty(us));
}


// Motor mixing
void updateMotors()
{
  int FL_out = throttle - pitch;
  int FR_out = throttle - pitch;
  int RL_out = throttle + pitch;
  int RR_out = throttle + pitch;

  writeMotor(FL, FL_out);
  writeMotor(FR, FR_out);
  writeMotor(RL, RL_out);
  writeMotor(RR, RR_out);
}


// Webpage UI
String webpage = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
button{width:140px;height:60px;font-size:18px;margin:10px;}
</style>
</head>

<body align="center">

<h2>ESP32 Drone Controller 🚁</h2>

<button onclick="send('throttle_up')">Throttle +</button>
<button onclick="send('throttle_down')">Throttle -</button>

<br><br>

<button onclick="send('forward')">Forward</button>
<button onclick="send('backward')">Backward</button>

<br><br>

<button onclick="send('stop')">STOP</button>

<script>
function send(cmd)
{
 fetch("/control?cmd="+cmd);
}
</script>

</body>
</html>
)====";


// Serve webpage
void handleRoot()
{
  server.send(200, "text/html", webpage);
}


// Handle commands
void handleControl()
{
  String cmd = server.arg("cmd");

  if(cmd == "throttle_up") throttle += 5;
  if(cmd == "throttle_down") throttle -= 5;

  if(cmd == "forward") pitch = 40;
  if(cmd == "backward") pitch = -40;

  if(cmd == "stop")
  {
    throttle = 1000;
    pitch = 0;
  }

  throttle = constrain(throttle, MIN_US, MAX_US);

  updateMotors();

  server.send(200,"text/plain","OK");
}


void setup()
{
  Serial.begin(115200);

  Wire.begin(21,22);

  if(!mpu.begin())
  {
    Serial.println("MPU6050 failed");
    while(1);
  }

  if(!mag.begin())
  {
    Serial.println("Compass failed");
    while(1);
  }

  if(!bmp.begin(0x76))
  {
    if(!bmp.begin(0x77))
    {
      Serial.println("BMP280 failed");
      while(1);
    }
  }

  Serial.println("Sensors ready");

  delay(2000);

  groundAltitude = bmp.readAltitude(1013.25);

  // Attach ESC PWM
  ledcAttach(FL, PWM_FREQ, PWM_RES);
  ledcAttach(FR, PWM_FREQ, PWM_RES);
  ledcAttach(RL, PWM_FREQ, PWM_RES);
  ledcAttach(RR, PWM_FREQ, PWM_RES);

  Serial.println("ESC arming...");

  for(int i=0;i<300;i++)
  {
    updateMotors();
    delay(10);
  }

  Serial.println("ESC ready");

  WiFi.softAP(ssid, password);

  Serial.print("Connect WiFi: ");
  Serial.println(ssid);

  Serial.print("Open browser: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/control", handleControl);

  server.begin();
}


void loop()
{
  server.handleClient();

  Serial.print("Throttle: ");
  Serial.print(throttle);

  Serial.print(" Pitch: ");
  Serial.println(pitch);

  delay(100);
}