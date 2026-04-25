#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_MPU6050 mpu;
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP280 bmp;   // BMP280 object


// ESC pins
#define FL 25
#define FR 26
#define RL 27
#define RR 14

#define PWM_FREQ 50
#define PWM_RES 16

#define MIN_US 1000
#define MAX_US 2000


float roll, pitch, yaw;
float altitude;
float groundAltitude;

int throttle = 1000;


// Convert microseconds to duty cycle
uint32_t usToDuty(int us)
{
  return map(us, 1000, 2000, 3276, 6553);
}


// Write ESC signal safely
void writeMotor(int pin, int us)
{
  us = constrain(us, MIN_US, MAX_US);
  ledcWrite(pin, usToDuty(us));
}


// Apply outputs
void updateMotors()
{
  writeMotor(FL, throttle);
  writeMotor(FR, throttle);
  writeMotor(RL, throttle);
  writeMotor(RR, throttle);
}


// Calculate orientation
void computeAngles()
{
  sensors_event_t a, g, temp;
  sensors_event_t mag_event;

  mpu.getEvent(&a, &g, &temp);
  mag.getEvent(&mag_event);

  roll  = atan2(a.acceleration.y, a.acceleration.z) * 57.3;

  pitch = atan2(-a.acceleration.x,
          sqrt(a.acceleration.y * a.acceleration.y +
               a.acceleration.z * a.acceleration.z)) * 57.3;

  yaw = atan2(mag_event.magnetic.y,
              mag_event.magnetic.x) * 57.3;
}


// Calculate altitude relative to ground
void computeAltitude()
{
  altitude = bmp.readAltitude(1013.25) - groundAltitude;
}


void setup()
{
  Serial.begin(115200);

  Wire.begin(21,22);

  // MPU6050 init
  if(!mpu.begin())
  {
    Serial.println("MPU6050 failed");
    while(1);
  }

  // Compass init
  if(!mag.begin())
  {
    Serial.println("Compass failed");
    while(1);
  }

  // BMP280 init
  if(!bmp.begin(0x76))
  {
    if(!bmp.begin(0x77))
    {
      Serial.println("BMP280 failed");
      while(1);
    }
  }

  Serial.println("BMP280 ready");

  // Capture ground altitude reference
  delay(2000);
  groundAltitude = bmp.readAltitude(1013.25);

  Serial.print("Ground altitude reference: ");
  Serial.println(groundAltitude);


  ledcAttach(FL, PWM_FREQ, PWM_RES);
  ledcAttach(FR, PWM_FREQ, PWM_RES);
  ledcAttach(RL, PWM_FREQ, PWM_RES);
  ledcAttach(RR, PWM_FREQ, PWM_RES);

  Serial.println("ESC arming...");

  // SAFE ARMING SEQUENCE
  for(int i=0;i<300;i++)
  {
    updateMotors();
    delay(10);
  }

  Serial.println("ESC ready");
}


void loop()
{
  computeAngles();
  computeAltitude();

  Serial.print("Roll: ");
  Serial.print(roll);

  Serial.print(" Pitch: ");
  Serial.print(pitch);

  Serial.print(" Yaw: ");
  Serial.print(yaw);

  Serial.print(" Altitude(m): ");
  Serial.println(altitude);

  updateMotors();

  delay(50);
}