#define FL 18
#define FR 19
#define RL 27
#define RR 14

#define PWM_FREQ 50
#define PWM_RES 16

uint32_t usToDuty(int us)
{
  return map(us, 1000, 2000, 3276, 6553);
}

void setup()
{
  Serial.begin(115200);

  ledcAttach(FL, PWM_FREQ, PWM_RES);
  ledcAttach(FR, PWM_FREQ, PWM_RES);
  ledcAttach(RL, PWM_FREQ, PWM_RES);
  ledcAttach(RR, PWM_FREQ, PWM_RES);

  Serial.println("Disconnect battery now...");
  delay(4000);

  Serial.println("Connect battery NOW");

  ledcWrite(FL, usToDuty(2000));
  ledcWrite(FR, usToDuty(2000));
  ledcWrite(RL, usToDuty(2000));
  ledcWrite(RR, usToDuty(2000));

  delay(5000);

  Serial.println("Setting MIN throttle");

  ledcWrite(FL, usToDuty(1000));
  ledcWrite(FR, usToDuty(1000));
  ledcWrite(RL, usToDuty(1000));
  ledcWrite(RR, usToDuty(1000));

  delay(5000);

  Serial.println("Calibration done");
}

void loop(){}