//non lcd
#include <ECE3.h>
//

//odo
// #include <ECE3_SPEED_LCD.h>
//=-=-

#define left_nslp_pin 31 // nslp ==> awake & ready for PWM
#define left_dir_pin 29
#define left_pwm_pin 40

#define right_nslp_pin 11 // nslp ==> awake & ready for PWM
#define right_dir_pin 30
#define right_pwm_pin 39

uint16_t sensorValues[8];

// global variables
//int base = 140;
//int base = 160;// 15.46
//int base = 210; // 13.4 
//int base = 230;//13.4 turn 215
//int base = 240;//13.01 turn 210
int base = 240;//turn 212

float leftSpd = base;
float rightSpd = base;
//int turndelay = 240;
int turndelay = 212;

float Kp = .025;
float Kd = .353;//.35
float Ki = 0;

float previous_error = 0;
float integral = 0;

bool lastTurnAround = false;
bool hasTurned = false;
//=-=-=-=-

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

  //  increaseTo(base);
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  if (sensorTurnAround() && lastTurnAround && !hasTurned)
  {
    performTurnAround(turndelay);

    lastTurnAround = false;
    hasTurned = true;

    increaseTo(base);
  }

  if (sensorTurnAround() && lastTurnAround && hasTurned)
  {
    stop_car_forever();
  }

  lastTurnAround = sensorTurnAround();

  float measuredValue = combinedValues();

  float error = measuredValue;
  float derivative = (error - previous_error);
  integral = integral + error; // integral + error

  float output = Kp * error + Ki * integral + Kd * derivative;

  float leftOutput = leftSpd - output;
  float rightOutput = rightSpd + output;

  if (leftOutput > 255)  {
    leftOutput = 254;
  }
  if (rightOutput > 255) {
    rightOutput = 254;
  }
  if (leftOutput < 0)    {
    leftOutput = 0;
  }
  if (rightOutput < 0)   {
    rightOutput = 0;
  }

  analogWrite(left_pwm_pin, leftOutput);
  analogWrite(right_pwm_pin, rightOutput);

  //  Serial.println(output);

  previous_error = error;

  //odo
 // odo();
  //
}

int combinedValues()
{
  float newVal = 0;
  float arr[8]  = { -1.875, -1.75, -1.5, -1, 1, 1.5, 1.75, 1.875};
  for (int i = 0; i < 8; i++)
  {
    newVal += arr[i] * sensorValues[i];
  }
  return newVal;
}

bool sensorTurnAround()
{
  int val = 0;
  for (int i = 0; i < 8; i++)
  {
    val = val + sensorValues[i];
  }

  if (val > 17000)
  {
    return true;
  }

  return false;
}

void performTurnAround(int delayTime)
{
  decreaseToStop();  
  
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, HIGH);

  analogWrite(left_pwm_pin, 255);
  analogWrite(right_pwm_pin, 255);

  delay(delayTime);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);
}

void increaseTo(int baseSpeed)
{
  for (int speed = 0 ; speed < baseSpeed; speed += 10)
  {
    analogWrite(left_pwm_pin, speed);
    analogWrite(right_pwm_pin, speed);
    delay(20);
  }
}

void odo()
{
  if ((getEncoderCount_left() + getEncoderCount_right()) / 2 > 15000)
  {
    digitalWrite(BLUE_LED, HIGH);
  }
}

void stop_car_forever()
{
  decreaseToStop();
  
  while(1){}
}

void decreaseToStop()
{
  digitalWrite(right_dir_pin, HIGH);
  digitalWrite(left_dir_pin, HIGH);
  
  analogWrite(left_pwm_pin, 30);
  analogWrite(right_pwm_pin, 30);

  delay(390);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}
