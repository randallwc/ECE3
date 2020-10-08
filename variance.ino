//non lcd
//#include <ECE3.h>
//

//odo
#include <ECE3_SPEED_LCD.h>
#include <stdio.h>
//=-=-

#define left_nslp_pin 31 // nslp ==> awake & ready for PWM
#define left_dir_pin 29
#define left_pwm_pin 40

#define right_nslp_pin 11 // nslp ==> awake & ready for PWM
#define right_dir_pin 30
#define right_pwm_pin 39

uint16_t sensorValues[8];

// global variables
int base = 150;//140

float leftSpd = base;
float rightSpd = base;
int turndelay = 215;//240

float Kp = .02;
float Kd = .35;
float Ki = 0;

float previous_error = 0;
float integral = 0;

bool lastTurnAround = false;
bool hasTurned = false;
//=-=-=-=-

//variance
double tot_var = 0;
double var_count = 1;
double old_mean = 0;
double old_var = 0;

char var_asChar[20];
char buffer[20];
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

  //  increaseTo();
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  //variance
  float display = variance();
  //

  if (sensorTurnAround() && lastTurnAround && !hasTurned) // uturn
  {
    performTurnAround(turndelay);

    lastTurnAround = false;
    hasTurned = true;

    increaseTo();
  }

  if (sensorTurnAround() && lastTurnAround && hasTurned) // stop & variance
  {
    //display variance on LCD
    dtostrf(display,6,2,var_asChar);
    sprintf(buffer,"var %s",var_asChar);
    ECE3_LCD_output_custom(buffer,1);
    //
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

  previous_error = error;
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
//  analogWrite(left_pwm_pin, 0);
//  analogWrite(right_pwm_pin, 0);
//
//  delay(10);

  decreaseToStop();
  
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, HIGH);

  analogWrite(left_pwm_pin, 255);
  analogWrite(right_pwm_pin, 255);

  delay(delayTime);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);
}

void increaseTo()
{
  for (int speed = 0 ; speed < base; speed += 10)
  {
    analogWrite(left_pwm_pin, speed);
    analogWrite(right_pwm_pin, speed);
    delay(20);
  }
}

void decreaseToStop()
{
  digitalWrite(right_dir_pin, HIGH);
  digitalWrite(left_dir_pin, HIGH);
  
  analogWrite(left_pwm_pin, 30);
  analogWrite(right_pwm_pin, 30);

  delay(400);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void stop_car_forever()
{
  decreaseToStop();

  while(1)
  {
//    Serial.print("output: ");
//    Serial.println(buffer);
//    Serial.println("=-=-=-=-");
//    
//    //stops forever
//    delay(5000);
  }
}

double combinedVar()
{
  double newVal = 0;
  double var_sensorWeights[8]={-0.004, -0.003, -0.002, -0.001, 0.001, 0.002, 0.003, 0.004};
  for (int i = 0; i < 8; i++)
  {
    newVal += var_sensorWeights[i] * sensorValues[i];
  }
  return newVal;
}

double variance()
{
  //COMPUTE THE *VARIANCE* PATH ERROR USING THE VARIANCE WEIGHTS.
  double measured_value = combinedVar();
//  Serial.println(measured_value);

  //COMPUTE UPDATED (CURRENT) AVERAGE OF *VARIANCE* PATH ERROR.
  double cur_mean = old_mean + ( (measured_value - old_mean) / (var_count) );
//  Serial.println(cur_mean);

  //COMPUTE UPDATED (CURRENT) VARIANCE.
  double cur_var = old_var + sq(old_mean) - sq(cur_mean) + ( ( sq(measured_value) - old_var - sq(old_mean) ) / (var_count) );

  //OLD MEAN = CURRENT MEAN.
  old_mean = cur_mean;

  //OLD VARIANCE = CURRENT VARIANCE.
  old_var = cur_var;

  //increase counter of runs used for the average
  var_count = var_count + 1;

//  Serial.println(cur_var);
  
  return old_var;
}
