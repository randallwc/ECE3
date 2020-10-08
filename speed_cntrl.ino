//#include <ECE3.h>

#include <ECE3_SPEED_LCD.h>

//defines for left pin
#define left_nslp_pin 31 // nslp ==> awake & ready for PWM
#define left_dir_pin 29
#define left_pwm_pin 40
//=-=-=-=-

//defines for right pin
#define right_nslp_pin 11 // nslp ==> awake & ready for PWM
#define right_dir_pin 30
#define right_pwm_pin 39
//=-=-=-=-

//constants
float Kp = .2;//.2
float Kd = .1;//.1
float Ki = .5;//.5
const int ONE_ROT_LEFT = 350;
const int ONE_ROT_RIGHT = 350;
//=-=-=-=-=-

//initializations of global variables
int setpoint = 50;//50
float previous_error = 0;
float measured_value = 0;
float previous_value = 0;
float integral = 0;

const int SIZE_OF_VALUES = 3;//3
int values[SIZE_OF_VALUES];
//=-=-=-=-=-=-

void setup()
{  
  ECE3_Init();

//LEFT WHEEL
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  digitalWrite(left_nslp_pin,HIGH); // no sleep
  digitalWrite(left_dir_pin,LOW); // driection
//=-=-=-=-=-

//RIGHT WHEEL
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);
//=-=-=-=-=-

//initialization of values
for(int i = 0; i < SIZE_OF_VALUES; i++)
{
  values[i] = 0;
}
//=-=-=-=-=-
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

// DELAY IN SETUP  
  delay(750);
//=-=-=-=-=-=-=-=-=-

}

int prev_ms = 0;

void loop()
{ 
  ECE3_LCD_output_speed();
  
  float current_value = getEncoderCount_left();
//  measured_value = getEncoderCount_right();

  measured_value = avgEncCount( current_value , previous_value);

  float error = setpoint - measured_value;
  float derivative = error - previous_error;
  integral = integral + error;
  float output = Kp * error + Kd * derivative + Ki * integral;  
  previous_error = error;
  previous_value = current_value;

//  Serial.print(measured_value);
//  Serial.print(current_value);
  Serial.print(error);
//  Serial.print(derivative);
//  Serial.print(output);
  Serial.print(',');
  Serial.println(0);

  spin('l', output);

//  int cur_ms = millis();
//  Serial.println(cur_ms - prev_ms);
//  prev_ms = cur_ms;
}

float avgEncCount(float current_encoder, float previous_encoder)
{
  for(int i = (SIZE_OF_VALUES - 1); i != 0; i--)
  {
    values[i] = values[i-1];
  }
  values[0] = current_encoder - previous_encoder;

  float average = 0;
  for(int i = 0; i < SIZE_OF_VALUES; i++)
  {
    average = average + values[i];
  }
  
  return average;
}

void spin(char dir, float amt)
{
  int maxspd = 200;

//constrain between max spd and zero
  if (amt > maxspd)
  {
    amt = maxspd;
  }
  if (amt < 0)
  {
    amt = 0;
  }
//=-=-=-=-=-

//right or left
  switch(dir)
  {
    case 'r': // right
      analogWrite(right_pwm_pin, amt);
      break;
    case 'l':  // left
      analogWrite(left_pwm_pin, amt);
      break;
    default:
      break;
  }
//=-=-=-=-=-=-=
}

/*
float speedArr[10] = {0,0,0,0,0,0,0,0,0,0};

float calcSpdAndPrintSpd(float measured_val)
{
  int iter = 5;
  int dely = 100;
  for(int i = 9; i != 0; i--)
  {
    speedArr[i] = speedArr[i-1];
  }
  speedArr[0] = measured_val;
  
  float speed = (speedArr[0] - speedArr[iter])/(iter * 1.);  // encoder counts per run
  //  encoder / run => rot / run => rot / ms => rot / sec
  //  convert encoder counts per run into rotations per milisecond
  speed = (speed * 1.) * ( 1. / ONE_ROT_RIGHT) * (1. / (dely * 1.)) * (1000./ 1);

  Serial.println(speed);

  delay(dely);

  return speed;
}
*/

//SPEED CONTROL: a PID control algorithm will be developed by the
//team to set one wheel speed to a predetermined level. Then, while 
//the car is being held in one’s hands, a finger will be applied to 
//the wheel to slow it down. The PID algorithm will be expected to 
//return the wheel speed to the predetermined level in spite of the 
//finger adding friction to the wheel. The wheel speed will be 
//indicated on the LCD display.
