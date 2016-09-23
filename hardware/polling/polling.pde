//////////////////////////////////////////////
//
//  Software to interface with MPU9250
//  and easy driver stepper motor controller
//
//  Author: Nicholas Currier
//
//////////////////////////////////////////////

 extern "C"
 {
   #include "avr_i2c.h"
 }
 #include "mpu9250.h"

// defines for easy driver stepper motor controller
#define DIR_PIN 3
#define STEP_PIN 2
#define ENABLE_PIN 4
#define SPEED  0.1             //from 0 to 1  slower gives more torque
#define STEPS_PER_ROT 200        //200 is std, 1600 is microstepping

// defines for MPU9250 breakout, these are part of the MPU9250 library as well
// here for reference hookup only
#define CLK_PIN 13
#define SDO_PIN 12 
#define SDI_PIN 11
#define NSC_PIN 10


// other defines
#define LED_PIN 9

// delay parameters
#define SEC_WAIT 0.02
int waitTimePerCycle = int(SEC_WAIT*1000.0);

MPU_9250 myIMU;

double angratex = 0;
double angratey = 0;
double angratez = 0;
double angx = 0;
double angy = 0;
double angz = 0;
double accx = 0;
double accy = 0;
double accz = 0;

double angStep = 400.0;

void rotate(int steps, float speed);
void rotateDeg(float deg, float speed);


void setup()
{
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW); // LOW is enabled

  struct int_param_s int_param;
  myIMU.begin(int_param);
  myIMU.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  Serial.begin(57600);
 }

void loop()
{
  digitalWrite(LED_PIN, HIGH);
  
  //dump the data to the serial bus
  Serial.print(millis()/1000.0);
  Serial.print(',');
  Serial.print(angratex);
  Serial.print(',');
  Serial.print(angratey);
  Serial.print(',');
  Serial.print(angratez);
  Serial.print(',');
  Serial.print(accx);
  Serial.print(',');
  Serial.print(accy);
  Serial.print(',');
  Serial.print(accz);
  Serial.print('\n');

  angx = angx + angStep;
  double fulldeg = int(angx/360.0)*360.0;
  angx = angx - fulldeg;
  angratex = angx;

  rotateDeg(angStep, SPEED);

  delay(waitTimePerCycle);
  digitalWrite(LED_PIN, LOW);
  delay(waitTimePerCycle);

  long temperature = 0;
  unsigned long timestamp = 0;
  myIMU.get_temperature(temperature, timestamp);
  Serial.print("Temperature (˚C): ");
  Serial.print(float(temperature/65536.0));
}


//rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
void rotate(int steps, float speed)
{
  int dir = (steps < 0)? HIGH:LOW;
  steps = abs(steps);

  digitalWrite(DIR_PIN,dir); 

  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay); 

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
} 

//rotate a specific number of degrees (negitive for reverse movement)
//speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
void rotateDeg(float deg, float speed)
{
  int dir = (deg < 0)? HIGH:LOW;
  digitalWrite(DIR_PIN,dir); 

  int steps = abs(deg)*(1/0.225);
  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay); 

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
}
