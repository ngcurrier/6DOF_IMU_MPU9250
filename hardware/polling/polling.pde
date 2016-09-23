//////////////////////////////////////////////
//
//  Software to interface with MPU9250
//  and easy driver stepper motor controller
//
//  Author: Nicholas Currier
//
//////////////////////////////////////////////

#include "MPU9250.h"
#include "quaternionFilters.h"
#include <SPI.h>
#include <Wire.h>

// defines for easy driver stepper motor controller
#define DIR_PIN 3
#define STEP_PIN 2
#define ENABLE_PIN 4
#define SPEED  0.1             //from 0 to 1  slower gives more torque
#define STEPS_PER_ROT 200        //200 is std, 1600 is microstepping

// defines for MPU9250 breakout, these are part of the MPU9250 library as well
// here for reference hookup only
#define CLK_PIN A5
#define SDO_PIN A4 
#define SDI_PIN 11
#define NSC_PIN 10
#define INT_PIN 12

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// other defines
#define LED_PIN 9

// delay parameters
#define SEC_WAIT 0.02
int waitTimePerCycle = int(SEC_WAIT*1000.0);

MPU9250 myIMU;

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
  // setup MPU9250 control
  Wire.begin();
  pinMode(INT_PIN, INPUT);
  digitalWrite(INT_PIN, LOW);
  
  // setup pins for stepper motor control
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
 
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW); // LOW is enabled

  Serial.begin(57600);
  Serial.print("Trying byte ");
  Serial.print("I should be on ");
  Serial.print(0x71, HEX);
  digitalWrite(LED_PIN, HIGH);
  byte c = 0x0;
  c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 ");
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I should be ");
  Serial.println(0x71, HEX);
  digitalWrite(LED_PIN, LOW);
  c = 0x71;
  if (c == 0x71){
    Serial.println("MPU9250 is online...");
    
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }
  }
  else{
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1);  //Loop forever if we cannot connect
  }
}

void loop()
{
  digitalWrite(LED_PIN, HIGH);

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  myIMU.updateTime();

  angratex = myIMU.mx;
  
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
