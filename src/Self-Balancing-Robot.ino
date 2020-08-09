#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SoftwareSerial.h>
#include "math.h"
#include <PID.h>
#include <digitalWriteFast.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

#define M_PI 3.14159265359

// RIGHT MOTOR
#define AIN1 13           //control pin 1 on the motor driver for the right motor AIN1
#define AIN2 12           //control pin 2 on the motor driver for the right motor AIN2
#define PWMA 11           //speed control pin on the motor driver for the right motor PWMA

// LEFT MOTOR
#define PWMB 10           //speed control pin on the motor driver for the left motor PWMB
#define BIN2 9            //control pin 2 on the motor driver for the left motor BIN2
#define BIN1 8            //control pin 1 on the motor driver for the left motor BIN1

const double Kp = 38;
const double Ki = 0.01;
const double Kd = 84;
PID pidAngle(Kp, Ki, Kd, REVERSE);

const double setpointAngle = 2.5; // Point of Equilibrium
double actualAngle = 0.0;

// Deadband
const int speedOffset = 35;
const int minMotorSpeed = -255 + speedOffset;
const int maxMotorSpeed = 255 - speedOffset;
int wheelSpeed = 0;

double limitAngle = 0.0;

// IMU Variables
double accData[3];
double accelY;
double accelX;
double accelZ;

double gyrData[3];
double gyroX;
double gyroY;
double gyroZ;

double pitchAcc;
float filteredOutput;
const float ALPHA = 0.95; // reliance between gyro / acc

unsigned long t0 = micros();
unsigned long t1 = 0;

const int S_IDLE = 0;
const int S_RUNNING = 1;
int currentState = S_RUNNING;

boolean DEBUG = false;


void setup()
{
  if (!accel.begin())
  {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }

  if (!gyro.begin())
  {
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while (1);
  }

  // MOTOR
  pinModeFast(AIN1, OUTPUT);
  pinModeFast(AIN2, OUTPUT);
  pinModeFast(PWMA, OUTPUT);
  pinModeFast(BIN1, OUTPUT);
  pinModeFast(BIN2, OUTPUT);
  pinModeFast(PWMB, OUTPUT);

  // PID INITs
  pidAngle.setUpdateTime(3);
  pidAngle.setOutputLimits(minMotorSpeed, maxMotorSpeed);

  Serial.begin(115200);
  delay(1500);
}


void loop()
{
  updateSensors();
  actualAngle = complementaryFilter(accData, gyrData, 0.95);

  switch (currentState)
  {
    case S_IDLE:
      // pass
      break;

    case S_RUNNING:
      if ( abs(actualAngle) > 50)
      {
        pauseMotors();
      }
      else
      {
        wheelSpeed = pidAngle.compute(actualAngle, setpointAngle);
        setMotors(wheelSpeed);
      }
      break;

    default:
      break;
  }
}

void updateSensors()
{
  sensors_event_t event;

  accel.getEvent(&event);
  accData[0] = event.acceleration.x;
  accData[1] = event.acceleration.y;
  accData[2] = event.acceleration.z;

  gyro.getEvent(&event);
  gyrData[0] = event.gyro.x;
  gyrData[1] = event.gyro.y;
  gyrData[2] = event.gyro.z;
}

float complementaryFilter(double accData[3], double gyrData[3], float alphaNumber)
{
  int dt = t1 - t0;
  // Turning around the X axis results in a vector on the Y-axis
  pitchAcc = atan2f((float)accData[1], (float)accData[2] * (-1)) * 180 / M_PI;
  // Turning around the Y axis results in a vector on the X-axis
  // rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
  filteredOutput = alphaNumber * (filteredOutput + gyrData[0] * dt / 1000000) + (1 - alphaNumber) * pitchAcc;
  t0 = t1;
  return filteredOutput;
}

void setMotors(int motorSpeed)
{
  rightMotor(motorSpeed);
  leftMotor(motorSpeed);
}

void pauseMotors()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void rightMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWriteFast(AIN1, HIGH);                         // set pin 1 to high
    digitalWriteFast(AIN2, LOW);                          // set pin 2 to low
  }
  else if (motorSpeed < 0)
  {
    digitalWriteFast(AIN1, LOW);                          // set pin 1 to low
    digitalWriteFast(AIN2, HIGH);                         // set pin 2 to high
  }
  analogWrite(PWMA, abs(motorSpeed + speedOffset));       //now that the motor direction is set, drive it at the entered speed
}

void leftMotor(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWriteFast(BIN1, HIGH);                         // set pin 1 to high
    digitalWriteFast(BIN2, LOW);                          // set pin 2 to low
  }
  else if (motorSpeed < 0)
  {
    digitalWriteFast(BIN1, LOW);                          // set pin 1 to low
    digitalWriteFast(BIN2, HIGH);                         // set pin 2 to high
  }
  analogWrite(PWMB, abs(motorSpeed + speedOffset));       // now that the motor direction is set, drive it at the entered speed
}

void changeStateTo(int newState)
{
  currentState = newState;
}
