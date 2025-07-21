/**
   FrSky SBUs RC control board for Joe's Drive Mk3

   Author:
      Patrick Ryan
      pat.m.ryan@gmail.com
      @circuitBurn

   Pin Mapping:
      See "constants.h"

   Serial Ports:
      Serial1 - DFPlayer Mini
      Serial2 - SBUS RC

   I2C:
      PWM Servo Board

   Resources:
      https://github.com/circuitBurn/BB-8
      https://bb8builders.club/
      https://www.facebook.com/groups/863379917081398
*/
#include "Arduino.h"
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"
#include "BTS7960.h"
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "constants.h"
#include "enums.h"

// PID1 is for the side to side tilt
double Pk1 = 24;
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

// PID2 for S2S stability
double Pk2 = 1.0;
double Ik2 = 0.0;
double Dk2 = 0.0;
double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

// PID3 for the main drive
double Pk3 = 1.5;
double Ik3 = 0.0;
double Dk3 = 0.01;
double Setpoint3, Input3, Output3, Output3a;
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3, Dk3, DIRECT);

// PID4 for dome rotation
double Pk4 = 2.0;
double Ik4 = 0.1;
double Dk4 = 0.01;
double Setpoint4, Input4, Output4;
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4, Dk4, DIRECT);

// SBUS
bfs::SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// BTS7960 Motor Drivers
BTS7960 driveController(DRIVE_EN_PIN, DRIVE_L_PWM_PIN, DRIVE_R_PWM_PIN);
BTS7960 s2sController(S2S_EN_PIN, S2S_L_PWM_PIN, S2S_R_PWM_PIN);
BTS7960 flywheelController(DRIVE_EN_PIN, FLYWHEEL_L_PWM_PIN, FLYWHEEL_R_PWM_PIN);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
float pitch, roll;

// Drive mode
DriveMode driveMode = DriveMode::Disabled;

// Specifies the direction of BB-8
DriveDirection driveDirection = DriveDirection::Forward;

void setup()
{
  sbus_rx.Begin();

  randomSeed(analogRead(0));

  pinMode(DRIVE_R_PWM_PIN, OUTPUT);
  pinMode(DRIVE_L_PWM_PIN, OUTPUT);
  pinMode(DRIVE_EN_PIN, OUTPUT);

  pinMode(S2S_R_PWM_PIN, OUTPUT);
  pinMode(S2S_L_PWM_PIN, OUTPUT);
  pinMode(S2S_EN_PIN, OUTPUT);

  pinMode(FLYWHEEL_R_PWM_PIN, OUTPUT);
  pinMode(FLYWHEEL_L_PWM_PIN, OUTPUT);

  pinMode(DOME_SPIN_A_PIN, OUTPUT);
  pinMode(DOME_SPIN_B_PIN, OUTPUT);

  // Motor Drivers
  driveController.Enable();
  s2sController.Enable();
  flywheelController.Enable();

  // PID Setup - S2S servo
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(15);

  // PID Setup - S2S stability
  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(15);

  // PID Setup - main drive motor
  PID3.SetMode(AUTOMATIC);
  PID3.SetOutputLimits(-255, 255);
  PID3.SetSampleTime(15);

  // PID Setup - dome motor
  PID4.SetMode(AUTOMATIC);
  PID4.SetOutputLimits(-255, 255);
  PID4.SetSampleTime(15);

  /* Initialise the IMU */
  if (!bno.begin())
  {
    while (1)
      ;
  }

  // Sound
  Serial1.begin(9600);
  myDFPlayer.begin(Serial1);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);
}

void loop()
{
  readIMU();

  if (sbus_rx.Read())
  {
    driveMode = getDriveMode();
    driveDirection = getDriveDirection();

    if (driveMode == DriveMode::Enabled)
    {
      enableDrive();
      mainDrive();
      flywheel();
      sideToSide();
      domeSpin();
      checkAudioTrigger();
    }
    else if (driveMode == DriveMode::Static)
    {
      disableDrive();
      domeSpin();
      checkAudioTrigger();
    }
    else
    {
      // Disabled
      disableDrive();
    }
  }
}

/**
   ReadIMU

   Leaning forward = +y
   Tilting left    = -z

   Pitch is leaning forwards/backwards - Y axis
   Roll is leaning sideways - Z axis
*/
void readIMU()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
}

bool inRcDeadband(int value)
{
  return value >= RC_DEADBAND_LOW && value <= RC_DEADBAND_HIGH;
}

float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/**
 * @returns DriveMode
 */
DriveMode getDriveMode()
{
  int driveVal = sbus_rx.data().ch[CH_DRIVE_EN];
  if (driveVal == RC_MIN)
  {
    return DriveMode::Enabled;
  }
  else if (driveVal == RC_MID)
  {
    return DriveMode::Static;
  }
  else
  {
    return DriveMode::Disabled;
  }
}

/**
 * Drive can be "reversed" to make a quick turnaround
 */
DriveDirection getDriveDirection()
{
  int val = sbus_rx.data().ch[CH_DIRECTION];
  if (val < RC_MAX)
  {
    return DriveDirection::Forward;
  }
  else
  {
    return DriveDirection::Reverse;
  }
}

void disableDrive()
{
  flywheelController.Disable();
  s2sController.Disable();
  driveController.Disable();
}

void enableDrive()
{
  flywheelController.Enable();
  s2sController.Enable();
  driveController.Enable();
}
