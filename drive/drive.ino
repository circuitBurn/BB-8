/**
   FrSky SBUs RC control board for Joe's Drive Mk3

   Author:
      Patrick Ryan
      pat.m.ryan@gmail.com
      @circuitBurn

   Pin Mapping:
      TODO:

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
#include <EEPROMex.h>
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"
#include "BTS7960.h"
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MAX_DRIVE_SPEED 128

// RC Values Configuration
#define RC_MIN 172
#define RC_MAX 1811
#define RC_DEADBAND_LOW 960
#define RC_DEADBAND_HIGH 970

// RC Channel Mapping
// The channels are N - 1 where N is the defined channel number on the transmitter
// Ex: CH1 on Tx is 0 here, CH2 is 1
#define CH_DRIVE_MAIN 0
#define CH_DOME_TILT_X 1
#define CH_DOME_TILT_Y 2
#define CH_DRIVE_S2S 3
#define CH_FLYWHEEL 4
#define CH_DOME_SPIN 5
#define CH_SOUND_TRIGGER 7
#define CH_SOUND_BANK 8
#define CH_IMU_SEL 9
#define CH_DRIVE_EN 10
#define CH_ROLL_OFFSET 11
#define CH_FLYWHEEL_EN 12

// Main Drive Motor Controller
#define DRIVE_R_PWM 13
#define DRIVE_L_PWM 12
#define DRIVE_EN 29

// Flywheel Motor Controller
#define FLY_R_PWM 8
#define FLY_L_PWM 9

// Side to side Motor Controller
#define S2S_R_PWM 6
#define S2S_L_PWM 7
#define S2S_EN 33
#define S2S_POT A0
#define S2S_MAX_TILT 26
#define S2S_EASE 4.5
#define S2S_OFFSET 25 // Negative will tilt the drive to the right

// Dome
#define domeSpinPWM1 10
#define domeSpinPWM2 11
#define DOME_POT A4

// PID1 is for the side to side tilt
double Pk1 = 14;//15.0;
double Ik1 = 0.0;
double Dk1 = 0;//0.5;
double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID2 for S2S stability
double Pk2 = 1.5;//0.6;
double Ik2 = 0;
double Dk2 = 0.025;//0.03;
double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

// PID3 for the main drive
double Pk3 = 5.0;
double Ik3 = 0.0;
double Dk3 = 0.0;
double Setpoint3, Input3, Output3, Output3a;
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);

// PID4 for dome rotation
double Pk4 = 2.0;
double Ik4 = 0.1;
double Dk4 = 0.01;
double Setpoint4, Input4, Output4;
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);

// Offsets
float pitchOffset, rollOffset, potOffsetS2S;

bool motorsEnabled = true;

// Dome Servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();

// SBUS
SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// BTS7960 Motor Drivers
BTS7960 driveController(DRIVE_EN, DRIVE_L_PWM, DRIVE_R_PWM);
BTS7960 s2sController(S2S_EN, S2S_L_PWM, S2S_R_PWM);
BTS7960 flywheelController(DRIVE_EN, FLY_L_PWM, FLY_R_PWM);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
float pitch, roll;

// S2S pot smoothing
// TODO: rename
const int numReadings = 10;
int readings[numReadings] = {510, 510, 510, 510, 510, 510, 510, 510, 510, 510};
int readIndex = 0;
int total = 0;
int average = 0;

void setup()
{
  sbus_rx.Begin();

  //    Serial.begin(115200);

  randomSeed(analogRead(0));

  pinMode(DRIVE_R_PWM, OUTPUT);
  pinMode(DRIVE_L_PWM, OUTPUT);
  pinMode(DRIVE_EN, OUTPUT);

  pinMode(S2S_R_PWM, OUTPUT);
  pinMode(S2S_L_PWM, OUTPUT);
  pinMode(S2S_EN, OUTPUT);

  pinMode(FLY_R_PWM, OUTPUT);
  pinMode(FLY_L_PWM, OUTPUT);

  pinMode(domeSpinPWM1, OUTPUT);
  pinMode(domeSpinPWM2, OUTPUT);

  // Servos
  servos.begin();
  servos.setPWMFreq(60);

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
    /* There was a problem detecting the BNO055 ... check your connections */
    //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // TODO: loadOffsets
  pitchOffset = 4;//-0.55; //4.4;
  rollOffset = -3.6;
  potOffsetS2S = 3;

  // Fill the smoothing readings with the initial pot offset
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    //    readings[thisReading] = analogRead(S2S_POT) + S2S_OFFSET;
    readings[thisReading] = potOffsetS2S;
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
    motorsEnabled = is_drive_enabled();
    if (motorsEnabled)
    {
      enable_drive();
      main_drive();
      flywheel();
      side_to_side();
    }
    else
    {
      disable_drive();
    }
    dome_spin();
    dome_servos();
    sound_trigger();
    //        debug_print();
  }
}
