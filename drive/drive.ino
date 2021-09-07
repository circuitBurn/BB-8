#include "Arduino.h"
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"
#include <VarSpeedServo.h>
#include "BTS7960.h"
#include <PID_v1.h>

#define domeSpinPWM1 10
#define domeSpinPWM2 11

#define leftDomeTiltServo 4
#define rightDomeTiltServo 5
#define MaxDomeTiltY 20
#define MaxDomeTiltX 18

#define RC_MIN 172
#define RC_MAX 1811
#define RC_DEADBAND_LOW 960
#define RC_DEADBAND_HIGH 970

// Define the RC channels
#define CH_DRIVE_MAIN 0
#define CH_DOME_TILT_Y 1
#define CH_DOME_TILT_X 2
#define CH_DRIVE_S2S 3
#define CH_FLYWHEEL 4
#define CH_DOME_SPIN 5

// Main Drive
#define DRIVE_R_PWM 13
#define DRIVE_L_PWM 12
#define DRIVE_EN 29

// Flywheel
#define FLY_R_PWM 8
#define FLY_L_PWM 9
#define FLY_EN 29

// Side to side
#define S2S_R_PWM 6
#define S2S_L_PWM 7
#define S2S_EN 33
#define S2S_POT A0
#define maxS2STilt 20
//#define reverseS2SPot
#define S2SEase 2.5

// TODO:
#define DomeYEase .4 // Spead of front to back dome movement, higher == faster
#define DomeXEase .7 // Speed of side to side domemovement, higher == faster
#define domeSpeed 90 // Speed that the servos will move

// PID Controls

//PID1 is for the side to side tilt
double Pk1 = 14;
double Ik1 = 0;
double Dk1 = 0.0;
double Setpoint1, Input1, Output1, Output1a;
double roll = 0;
double rollOffset = 0;

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID2 is for side to side stability
double Pk2 = .5;
double Ik2 = 0;
double Dk2 = .01;
double Setpoint2, Input2, Output2, Output2a;

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - S2S stability

// TODO:
int rcDome = 0;
int rcFly = 0;
double Joy2YPitch, Joy2YDirection, Joy2XDirection;
int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase, Joy2XEaseMap;
double Joy2YEaseMap;

int potOffsetS2S = 3;
int S2Spot;

// SBUS
SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// Neck Servos
VarSpeedServo leftServo;
VarSpeedServo rightServo;

// BTS7960 Motor Drivers
BTS7960 driveController(DRIVE_EN, DRIVE_L_PWM, DRIVE_R_PWM);
BTS7960 s2sController(S2S_EN, S2S_L_PWM, S2S_R_PWM);
BTS7960 flywheelController(FLY_EN, FLY_L_PWM, FLY_R_PWM);

// TODO:
int s2sPot = 512;

void setup()
{
  // SBUS
  sbus_rx.Begin();

  // Sound
  myDFPlayer.begin(Serial2);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);

  // Pinmodes
  pinMode(DRIVE_R_PWM, OUTPUT);
  pinMode(DRIVE_L_PWM, OUTPUT);
  pinMode(DRIVE_EN, OUTPUT);

  pinMode(S2S_R_PWM, OUTPUT);
  pinMode(S2S_L_PWM, OUTPUT);
  pinMode(S2S_EN, OUTPUT);

  pinMode(FLY_R_PWM, OUTPUT);
  pinMode(FLY_L_PWM, OUTPUT);
  pinMode(FLY_EN, OUTPUT);

  pinMode(domeSpinPWM1, OUTPUT);
  pinMode(domeSpinPWM2, OUTPUT);

  // Servos
  leftServo.attach(leftDomeTiltServo);
  rightServo.attach(rightDomeTiltServo);
  leftServo.write(95, 50, false);
  rightServo.write(90, 50, false);

  // Motor Drivers
  driveController.Enable();
  s2sController.Enable();
  flywheelController.Enable();

  // PID Setup

  // S2S SERVO
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(15);

  // S2S Stability
  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(15);
}

void loop()
{
  if (sbus_rx.Read())
  {
    // Dome
    rcDome = map(sbus_rx.rx_channels()[CH_DOME_SPIN], RC_MIN, RC_MAX, -255, 255);
    if (rcDome < 25 && rcDome > -25)
    {
      rcDome = 0;
    }

    if (rcDome <= -20)
    {
      analogWrite(domeSpinPWM1, abs(rcDome));
      analogWrite(domeSpinPWM2, 0);
    }
    else if (rcDome >= 20)
    {
      analogWrite(domeSpinPWM1, 0);
      analogWrite(domeSpinPWM2, abs(rcDome));
    }
    else
    {
      analogWrite(domeSpinPWM1, 0);
      analogWrite(domeSpinPWM2, 0);
    }

    // Servos
    int Joy2XDirection = map(sbus_rx.rx_channels()[CH_DOME_TILT_Y], RC_MIN, RC_MAX, -MaxDomeTiltX, MaxDomeTiltX);
    int Joy2YDirection = map(sbus_rx.rx_channels()[CH_DOME_TILT_X], RC_MAX, RC_MIN, -MaxDomeTiltY, MaxDomeTiltY);

    if ((Joy2XDirection + DomeXEase) > Joy2XEase && (Joy2XDirection - DomeXEase) < Joy2XEase)
    {
      Joy2XEase = Joy2XDirection;
    }
    else if (Joy2XEase > Joy2XDirection)
    {
      Joy2XEase -= DomeXEase;
    }
    else if (Joy2XEase < Joy2XDirection)
    {
      Joy2XEase += DomeXEase;
    }
    if ((Joy2YDirection + DomeYEase) > Joy2YEase && (Joy2YDirection - DomeYEase) < Joy2YEase)
    {
      Joy2YEase = Joy2YDirection;
    }
    else if (Joy2YEase > Joy2YDirection)
    {
      Joy2YEase -= DomeYEase;
    }
    else if (Joy2YEase < Joy2YDirection)
    {
      Joy2YEase += DomeYEase;
    }

    Joy2XEaseMap = Joy2XEase;
    Joy2YEaseMap = Joy2YEase;

    if (Joy2YEaseMap < 0)
    {
      Joy2Ya = map(Joy2YEaseMap, -20, 0, 70, 0);
      Joy2XLowOffset = map(Joy2Ya, 1, 70, -15, -50);
      Joy2XHighOffset = map(Joy2Ya, 1, 70, 30, 20);
    }
    else if (Joy2YEaseMap > 0)
    {
      Joy2Ya = map(Joy2YEaseMap, 0, 24, 0, -80);
      Joy2XLowOffset = map(Joy2Ya, -1, -80, -15, 10);
      Joy2XHighOffset = map(Joy2Ya, -1, -80, 30, 90);
    }
    else
    {
      Joy2Ya = 0;
    }

    if (Joy2XEaseMap > 0)
    {
      Joy2XLowOffsetA = map(Joy2XEaseMap, 0, 18, 0, Joy2XLowOffset);
      Joy2XHighOffsetA = map(Joy2XEaseMap, 0, 18, 0, Joy2XHighOffset);
      ServoLeft = Joy2Ya + Joy2XHighOffsetA;
      ServoRight = Joy2Ya + Joy2XLowOffsetA;
    }
    else if (Joy2XEaseMap < 0)
    {
      Joy2XLowOffsetA = map(Joy2XEaseMap, -18, 0, Joy2XLowOffset, 0);
      Joy2XHighOffsetA = map(Joy2XEaseMap, -18, 0, Joy2XHighOffset, 0);
      ServoRight = Joy2Ya + Joy2XHighOffsetA;
      ServoLeft = Joy2Ya + Joy2XLowOffsetA;
    }
    else
    {
      Joy2XHighOffsetA = 0;
      Joy2XLowOffsetA = 0;
      ServoRight = Joy2Ya;
      ServoLeft = Joy2Ya;
    }

    leftServo.write(constrain(map(ServoLeft, 90, -90, 0, 180), 0, 180) + 5, domeSpeed, false);
    rightServo.write(constrain(map(ServoRight, 90, -90, 180, 0), 0, 180), domeSpeed, false);

    // FWD/REV Drive
    int speed = map(sbus_rx.rx_channels()[CH_DRIVE_MAIN], RC_MIN, RC_MAX, -255, 255);
    if (in_rc_deadband(sbus_rx.rx_channels()[CH_DRIVE_MAIN]))
    {
      driveController.Stop();
    }
    else
    {
      driveController.Enable();
      if (speed < 0)
      {
        driveController.TurnLeft(abs(speed));
      }
      else
      {
        driveController.TurnRight(speed);
      }
    }

    // Flywheel
    int flywheelSpeed = map(sbus_rx.rx_channels()[CH_FLYWHEEL], RC_MIN, RC_MAX, -255, 255);
    if (in_rc_deadband(sbus_rx.rx_channels()[CH_FLYWHEEL]))
    {
      flywheelController.Stop();
    }
    else
    {
      if (flywheelSpeed < 0)
      {
        flywheelController.TurnRight(abs(flywheelSpeed));
      }
      else
      {
        flywheelController.TurnLeft(flywheelSpeed);
      }
    }

    // S2S Drive
    int joystickS2SSpeed = map(sbus_rx.rx_channels()[CH_DRIVE_S2S], RC_MIN, RC_MAX, -255, 255);
    if ((joystickS2SSpeed > -S2SEase) && (Setpoint2 < S2SEase) && (joystickS2SSpeed == 0)) {
      Setpoint2 = 0;
    } else if ((joystickS2SSpeed > Setpoint2) && (joystickS2SSpeed != Setpoint2)) {
      Setpoint2 += S2SEase;
    } else if ((joystickS2SSpeed < Setpoint2) && (joystickS2SSpeed != Setpoint2)) {
      Setpoint2 -= S2SEase;
    }

    #ifdef reverseS2SPot
        S2Spot = map(analogRead(S2S_POT), 0, 1024, -135, 135);
    #else
        S2Spot = map(analogRead(S2S_POT), 0, 1024, 135, -135);
    #endif

    // PID2 is used to control the 'servo' control of the side to side movement.
    Input2 = roll + rollOffset;
    Setpoint2 = constrain(Setpoint2, -maxS2STilt, maxS2STilt);
    PID2.Compute();

    // PID1 is for side to side stabilization
    Input1  = S2Spot + potOffsetS2S;
    Setpoint1 = map(constrain(Output2, -maxS2STilt, maxS2STilt), -maxS2STilt, maxS2STilt, maxS2STilt, -maxS2STilt);
    PID1.Compute();

    if ((Output1 <= -1) && (Input1 > -maxS2STilt)) {
      Output1a = abs(Output1);
      s2sController.TurnRight(Output1a);
    } else if ((Output1 >= 1) && (Input1 < maxS2STilt)) {
      Output1a = abs(Output1);
      s2sController.TurnLeft(Output1a);
    } else {
      s2sController.Stop();
    }
  }
}

bool in_rc_deadband(int value)
{
  return false;
  //  return value >= RC_DEADBAND_LOW && value <= RC_DEADBAND_HIGH;
}

void setS2SOffset() {
  //  if (recIMUData.roll < 0) {
  //    rollOffset = abs(recIMUData.roll);
  //  } else {
  //    rollOffset = recIMUData.roll * -1;
  //  }

  if (S2Spot < 0) {
    potOffsetS2S = abs(S2Spot);
  } else {
    potOffsetS2S = S2Spot * -1;
  }
  //  EEPROM.writeFloat(4, rollOffset);
  //  EEPROM.writeInt(8, potOffsetS2S);
}
