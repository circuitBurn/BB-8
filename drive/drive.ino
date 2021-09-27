#include "Arduino.h"
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"
#include <VarSpeedServo.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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
// The channels are N - 1 where N is the defined channel number on the transmitter
// Ex: CH1 is 0, CH2 is 1
#define CH_DRIVE_MAIN 0
#define CH_DOME_TILT_X 1
#define CH_DOME_TILT_Y 2
#define CH_DRIVE_S2S 3
#define CH_FLYWHEEL 4
#define CH_DOME_SPIN 5
#define CH_SOUND_TRIGGER 7
#define CH_SOUND_BANK 8

// Main Drive
#define DRIVE_R_PWM 13
#define DRIVE_L_PWM 12
#define DRIVE_EN 29

// Flywheel
#define FLY_R_PWM 8
#define FLY_L_PWM 9

// Side to side
#define S2S_R_PWM 6
#define S2S_L_PWM 7
#define S2S_EN 33
#define S2S_POT A0
#define maxS2STilt 25
//#define reverseS2SPot
#define S2SEase 5.5

/**
   Gyro:

   Leaning forward = +y
   Tilting left    = -z
*/

// Dome
#define DomeYEase .4 // Spead of front to back dome movement, higher == faster
#define DomeXEase .7 // Speed of side to side domemovement, higher == faster
#define DomeServoSpeed 180 // Speed that the servos will move

// PID1 is for the side to side tilt
double Pk1 = 14;
double Ik1 = 0;
double Dk1 = 0.0;
double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID Setup - S2S stability
double Pk2 = .5;
double Ik2 = 0;
double Dk2 = .01;
double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

//PID3 is for the main drive
double Pk3 = 5;
double Ik3 = 0;
double Dk3 = 0;
double Setpoint3, Input3, Output3, Output3a;
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);

// Sound
int soundTriggerState;
int lastSoundTriggerState = LOW;
bool soundTriggerLatched = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Dome Servos
int domeTiltXRaw, domeTiltYRaw;
int domeTiltX, domeTiltY;

double Joy2YPitch;
int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase, Joy2XEaseMap;
double Joy2YEaseMap;
double pitchOffset = -0.75;//4.4;
double rollOffset = 5;

// TODO:
int s2sPot = 512;

int potOffsetS2S = 3;
int S2Spot;

int domeRaw, domeSpeed;
int s2sRaw, s2sSpeed;
int driveRaw, driveSpeed;
int flywheelRaw, flywheelSpeed;
int soundBank, soundTriggerRaw, soundRaw;

// SBUS
SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// Neck Servos
VarSpeedServo leftServo;
VarSpeedServo rightServo;
bool servosEnabled = false;

// BTS7960 Motor Drivers
BTS7960 driveController(DRIVE_EN, DRIVE_L_PWM, DRIVE_R_PWM);
BTS7960 s2sController(S2S_EN, S2S_L_PWM, S2S_R_PWM);
BTS7960 flywheelController(DRIVE_EN, FLY_L_PWM, FLY_R_PWM);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
float pitch, roll;

void setup()
{
  // SBUS
  sbus_rx.Begin();

  Serial.begin(115200);

  // Pinmodes
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
  leftServo.attach(leftDomeTiltServo);
  rightServo.attach(rightDomeTiltServo);
  leftServo.write(90, 50, false);
  rightServo.write(90, 50, false);

  // Motor Drivers
  driveController.Enable();
  s2sController.Enable();
  flywheelController.Enable();

  // S2S Servo PID
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(15);

  // S2S Stability PID
  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(15);

  // PID Setup - main drive motor
  PID3.SetMode(AUTOMATIC);
  PID3.SetOutputLimits(-255, 255);
  PID3.SetSampleTime(15);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  // Sound
  Serial1.begin(9600);
  myDFPlayer.begin(Serial1);
  myDFPlayer.volume(30);
  myDFPlayer.play(1);
}

bool motorsEnabled = true;

void loop()
{
  //  Serial.println(analogRead(A4));

  updateIMU();

  if (sbus_rx.Read())
  {
    //    motorsEnabled = areMotorsEnabled();
    if (motorsEnabled)
    {
      mainDrive();
      flyWheel();
      side2Side();
    }
    domeSpin();
    domeServos();
    checkSoundTrigger();
  }
}



void disableDrive()
{
  servosEnabled = false;
  flywheelController.Disable();
  s2sController.Disable();
  driveController.Disable();
}

void enableDrive()
{
  servosEnabled = true;
  flywheelController.Disable();
  s2sController.Disable();
  driveController.Disable();
}

void loadCalibration()
{
  // TODO;
//  pitchOffset = EEPROM.readFloat(0);
//  rollOffset = EEPROM.readFloat(4);
//  potOffsetS2S = EEPROM.readInt(8);
//  domeTiltPotOffset = EEPROM.readInt(10);
//  domeSpinOffset = EEPROM.readInt(12);
}

void calibrate()
{
  // TODO: read pitch
  // TODO: read roll
  // TODO: update EEPROM
}


//bool areMotorsEnabled()
//{
//  return map(sbus_rx.rx_channels()[CH_MOTORS_ENABLED]RC_MIN, RC_MAX, 0, 1) == 1;
//}

void domeSpin()
{
  domeRaw = sbus_rx.rx_channels()[CH_DOME_SPIN];
  domeSpeed = map(domeRaw, RC_MIN, RC_MAX, -255, 255);
  if (domeSpeed < 25 && domeSpeed > -25)
  {
    domeSpeed = 0;
  }

  if (domeSpeed <= -20)
  {
    analogWrite(domeSpinPWM1, abs(domeSpeed));
    analogWrite(domeSpinPWM2, 0);
  }
  else if (domeSpeed >= 20)
  {
    analogWrite(domeSpinPWM1, 0);
    analogWrite(domeSpinPWM2, abs(domeSpeed));
  }
  else
  {
    analogWrite(domeSpinPWM1, 0);
    analogWrite(domeSpinPWM2, 0);
  }
}

void domeServos()
{
  if (!servosEnabled)
  {
    leftServo.write(90);
    rightServo.write(90);
    return;
  }

  domeTiltYRaw = sbus_rx.rx_channels()[CH_DOME_TILT_Y];
  domeTiltXRaw = sbus_rx.rx_channels()[CH_DOME_TILT_X];
  domeTiltX = map(domeTiltXRaw, RC_MIN, RC_MAX, MaxDomeTiltX, -MaxDomeTiltX);
  domeTiltY = map(domeTiltYRaw, RC_MAX, RC_MIN, -MaxDomeTiltY, MaxDomeTiltY);

  // X direction
  if ((domeTiltX + DomeXEase) > Joy2XEase && (domeTiltX - DomeXEase) < Joy2XEase)
  {
    Joy2XEase = domeTiltX;
  }
  else if (Joy2XEase > domeTiltX)
  {
    Joy2XEase -= DomeXEase;
  }
  else if (Joy2XEase < domeTiltX)
  {
    Joy2XEase += DomeXEase;
  }

  // Y direction
  if ((domeTiltY + DomeYEase) > Joy2YEase && (domeTiltY - DomeYEase) < Joy2YEase)
  {
    Joy2YEase = domeTiltY;
  }
  else if (Joy2YEase > domeTiltY)
  {
    Joy2YEase -= DomeYEase;
  }
  else if (Joy2YEase < domeTiltY)
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

  leftServo.write(constrain(map(ServoLeft, 90, -90, 0, 180), 0, 180) + 5, DomeServoSpeed, false);
  rightServo.write(constrain(map(ServoRight, 90, -90, 180, 0), 0, 180), DomeServoSpeed, false);
}

void side2Side()
{
  s2sRaw = sbus_rx.rx_channels()[CH_DRIVE_S2S];
  s2sSpeed = map(s2sRaw, RC_MIN, RC_MAX, -255, 255);

  if ((s2sSpeed > -S2SEase) && (Setpoint2 < S2SEase) && (s2sSpeed >= -2 && s2sSpeed <= 2)) {
    Setpoint2 = 0;
  } else if ((s2sSpeed > Setpoint2) && (s2sSpeed != Setpoint2)) {
    Setpoint2 += S2SEase;
  } else if ((s2sSpeed < Setpoint2) && (s2sSpeed != Setpoint2)) {
    Setpoint2 -= S2SEase;
  }

#ifdef reverseS2SPot
  S2Spot = map(analogRead(S2S_POT), 0, 1024, -135, 135);
#else
  S2Spot = map(analogRead(S2S_POT), 0, 1024, 135, -135);
#endif

  // PID2 is used to control the 'servo' control of the side to side movement.
  Input2 = 0;//roll + rollOffset;
  Setpoint2 = constrain(Setpoint2, -maxS2STilt, maxS2STilt);
  PID2.Compute();

  // PID1 is for side to side stabilization
  Input1 = S2Spot + potOffsetS2S;
  Setpoint1 = map(constrain(Output2, -maxS2STilt, maxS2STilt), -maxS2STilt, maxS2STilt, maxS2STilt, -maxS2STilt);
  PID1.Compute();

  Serial.println(Output1);

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

/**
   Gyro:

   Leaning forward = +y
   Tilting left    = -z

   Pitch is leaning forawrds/backwards - Y axis
   Roll is leaning sideways - Z axis
*/

void updateIMU()
{
  // Get IMU data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
  //  Serial.print("Pitch: ");
  //  Serial.print(pitch);
  //  Serial.print("\t");
  //  Serial.print("Roll: ");
  //  Serial.print(roll);
  //  Serial.print("\t");
  //  Serial.println();
}

void mainDrive()
{
  driveRaw = sbus_rx.rx_channels()[CH_DRIVE_MAIN];
  driveSpeed = map(driveRaw, RC_MIN, RC_MAX, 255, -255);

  Setpoint3 = constrain(driveSpeed, -55, 55);
  Input3 = (pitch + pitchOffset) + (Joy2YEaseMap /= 2);// - domeOffset;
  PID3.Compute();

  if (Output3 >= 2) {   //make BB8 roll
    Output3a = abs(Output3);
    driveController.TurnLeft(abs(Output3));
  } else if (Output3 < -2) {
    Output3a = abs(Output3);
    driveController.TurnRight(abs(Output3));
  }
  else
  {
    driveController.Stop();
  }
}

void flyWheel()
{
  flywheelRaw = sbus_rx.rx_channels()[CH_FLYWHEEL];
  flywheelSpeed = map(flywheelRaw, RC_MIN, RC_MAX, -255, 255);

  if (in_rc_deadband(flywheelRaw))
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
}

void checkSoundTrigger()
{
  soundTriggerRaw = sbus_rx.rx_channels()[CH_SOUND_TRIGGER];

  if (soundTriggerRaw != lastSoundTriggerState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the soundTriggerRaw is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (soundTriggerRaw != soundTriggerState) {
      soundTriggerState = soundTriggerRaw;

      if (!soundTriggerLatched && soundTriggerRaw == RC_MAX)
      {
        soundTriggerLatched = true;
      }
      else if (soundTriggerLatched && soundTriggerRaw == RC_MIN)
      {
        soundTriggerLatched = false;
        playSound();
      }
    }
  }

  lastSoundTriggerState = soundTriggerRaw;
}

void playSound()
{
  soundRaw = sbus_rx.rx_channels()[CH_SOUND_BANK];
  soundBank = map(soundRaw, 172, 1811, 0, 2);
  if (soundBank == 2)
  {
    // Star Wars sounds
    myDFPlayer.play(random(50, 53));
  }
  else
  {
    // BB-8 Sounds
    myDFPlayer.play(random(0, 49));
  }
}

bool in_rc_deadband(int value)
{
  return value >= RC_DEADBAND_LOW && value <= RC_DEADBAND_HIGH;
}

//void setS2SOffset() {
//  //  if (recIMUData.roll < 0) {
//  //    rollOffset = abs(recIMUData.roll);
//  //  } else {
//  //    rollOffset = recIMUData.roll * -1;
//  //  }
//
//  if (S2Spot < 0) {
//    potOffsetS2S = abs(S2Spot);
//  } else {
//    potOffsetS2S = S2Spot * -1;
//  }
//  //  EEPROM.writeFloat(4, rollOffset);
//  //  EEPROM.writeInt(8, potOffsetS2S);
//}
