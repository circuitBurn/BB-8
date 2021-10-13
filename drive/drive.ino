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

#define domeSpinPWM1 10
#define domeSpinPWM2 11

#define DOME_POT A4
#define MaxDomeTiltY 25
#define MaxDomeTiltX 30

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
#define CH_DRIVES_EN 9

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
#define S2S_MAX_TILT 26
#define S2S_EASE 5.5
#define S2S_OFFSET 6

// Dome
#define DomeYEase 1.5 // Spead of front to back dome movement, higher == faster
#define DomeXEase 2.5 // Speed of side to side domemovement, higher == faster

// PID1 is for the side to side tilt
double Pk1 = 32.0;
double Ik1 = 0.0;
double Dk1 = 0.0;
double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID for S2S stability
double Pk2 = 1.0;
double Ik2 = 0.0;
double Dk2 = 0.01;
double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

// PID for the main drive
double Pk3 = 5.0;
double Ik3 = 0.0;
double Dk3 = 0.0;
double Setpoint3, Input3, Output3, Output3a;
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);

// PID for dome rotation
double Pk4 = 3.0;
double Ik4 = 0.1;
double Dk4 = 0.01;
double Setpoint4, Input4, Output4;
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);

// Sound
int soundTriggerState;
int lastSoundTriggerState = LOW;
bool soundTriggerLatched = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Offsets
float pitchOffset, rollOffset, potOffsetS2S, domeTiltPotOffset, domeSpinOffset;

// Dome Servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
int domeTiltXRaw, domeTiltYRaw;
int domeTiltX, domeTiltY;
double Joy2YPitch;
int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase, Joy2XEaseMap;
double Joy2YEaseMap;

int domeRaw, domeSpeed;
int S2Spot, s2sRaw, s2sSpeed;
int driveRaw, driveSpeed;
int flywheelRaw, flywheelSpeed;
int soundBank, soundTriggerRaw, soundRaw;
bool motorsEnabled = true;

// SBUS
SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// Neck Servos
//Servo leftServo;
//Servo rightServo;

// BTS7960 Motor Drivers
BTS7960 driveController(DRIVE_EN, DRIVE_L_PWM, DRIVE_R_PWM);
BTS7960 s2sController(S2S_EN, S2S_L_PWM, S2S_R_PWM);
BTS7960 flywheelController(DRIVE_EN, FLY_L_PWM, FLY_R_PWM);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
float pitch, roll;

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

void setup()
{
  // SBUS
  sbus_rx.Begin();

  Serial.begin(115200);

  randomSeed(analogRead(0));

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
  servos.begin();
  servos.setPWMFreq(60);
  servos.writeMicroseconds(13, 1500);
  servos.writeMicroseconds(14, 1500);

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

  // PID Setup dome motor
  PID4.SetMode(AUTOMATIC);
  PID4.SetOutputLimits(-255, 255);
  PID4.SetSampleTime(15);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(500);

  // TODO: loadOffsets
  pitchOffset = -0.55; //4.4;
  rollOffset = -3.5; //5.75;
  potOffsetS2S = 3;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = potOffsetS2S;
  }

  // Sound
  Serial1.begin(9600);
  myDFPlayer.begin(Serial1);
  //  myDFPlayer.volume(30);
  //  myDFPlayer.play(2);
}

void loop()
{
  readIMU();

  if (sbus_rx.Read())
  {
    motorsEnabled = areMotorsEnabled();
    if (motorsEnabled)
    {
      mainDrive();
      flyWheel();
      side2Side();
    }
    domeSpin();
    domeServos();
    checkSoundTrigger();
    //    debugPrint();
  }
}

void domeSpin()
{
  domeRaw = sbus_rx.rx_channels()[CH_DOME_SPIN];
  Setpoint4 = map(domeRaw, RC_MIN, RC_MAX, 0, 1024);
  Input4 = analogRead(DOME_POT);
  PID4.Compute();
  domeSpeed = constrain((int)Output4, -255, 255);
  if (domeSpeed >= 1)
  {
    analogWrite(domeSpinPWM1, abs(domeSpeed));
    analogWrite(domeSpinPWM2, 0);
  }
  else if (domeSpeed <= -1)
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

int domeTiltFwd, domeTiltFwda, domeTiltSide;
int ch3, ch3a, ch4, ch4a;
int current_pos_head1;  // variables for smoothing head back/forward
int target_pos_head1;
int pot_head1;   // target position/inout
int diff_head1; // difference of position
double easing_head1;

int varServo1;
int varServo2;

void domeServos()
{
  // Forwards-backwards
  ch3 = sbus_rx.rx_channels()[CH_DOME_TILT_Y];
  ch3a = map(ch3, RC_MIN, RC_MAX, 180, 0);
//  ch3a = ch3a - (pitch * 4.5);

  target_pos_head1 = ch3a;

  easing_head1 = 100;          //modify this value for sensitivity
  easing_head1 /= 1000;

  diff_head1 = target_pos_head1 - current_pos_head1;

  // Avoid any strange zero condition
  if ( diff_head1 != 0.00 )
  {
    current_pos_head1 += diff_head1 * easing_head1;
  }

  varServo1 = current_pos_head1;
  varServo2 = current_pos_head1;

  // Left-right
  ch4 = sbus_rx.rx_channels()[CH_DOME_TILT_X];
  ch4a = map(ch4, RC_MIN, RC_MAX, 35, -35);

  varServo1 = varServo1 - ch4a;
  varServo2 = varServo2 + ch4a;

  varServo1 = constrain(varServo1, 0, 180);
  varServo2 = constrain(varServo2, 0, 180);

  // Reverse the servo value
  varServo2 = map(varServo2, 0, 180, 180, 0);

  Serial.print(varServo1);
  Serial.print("\t");
  Serial.println(varServo2);

  varServo1 = map(varServo1, 0, 180, 1000, 2000);
  varServo2 = map(varServo2, 0, 180, 1000, 2000);

  servos.writeMicroseconds(13, varServo1);
  servos.writeMicroseconds(14, varServo2);
}

void domeServos2()
{
  dome();
  return;
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
    Joy2XLowOffsetA = map(Joy2XEaseMap, 0, 25, 0, Joy2XLowOffset);
    Joy2XHighOffsetA = map(Joy2XEaseMap, 0, 25, 0, Joy2XHighOffset);
    ServoLeft = Joy2Ya + Joy2XHighOffsetA;
    ServoRight = Joy2Ya + Joy2XLowOffsetA;
  }
  else if (Joy2XEaseMap < 0)
  {
    Joy2XLowOffsetA = map(Joy2XEaseMap, -25, 0, Joy2XLowOffset, 0);
    Joy2XHighOffsetA = map(Joy2XEaseMap, -25, 0, Joy2XHighOffset, 0);
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
  int leftMicros = map(ServoLeft + 20, 90, -90, 1000, 2000);
  int rightMicros = map(ServoRight, 90, -90, 2000, 1000);

  servos.writeMicroseconds(13, leftMicros);
  servos.writeMicroseconds(14, rightMicros);
}

void side2Side()
{
  s2sRaw = sbus_rx.rx_channels()[CH_DRIVE_S2S];
  s2sSpeed = map(s2sRaw, RC_MIN, RC_MAX, -255, 255);

  if ((s2sSpeed > -S2S_EASE) && (Setpoint2 < S2S_EASE) && (s2sSpeed >= -2 && s2sSpeed <= 2))
  {
    Setpoint2 = 0;
  }
  else if ((s2sSpeed > Setpoint2) && (s2sSpeed != Setpoint2))
  {
    Setpoint2 += S2S_EASE;
  }
  else if ((s2sSpeed < Setpoint2) && (s2sSpeed != Setpoint2))
  {
    Setpoint2 -= S2S_EASE;
  }

  // subtract the last reading:
  total = total - readings[readIndex];

#ifdef reverseS2SPot
  readings[readIndex] = map(analogRead(S2S_POT), 0, 1024, -135, 135);
#else
  readings[readIndex] = map(analogRead(S2S_POT), 0, 1024, 135, -135);
#endif


  // read from the sensor:
  //  readings[readIndex] = S2Spot;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  S2Spot = average;

  //  Serial.println(roll);

  // PID2 is used to control the 'servo' control of the side to side movement.
  Input2 = roll - rollOffset;
  Setpoint2 = constrain(Setpoint2, -S2S_MAX_TILT, S2S_MAX_TILT);
  PID2.Compute();

  // PID1 is for side to side stabilization
  Input1 = S2Spot + potOffsetS2S;
  Setpoint1 = map(constrain(Output2, -S2S_MAX_TILT, S2S_MAX_TILT), -S2S_MAX_TILT, S2S_MAX_TILT, S2S_MAX_TILT, -S2S_MAX_TILT);
  PID1.Compute();

  if ((Output1 <= -1) && (Input1 > -S2S_MAX_TILT)) {
    Output1a = abs(Output1);
    s2sController.TurnRight(Output1a);
  } else if ((Output1 >= 1) && (Input1 < S2S_MAX_TILT)) {
    Output1a = abs(Output1);
    s2sController.TurnLeft(Output1a);
  } else {
    s2sController.Stop();
  }
}

void debugPrint()
{
  Serial.print("S2S: ");
  Serial.print(analogRead(S2S_POT));
  Serial.print("\t");

  Serial.print("S2S mapped: ");
  Serial.print(map(analogRead(S2S_POT), 0, 1024, -135, 135));
  Serial.print("\t");

  Serial.print("Input1: ");
  Serial.print(Input1);
  Serial.print("\t");

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\t");

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\t");

  Serial.println();
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

void mainDrive()
{
  driveRaw = sbus_rx.rx_channels()[CH_DRIVE_MAIN];
  driveSpeed = map(driveRaw, RC_MIN, RC_MAX, 255, -255);

  Setpoint3 = constrain(driveSpeed, -55, 55);
  Input3 = (pitch + pitchOffset);// + (Joy2YEaseMap /= 2);// - domeOffset;
  PID3.Compute();

  if (Output3 >= 1)
  {
    Output3a = abs(Output3);
    driveController.TurnLeft(abs(Output3));
  }
  else if (Output3 < -1)
  {
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
