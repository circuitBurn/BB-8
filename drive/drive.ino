#include "Arduino.h"
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"
#include <VarSpeedServo.h>
#include "BTS7960.h"

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
#define CH_DOME_SPIN 5

// TODO:
#define DomeYEase .4 // Spead of front to back dome movement, higher == faster
#define DomeXEase .7 // Speed of side to side domemovement, higher == faster
#define domeSpeed 90 // Speed that the servos will move

// TODO:
int rcDome = 0;
double Joy2YPitch, Joy2YDirection, Joy2XDirection;
int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase, Joy2XEaseMap;
double Joy2YEaseMap;

// SBUS
SbusRx sbus_rx(&Serial2);

// Sound
DFRobotDFPlayerMini myDFPlayer;

// Neck Servos
VarSpeedServo leftServo;
VarSpeedServo rightServo;

// BTS7960 Motor Drivers
// TODO: set up correct pinouts
BTS7960 driveController(EN, L_PWM, R_PWM);
BTS7960 s2sController(EN, L_PWM, R_PWM);
BTS7960 flywheelController(EN, L_PWM, R_PWM);

void setup()
{
    /* Serial to display the received data */
    //  Serial.begin(115200);
    //  while (!Serial) {}

    // SBUS
    sbus_rx.Begin();

    // Sound
    Serial1.begin(9600);
    myDFPlayer.begin(Serial2);
    myDFPlayer.volume(10);
    myDFPlayer.play(1);

    // Servos
    leftServo.attach(leftDomeTiltServo);
    rightServo.attach(rightDomeTiltServo);
    leftServo.write(95, 50, false);
    rightServo.write(90, 50, false);

    // Motor Drivers
    driveController.Enable();
    s2sController.Enable();
    flywheelController.Enable();
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

        leftServo.write(constrain(map(ServoLeft, -90, 90, 0, 180), 0, 180) + 5, domeSpeed, false);
        rightServo.write(constrain(map(ServoRight, -90, 90, 180, 0), 0, 180), domeSpeed, false);

        // FWD/REV Drive
        int speed = map(sbus_rx.rx_channels()[CH_DRIVE_MAIN], RC_MIN, RC_MAX, -255, 255);
        if (in_rc_deadband(sbus_rx.rx_channels()[CH_DRIVE_MAIN]))
        {
            driveController.Disable();
        }
        else
        {
            driveController.Enable();
            if (speed < 0)
            {
                driveController.TurnRight(abs(speed));
            }
            else
            {
                driveController.TurnLeft(speed);
            }
        }

        // S2S Drive
    }
}

bool in_rc_deadband(int value)
{
    return  >= RC_DEADBAND_LOW && value <= RC_DEADBAND_HIGH;
}