int domeRaw;
int domeSpeed;
int targetNod;
int currentNod;
int diffNod;
int varServo1;
int varServo2;
int targetSide;
int diffSide;
int currentSide;
int ch3, ch4;

/**
   Dome spin is assigned to the self-centering pot of a 3-axis joystick.

   Turning the knob fully clockwise or counter-clockwise will continuously
   spin the dome. Otherwise, the dome will "point" and hold position
   in the general direction of the knob.
*/
void dome_spin()
{

  if (is_dome_rotation_enabled())
  {
    domeRaw = sbus_rx.data().ch[CH_DOME_SPIN];

    if (domeRaw > 250 && domeRaw < 1700)
    {
      Setpoint4 = map(domeRaw, RC_MIN, RC_MAX, 0, 1024);
      Input4 = get_target_dome_rotation(analogRead(DOME_POT_PIN));
      PID4.Compute();
      domeSpeed = constrain((int)Output4, -255, 255);
    }
    else if (domeRaw <= 250)
    {
      domeSpeed = -255;
    }
    else if (domeRaw >= 1700)
    {
      domeSpeed = 255;
    }

    domeSpeed = constrain(domeSpeed, -DOME_SPIN_SPEED, DOME_SPIN_SPEED);

    if (domeSpeed >= 1)
    {
      analogWrite(DOME_SPIN_A_PIN, abs(domeSpeed));
      analogWrite(DOME_SPIN_B_PIN, 0);
    }
    else if (domeSpeed <= -1)
    {
      analogWrite(DOME_SPIN_A_PIN, 0);
      analogWrite(DOME_SPIN_B_PIN, abs(domeSpeed));
    }
    else
    {
      analogWrite(DOME_SPIN_A_PIN, 0);
      analogWrite(DOME_SPIN_B_PIN, 0);
    }
  }
  else
  {
    analogWrite(DOME_SPIN_A_PIN, 0);
    analogWrite(DOME_SPIN_B_PIN, 0);
  }
}

void dome_servos()
{
  if (is_dome_movement_enabled())
  {
    // Forwards-backwards
    ch3 = sbus_rx.data().ch[CH_DOME_TILT_Y];
    targetNod = get_target_dome_nod(ch3);

    // TODO: pitch correction
    //    targetNod = targetNod + (pitch * 4.5);

    diffNod = targetNod - currentNod;

    // Avoid any strange zero condition
    if (diffNod != 0.00)
    {
      currentNod += diffNod * NOD_EASING;
    }

    varServo1 = currentNod;
    varServo2 = currentNod;

    // Left-right
    ch4 = sbus_rx.data().ch[CH_DOME_TILT_X];
    targetSide = get_target_dome_tilt(ch4);

    diffSide = targetSide - currentSide;

    // Avoid any strange zero condition
    if (diffSide != 0.00)
    {
      currentSide += diffSide * TILT_EASING;
    }

    varServo1 = varServo1 - currentSide;
    varServo2 = varServo2 + currentSide;

    // TODO: map properly
    varServo1 = constrain(varServo1, 0, 180);
    varServo2 = constrain(varServo2, 0, 180);

    // Reverse the servo value
    varServo2 = map(varServo2, 0, 180, 180, 0);

    varServo1 = map(varServo1, 0, 180, 1000, 2000);
    varServo2 = map(varServo2, 0, 180, 1000, 2000);

    servos.writeMicroseconds(13, varServo1);
    servos.writeMicroseconds(14, varServo2);
  }
  else
  {
    //    Serial.println("Dome movement is disabled");
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

bool is_dome_rotation_enabled()
{
  return true; // driveMode == DriveMode::Enabled || driveMode == DriveMode::Static;
}

bool is_dome_movement_enabled()
{
  return driveMode == DriveMode::Enabled || driveMode == DriveMode::Static;
}

int get_target_dome_rotation(int val)
{
  if (driveDirection == DriveDirection::Forward)
  {
    return val + DOME_POT_OFFSET;
  }
  else // Reversed
  {
    return (val + 512 + DOME_POT_OFFSET) % 1024;
  }
}

int get_target_dome_nod(int val)
{
  if (driveDirection == DriveDirection::Forward)
  {
    return map(ch3, RC_MIN, RC_MAX, 180, 0);
  }
  else // Reversed
  {
    return map(ch3, RC_MIN, RC_MAX, 0, 180);
  }
}

int get_target_dome_tilt(int val)
{
  if (driveDirection == DriveDirection::Forward)
  {
    return map(val, RC_MIN, RC_MAX, MAX_DOME_S2S_ANGLE, -MAX_DOME_S2S_ANGLE);
  }
  else // Reversed
  {
    return map(val, RC_MIN, RC_MAX, -MAX_DOME_S2S_ANGLE, MAX_DOME_S2S_ANGLE);
  }
}
