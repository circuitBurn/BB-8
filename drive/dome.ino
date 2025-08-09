int domeRaw;
int domeSpeed;

/**
   Dome spin is assigned to the self-centering pot of a 3-axis joystick.

   Turning the knob fully clockwise or counter-clockwise will continuously
   spin the dome. Otherwise, the dome will "point" and hold position
   in the general direction of the knob.
*/
void domeSpin()
{
  domeRaw = sbus_rx.data().ch[CH_DOME_SPIN];

  if (domeRaw > 250 && domeRaw < 1700)
  {
    Setpoint4 = map(domeRaw, RC_MIN, RC_MAX, 0, 1024);
    Input4 = getDomeTarget(analogRead(DOME_POT_PIN));
    PID4.Compute();
    domeSpeed = constrain((int)Output4, -255, 255);
    domeSpeed = constrain(domeSpeed, -DOME_TURN_SPEED, DOME_TURN_SPEED);
  }
  else if (domeRaw <= 250)
  {
    domeSpeed = -DOME_SPIN_SPEED;
  }
  else if (domeRaw >= 1700)
  {
    domeSpeed = DOME_SPIN_SPEED;
  }

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

int getDomeTarget(int val)
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
