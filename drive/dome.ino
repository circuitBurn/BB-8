int domeRaw;
int domeSpeed;
int targetNod;
int currentNod;
int diffNod;
double easing_head1;
double easing_head2;
int varServo1;
int varServo2;
int targetSide;
int diffSide;
int currentSide;
int ch3, ch3a, ch4, ch4a;

/**
   Dome spin is assigned to the self-centering pot of a 3-axis joystick.

   Turning the knob fully clockwise or counter-clockwise will continuously
   spin the dome. Otherwise, the dome will "point" and hold position
   in the general direction of the knob.
*/
void dome_spin()
{
  domeRaw = sbus_rx.rx_channels()[CH_DOME_SPIN];

  if (domeRaw > 250 && domeRaw < 1700)
  {
    Setpoint4 = map(domeRaw, RC_MIN, RC_MAX, 0, 1024);
    Input4 = analogRead(DOME_POT) + 15;
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

  constrain(domeSpeed, -127, 127);

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

void dome_servos()
{
  // Forwards-backwards
  ch3 = sbus_rx.rx_channels()[CH_DOME_TILT_Y];
  ch3a = map(ch3, RC_MIN, RC_MAX, 180, 0);
  //  ch3a = ch3a + (pitch * 4.5);

  targetNod = ch3a;
  easing_head1 = 75;
  easing_head1 /= 1000;

  diffNod = targetNod - currentNod;

  // Avoid any strange zero condition
  if ( diffNod != 0.00 )
  {
    currentNod += diffNod * easing_head1;
  }

  varServo1 = currentNod;
  varServo2 = currentNod;

  // Left-right
  ch4 = sbus_rx.rx_channels()[CH_DOME_TILT_X];
  ch4a = map(ch4, RC_MIN, RC_MAX, 60, -60);

  targetSide = ch4a;
  easing_head2 = 75;
  easing_head2 /= 1000;

  diffSide = targetSide - currentSide;

  // Avoid any strange zero condition
  if ( diffSide != 0.00 )
  {
    currentSide += diffSide * easing_head2;
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
