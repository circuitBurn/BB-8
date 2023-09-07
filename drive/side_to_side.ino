int ch2, pot;
int s2s_current_position;
int s2s_target_position;
int s2s_position_difference;
double s2s_offset;

void side_to_side()
{
  ch2 = sbus_rx.data().ch[CH_DRIVE_S2S];

  // Get the s2s offset from the right slider
  s2s_offset = sbus_rx.data().ch[CH_S2S_OFFSET];
  s2s_offset = map(s2s_offset, RC_MIN, RC_MAX, -10, 50);

  s2s_target_position = get_target_s2s(ch2);

  // Calculate error
  s2s_position_difference = s2s_target_position - s2s_current_position;

  // Avoid any strange zero condition
  if (s2s_position_difference != 0.00)
  {
    s2s_current_position += s2s_position_difference * S2S_EASING;
  }

  Setpoint2 = s2s_current_position;

  pot = analogRead(S2S_POT_PIN) + s2s_offset;

  Input2 = roll;
  Setpoint2 = constrain(Setpoint2, -S2S_MAX_ANGLE, S2S_MAX_ANGLE);
  //  Pk2 = get_pk2();
  //  PID2.SetTunings(Pk2, Ik2, Dk2);
  PID2.Compute();

  Setpoint1 = Output2;

  Input1 = map(pot, 0, 1024, -255, 255);

  Input1 = constrain(Input1, -S2S_MAX_ANGLE, S2S_MAX_ANGLE);
  Setpoint1 = constrain(Setpoint1, -S2S_MAX_ANGLE, S2S_MAX_ANGLE);
  Setpoint1 = map(Setpoint1, S2S_MAX_ANGLE, -S2S_MAX_ANGLE, -S2S_MAX_ANGLE, S2S_MAX_ANGLE);

  // Update PK1 from RC control
  Pk1 = get_pk1();
  PID1.SetTunings(Pk1, Ik1, Dk1);
  PID1.Compute();

  if (Output1 < 0)
  {
    Output1a = abs(Output1);
    s2sController.TurnLeft(Output1a);
  }
  else if (Output1 > 0)
  {
    Output1a = abs(Output1);
    s2sController.TurnRight(Output1a);
  }
  else
  {
    s2sController.Stop();
  }
}

int get_target_s2s(int val)
{
  if (driveDirection == DriveDirection::Forward)
  {
    return map(val, RC_MIN, RC_MAX, S2S_MAX_ANGLE, -S2S_MAX_ANGLE);
  }
  else // Reversed
  {
    return map(val, RC_MIN, RC_MAX, -S2S_MAX_ANGLE, S2S_MAX_ANGLE);
  }
}
