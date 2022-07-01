int ch2, pot;
int s2s_current_position;
int s2s_target_position;
int s2s_position_difference;
double s2s_easing;

void side_to_side()
{
  // TODO: sort this variable out!
  int val = 65;
  ch2 = sbus_rx.rx_channels()[CH_DRIVE_S2S];

  s2s_target_position = map(ch2, RC_MIN, RC_MAX, val, -val);

  s2s_easing = 80.0;
  s2s_easing /= 1000.0;

  // Work out the required travel.
  s2s_position_difference = s2s_target_position - s2s_current_position;

  // Avoid any strange zero condition
  if ( s2s_position_difference != 0.00 )
  {
    s2s_current_position += s2s_position_difference * s2s_easing;
  }

  Setpoint2 = s2s_current_position;

  // TODO: Rolling average
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(S2S_POT) + S2S_OFFSET;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  // Wrap to beginning
  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }

  // calculate the average:
  average = (float)total / (float)numReadings;
  pot = average;

  //  pot = analogRead(S2S_POT) + S2S_OFFSET;

    Input2 = (roll + rollOffset) * -1;
//  Input2 = roll * -1;
  Setpoint2 = constrain(Setpoint2, -val, val);

  //  // Update PK2 from RC
  //  Pk2 = get_pk2();
  //  PID2.SetTunings(Pk2, Ik2, Dk2);
  PID2.Compute();

  Setpoint1 = Output2;

  Input1 = map(pot, 0, 1024, -255, 255);

  Input1 = constrain(Input1, -val, val);
  Setpoint1 = constrain(Setpoint1, -val, val);
  Setpoint1 = map(Setpoint1, val, -val, -val, val);

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
