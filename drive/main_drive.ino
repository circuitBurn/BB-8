int driveRaw, driveSpeed;

/**
   Forwards and backwards
*/
void main_drive()
{
  driveRaw = sbus_rx.rx_channels()[CH_DRIVE_MAIN];
  driveSpeed = map(driveRaw, RC_MIN, RC_MAX, MAX_DRIVE_SPEED, -MAX_DRIVE_SPEED);

  Setpoint3 = constrain(driveSpeed, -55, 55);
  Input3 = pitch + pitchOffset;
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