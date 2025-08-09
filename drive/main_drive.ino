int driveRaw, driveSpeed;

/**
   Forwards and backwards
*/
void mainDrive()
{
  driveRaw = sbus_rx.data().ch[CH_DRIVE_MAIN];
  driveSpeed = getDriveSpeed(driveRaw);

  Setpoint3 = constrain(driveSpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  Input3 = pitch;
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

/**
 * Returns the drive speed oriented to the specified "drive direction"
 */
int getDriveSpeed(int driveSpeedRaw)
{
  if (driveDirection == DriveDirection::Forward)
  { 
    return map(driveSpeedRaw, RC_MIN, RC_MAX, MAX_DRIVE_SPEED, -MAX_DRIVE_SPEED);
  }
  else
  {
    return map(driveSpeedRaw, RC_MIN, RC_MAX, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  }
}
