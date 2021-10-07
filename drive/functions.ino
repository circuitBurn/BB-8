bool areMotorsEnabled()
{
  return true;
  //  return map(sbus_rx.rx_channels()[CH_DRIVES_EN]RC_MIN, RC_MAX, 0, 1) == 1;
}

bool in_rc_deadband(int value)
{
  return value >= RC_DEADBAND_LOW && value <= RC_DEADBAND_HIGH;
}

void disableDrive()
{
  flywheelController.Disable();
  s2sController.Disable();
  driveController.Disable();
}

void enableDrive()
{
  flywheelController.Disable();
  s2sController.Disable();
  driveController.Disable();
}
