void checkSoundTrigger()
{
  soundTriggerRaw = sbus_rx.rx_channels()[CH_SOUND_TRIGGER];

  if (soundTriggerRaw != lastSoundTriggerState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the soundTriggerRaw is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (soundTriggerRaw != soundTriggerState) {
      soundTriggerState = soundTriggerRaw;

      if (!soundTriggerLatched && soundTriggerRaw == RC_MAX)
      {
        soundTriggerLatched = true;
      }
      else if (soundTriggerLatched && soundTriggerRaw == RC_MIN)
      {
        soundTriggerLatched = false;
        playSound();
      }
    }
  }

  lastSoundTriggerState = soundTriggerRaw;
}

void playSound()
{
  soundRaw = sbus_rx.rx_channels()[CH_SOUND_BANK];
  soundBank = map(soundRaw, 172, 1811, 0, 2);
  if (soundBank == 2)
  {
    // Star Wars sounds
    myDFPlayer.play(random(50, 53));
  }
  else
  {
    // BB-8 Sounds
    myDFPlayer.play(random(0, 49));
  }
}
