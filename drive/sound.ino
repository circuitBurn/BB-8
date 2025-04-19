int soundTriggerState = -1;
int lastSoundIndex = -1;
bool soundTriggerLatched = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 75;

/**
   Debounce self-resetting toggle switch and play a single sound.
*/
void check_sound_trigger()
{
  int buttonIndex = get_sound_index();

  if (buttonIndex != lastSoundIndex)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the buttonIndex is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (buttonIndex != soundTriggerState) {

      soundTriggerState = buttonIndex;

      if (!soundTriggerLatched && buttonIndex > -1)
      {
        soundTriggerLatched = true;

        int soundBankRaw = sbus_rx.data().ch[CH_SOUND_BANK];
        int soundBank = map(soundBankRaw, 172, 1811, 0, 2);
        int soundIndex = (soundBank * 10) + buttonIndex;
        // TODO: if the button index is top right, play rnadom from this bank
        myDFPlayer.play(soundIndex);
      }
      else if (soundTriggerLatched && buttonIndex == -1)
      {
        soundTriggerLatched = false;
      }
    }
  }

  lastSoundIndex = buttonIndex;
}

/**
   Map the 10 controller buttons to a different sound.
*/
int get_sound_index()
{
  int soundTriggerRaw = sbus_rx.data().ch[CH_SOUND_TRIGGER];


  if (soundTriggerRaw >= 1649 && soundTriggerRaw < 1680)
  {
    return 1;
  }
  else if (soundTriggerRaw >= 1520 && soundTriggerRaw < 1540)
  {
    return 2;
  }
  else if (soundTriggerRaw >= 1387 && soundTriggerRaw < 1407)
  {
    return 3;
  }
  else if (soundTriggerRaw >= 1254 && soundTriggerRaw < 1274)
  {
    return 4;
  }
  else if (soundTriggerRaw >= 1120 && soundTriggerRaw < 1140)
  {
    return 5;
  }
  else if (soundTriggerRaw >= 982 && soundTriggerRaw < 1002)
  {
    return 6;
  }
  else if (soundTriggerRaw >= 797 && soundTriggerRaw < 817)
  {
    return 7;
  }
  else if (soundTriggerRaw >= 600 && soundTriggerRaw < 620)
  {
    return 8;
  }
  else if (soundTriggerRaw >= 391 && soundTriggerRaw < 411)
  {
    return 9;
  }
  else if (soundTriggerRaw >= 162 && soundTriggerRaw < 182)
  {
    return 10;
  }
  else
  {
    return -1;
  }
}

/**
   Play a random sound from the selected sound bank switch.

   They are sorted into BB-8 noises and general "Star Wars" sounds.
*/
void play_sound()
{
  //  soundRaw = sbus_rx.data().ch[CH_SOUND_BANK];
  //  soundBank = map(soundRaw, 172, 1811, 0, 2);
  //  if (soundBank == 2)
  //  {
  //    // Short sounds
  //    myDFPlayer.play(random(13, 47));
  //  }
  //  else
  //  {
  //    // Long sounds
  //    myDFPlayer.play(random(0, 12));
  //  }
}
