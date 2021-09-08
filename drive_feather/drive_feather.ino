#include <Adafruit_NeoPixel.h>

// TODO: Neopixels
#define LED_PIN    6
#define LED_COUNT 60

// TODO: Feather radio

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  strip.begin();
  strip.show();
  strip.setBrightness(255);

  // TODO: set up colours
}

void loop()
{

}
