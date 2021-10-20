#include <Adafruit_NeoPixel.h>
#include "Commands.h"

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#define RFM69_CS 8
#define RFM69_INT 7
#define RFM69_RST 4

#define LED_PIN 11
#define LED_COUNT 6
#define BRIGHTNESS 255

// Individual LED addresses
#define LED_HP 0
#define LED_PSI 1
#define FRONT_LOGIC_1 2
#define FRONT_LOGIC_2 3
#define REAR_LOGIC_1 4
#define REAR_LOGIC_2 5

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init())
  {
    while (1)
      ;
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ))
  {
    while (1)
      ;
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  //  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
  //                  };

  uint8_t key[] = {0x03, 0x01, 0x08, 0x06, 0x07, 0x01, 0x05, 0x05,
                   0x03, 0x01, 0x08, 0x06, 0x07, 0x01, 0x05, 0x05
                  };

  rf69.setEncryptionKey(key);

  // Neopixel setup
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.show();

  // HP
  pixels.setPixelColor(LED_HP, pixels.Color(0, 0, 255));

  // PSI
  pixels.setPixelColor(LED_PSI, pixels.Color(0, 0, 0));

  // Front Logic
  pixels.setPixelColor(FRONT_LOGIC_1, pixels.Color(255, 255, 255));
  pixels.setPixelColor(FRONT_LOGIC_2, pixels.Color(255, 255, 255));

  // Read Logic
  pixels.setPixelColor(REAR_LOGIC_1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(REAR_LOGIC_2, pixels.Color(0, 0, 255));

  pixels.show();
}

void loop()
{
  if (rf69.available())
  {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {
      if (!len)
        return;
      buf[len] = 0;

      if (buf[0] == CMD_BATTERY_LOW)
      {
        enterLowBatteryMode();
      }
      else if (buf[0] == CMD_PSI_OFF)
      {
        psiOff();
      }
      else if (buf[0] == CMD_PSI_ON)
      {
        psiOn();
      }
    }
  }
}

/**
   Turn the front logic lights red to indicate low body battery
*/
void enterLowBatteryMode()
{
  pixels.setPixelColor(FRONT_LOGIC_1, pixels.Color(255, 0, 0));
  pixels.setPixelColor(FRONT_LOGIC_2, pixels.Color(255, 0, 0));
  pixels.show();
}

void psiOn()
{
  pixels.setPixelColor(LED_PSI, pixels.Color(255, 255, 255));
  pixels.show();
}

void psiOff()
{
  pixels.setPixelColor(LED_PSI, pixels.Color(0, 0, 0));
  pixels.show();
}
