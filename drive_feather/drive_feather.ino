#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <RH_RF69.h>

#include "Commands.h"

#define BUSY_SPK A0
#define BUSY_PIN A1

#define NEOPIXEL_PIN 11
#define NEOPIXEL_COUNT 20
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


Adafruit_NeoPixel body = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

unsigned long panel1Millis;
int panel1State = 1;
int r1 = 0;
int b1 = 0;

unsigned long panel2Millis;
int panel2State = 1;
int b21;
int b22;
int b23;
int b24;

unsigned long panel3Millis;
int panel3State = 1;
int r31 = 255;
int b31;
int r32;
int b32 = 255;

unsigned long panel4Millis;
int r41;
int b41 = 255;
int r42 = 255;
int b42;

int b5State = 1;
int b5;

int panel6State = 1;
int r61 = 0;
int b61 = 255;
int r62 = 255;
int b62 = 0;

unsigned long lastMillis;

//int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup()
{
  //  Serial.begin(115200);
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

  body.begin();
  body.show();

  turnPSIOff();
}

void loop()
{
  if ((millis() - lastMillis) > 50)
  {
    panel1();
    panel2();
    panel3();
    panel4();
    panel5();
    panel6();

    body.show();

    lastMillis = millis();
  }

  // Speaker light
  int sound = analogRead(A0);
  if (sound > 800)
  {
    turnPSIOn();
  }
  else if (sound < 790)
  {
    turnPSIOff();
  }
  else
  {
    turnPSIOff();
  }
}

void turnPSIOn()
{
  uint8_t data[] = "O";
  rf69.send((uint8_t *)data, strlen(data));
}

void turnPSIOff()
{
  uint8_t data[] = "F";
  rf69.send((uint8_t *)data, strlen(data));
}

void panel1()
{
  if (panel1State == 1 && b1 <= 255)
  {
    r1 ++;
    b1 += 8;

    if (b1 >= 240) {
      panel1State = 2;
      panel1Millis = millis();
    }

  } else if (panel1State == 2 && (millis() - panel1Millis >= 4500)) {

    r1 --;
    b1 -= 8;

    if (b1 <= 20 || r1 <= 1) {
      panel1State = 1;
    }

  }
  constrain(r1, 0, 30);
  constrain(b1, 0, 255);

  body.setPixelColor(0, body.Color(r1, 0, b1));
  body.setPixelColor(1, body.Color(r1, 0, b1));

}


void panel2() {

  if (panel2State == 1) {
    if (b21 <= 250) {
      b21 += 5;
      constrain(b21, 0, 255);
    }

    if (b21 >= 80 && b22 <= 250) {
      b22 += 5;
      constrain(b22, 0, 255);
    }

    if (b22 >= 80 && b23 <= 250) {
      b23 += 5;
      constrain(b23, 0, 255);
    }

    if (b23 >= 80 && b24 <= 250) {
      b24 += 5;
      constrain(b24, 0, 255);
      if (b24 >= 245) {

        panel2Millis = millis();
        panel2State = 2;
      }
    }
  } else if (millis() - panel2Millis >= 6000) {
    if (b21 >= 5) {
      b21 -= 5;
      constrain(b21, 0, 255);
    }

    if (b21 <= 180 && b22 >= 5) {
      b22 -= 5;
      constrain(b22, 0, 255);
    }

    if (b22 <= 180 && b23 >= 5) {
      b23 -= 5;
      constrain(b23, 0, 255);
    }

    if (b23 <= 180 && b24 >= 5) {
      b24 -= 5;
      constrain(b24, 0, 255);
    } else if (b24 <= 5) {
      panel2State = 1;
    }
  }



  body.setPixelColor(2, body.Color(0, 0, b21));
  body.setPixelColor(3, body.Color(0, 0, b22));
  body.setPixelColor(4, body.Color(0, 0, b23));
  body.setPixelColor(5, body.Color(0, 0, b24));


}




void panel3() {

  if (panel3State == 1 && (millis() - panel3Millis >= 7500)) {
    r31 -= 15;
    b31 += 15;
    r32 += 15;
    b32 -= 15;

    if (b31 == 255) {
      panel3Millis = millis();
      panel3State = 2;
    }
  } else if (panel3State == 2 && (millis() - panel3Millis >= 3750)) {
    r32 -= 15;
    b32 += 15;
    r31 += 15;
    b31 -= 15;

    if (b31 == 0) {
      panel3Millis = millis();
      panel3State = 1;
    }
  }



  constrain(r31, 0, 255);
  constrain(r32, 0, 255);
  constrain(b31, 0, 255);
  constrain(b32, 0, 255);

  body.setPixelColor(10, body.Color(r31, 0, b31));
  body.setPixelColor(7, body.Color(r31, 0, b31));
  body.setPixelColor(8, body.Color(r31, 0, b31));
  body.setPixelColor(9, body.Color(r31, 0, b31));
  body.setPixelColor(6, body.Color(r32, 0, b32));


}




void panel4() {

  if (millis() - panel4Millis >= 10000) {

    if (r41 == 155) {
      r41 = 0;
      b41 = 155;
      r42 = 155;
      b42 = 0;
      panel4Millis = millis();
    } else if (r41 == 0) {
      r41 = 155;
      b41 = 0;
      r42 = 0;
      b42 = 155;
      panel4Millis = millis();

    }

  }

  body.setPixelColor(11, body.Color(r41, 0, b41));
  body.setPixelColor(12, body.Color(r42, 0, b42));
  body.setPixelColor(13, body.Color(r41, 0, b41));
  body.setPixelColor(14, body.Color(100, 100, 100));
  body.setPixelColor(15, body.Color(r42, 0, b42));
  body.setPixelColor(16, body.Color(r42, 0, b42));
}


void panel5() {

  if (b5State == 1) {
    b5++;
    if (b5 >= 200) {
      b5State = 2;
    }
  } else if (b5State == 2) {
    b5--;
    if (b5 <= 20) {
      b5State = 1;
    }
  }
  body.setPixelColor(17, body.Color(0, 0, b5));


}

void panel6() {

  if (panel6State == 1) {

    r61 += 2;
    b61 -= 2;
    r62 -= 2;
    b62 += 2;

    constrain(r61, 0, 255);
    constrain(r62, 0, 255);
    constrain(b61, 0, 255);
    constrain(b62, 0, 255);

    body.setPixelColor(18, body.Color(r61, 0, b61));
    body.setPixelColor(19, body.Color(r62, 0, b62));

    if (r61 >= 250) {
      panel6State = 2;
    }

  } else if (panel6State == 2) {

    r62 += 2;
    b62 -= 2;
    r61 -= 2;
    b61 += 2;
    constrain(r61, 0, 255);
    constrain(r62, 0, 255);
    constrain(b61, 0, 255);
    constrain(b62, 0, 255);

    body.setPixelColor(18, body.Color(r61, 0, b61));
    body.setPixelColor(19, body.Color(r62, 0, b62));

    if (r62 >= 250) {
      panel6State = 1;
    }

  }

}
