#include <FastLED.h>

#define LED_PIN     31
#define NUM_LEDS    30
#define BRIGHTNESS  50
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);  // Limit to 5V, 500mA
}

void rainbowCycle() {
  static uint8_t hue = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue + (i * 10), 255, 255);
  }
  FastLED.show();
  hue++;
  delay(20);
}

void theaterChase(CRGB color, int delayTime) {
  for (int a = 0; a < 10; a++) {
    for (int b = 0; b < 3; b++) {
      for (int c = 0; c < NUM_LEDS; c++) {
        if ((c + b) % 3 == 0) {
          leds[c] = color;
        } else {
          leds[c] = CRGB::Black;
        }
      }
      FastLED.show();
      delay(delayTime);
    }
  }
}

void fireFlicker() {
  for (int i = 0; i < NUM_LEDS; i++) {
    int flicker = random(128, 255);
    leds[i] = CRGB(flicker, flicker / 4, 0);
  }
  FastLED.show();
  delay(50);
}


void rainbowBlendCycle() {
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1;
  fill_rainbow(leds, NUM_LEDS, startIndex, 7);
  FastLED.show();
  delay(20);
}

void colorWipe(CRGB color, int delayTime) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
    FastLED.show();
    delay(delayTime);
  }
}



int mode = 0;


void loop() {
  // fill_solid(leds, NUM_LEDS, CRGB::Purple);
  // FastLED.show();
  // delay(500);

  // fill_solid(leds, NUM_LEDS, CRGB::Gray);
  // FastLED.show();
  // delay(500);

  switch (mode) {
    case 0: rainbowCycle(); break;
    case 1: colorWipe(CRGB::Red, 20); break;
    case 2: fireFlicker(); break;
    case 3: theaterChase(CRGB::White, 50); break;
    case 4: rainbowBlendCycle(); break;
} 

 // Change mode every 10 seconds
  static unsigned long lastChange = 0;
  if (millis() - lastChange > 10000) {
    mode = (mode + 1) % 5;
    lastChange = millis();
  }
}
