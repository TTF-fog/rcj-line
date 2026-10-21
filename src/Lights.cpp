#include "Lights.h"

#include <Adafruit_NeoPixel.h>

#include "BoardPins.h"
#include "Claw.h"
#include "Imu.h"
#include "Motion.h"

namespace {

Adafruit_NeoPixel status(STATUS_PIXEL_COUNT, STATUS_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bar(LIGHT_BAR_COUNT, LIGHT_BAR_PIN, NEO_GRB + NEO_KHZ800);

unsigned long estopUntilMs = 0;
unsigned long lastPaintMs = 0;
uint32_t shownColor = 1;  // impossible value so the first update always paints

uint32_t statusColor() {
    unsigned long now = millis();
    if (now < estopUntilMs) return Adafruit_NeoPixel::Color(255, 0, 0);
    if (!imu::ok()) return (now / 400) % 2 ? Adafruit_NeoPixel::Color(255, 0, 0) : 0;
    if (claw::busy()) return Adafruit_NeoPixel::Color(160, 0, 255);
    switch (motion::mode()) {
        case motion::Mode::Turn: return Adafruit_NeoPixel::Color(255, 150, 0);
        case motion::Mode::Straight: return Adafruit_NeoPixel::Color(0, 200, 255);
        case motion::Mode::Drive: return Adafruit_NeoPixel::Color(0, 0, 255);
        default: return Adafruit_NeoPixel::Color(0, 40, 0);
    }
}

}  // namespace

namespace lights {

void begin() {
    status.begin();
    bar.begin();
    setBar(255, 255, 255);
    update();
}

void setBar(uint8_t r, uint8_t g, uint8_t b) {
    bar.fill(Adafruit_NeoPixel::Color(r, g, b));
    bar.show();
}

void flashEstop() { estopUntilMs = millis() + 1500; }

void update() {
    if (millis() - lastPaintMs < 50) return;
    lastPaintMs = millis();
    uint32_t color = statusColor();
    if (color == shownColor) return;
    shownColor = color;
    status.fill(color);
    status.show();
}

}  // namespace lights
