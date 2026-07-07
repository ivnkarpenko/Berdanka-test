#include "ui.h"

#include <Adafruit_GFX.h>
#include <ILI9488.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "app_state.h"

void drawCrossFull() {
  tft.drawLine(CX, 0,      CX, SCREEN_H, ILI9488_DARKGREY);
  tft.drawLine(0,  CY, SCREEN_W, CY,     ILI9488_DARKGREY);
}

void drawCrossInRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  if (CX >= x && CX < (x + w)) {
    int16_t y0 = y;
    int16_t y1 = y + h - 1;
    if (y0 < 0) y0 = 0;
    if (y1 > SCREEN_H - 1) y1 = SCREEN_H - 1;
    tft.drawLine(CX, y0, CX, y1, ILI9488_DARKGREY);
  }
  if (CY >= y && CY < (y + h)) {
    int16_t x0 = x;
    int16_t x1 = x + w - 1;
    if (x0 < 0) x0 = 0;
    if (x1 > SCREEN_W - 1) x1 = SCREEN_W - 1;
    tft.drawLine(x0, CY, x1, CY, ILI9488_DARKGREY);
  }
}

void drawZeroCountdown(uint32_t remainingSec) {
  constexpr uint8_t textSize = 3;
  constexpr int16_t pad = 10;
  constexpr int16_t charW = 6 * textSize;
  constexpr int16_t charH = 8 * textSize;
  constexpr int16_t textW = charW * 2;
  constexpr int16_t x = SCREEN_W - pad - textW;
  constexpr int16_t y = pad;

  char buf[3];
  snprintf(buf, sizeof(buf), "%2lu", (unsigned long)remainingSec);

  tft.fillRect(x - 4, y - 4, textW + 8, charH + 8, ILI9488_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(ILI9488_YELLOW, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);
}

static void drawStatusLineIfChanged(int16_t x, int16_t y, const char* s,
                                    uint16_t color, char* last, size_t lastSz,
                                    uint16_t &lastColor, int padWidth) {
  if (strncmp(s, last, lastSz) == 0 && color == lastColor) return;

  char buf[64];
  snprintf(buf, sizeof(buf), "%-*s", padWidth, s);

  tft.setTextSize(STATUS_TEXT_SIZE);
  tft.setTextColor(color, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);

  strncpy(last, s, lastSz - 1);
  last[lastSz - 1] = '\0';
  lastColor = color;
}

void drawIMUStatus(const char* text, uint16_t color) {
  drawStatusLineIfChanged(STATUS_X, STATUS_Y_IMU, text,
                          color, lastImuStatusLine, sizeof(lastImuStatusLine),
                          lastImuStatusColor, 28);
}

void drawNetStatus(bool tcpConnected, bool udpConnected) {
  char line[40];
  if (tcpConnected) {
    snprintf(line, sizeof(line), "TCP OK");
  } else if (udpConnected) {
    snprintf(line, sizeof(line), "UDP OK");
  } else {
    snprintf(line, sizeof(line), "NET WAIT");
  }
  uint16_t color = (tcpConnected || udpConnected) ? ILI9488_GREEN : ILI9488_YELLOW;

  drawStatusLineIfChanged(STATUS_X, STATUS_Y_TCP, line,
                          color, lastTcpStatusLine, sizeof(lastTcpStatusLine),
                          lastTcpStatusColor, 28);
}

static float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static void drawAngleBar(int16_t x, const char* label, float valueDeg,
                         float rangeDeg, uint16_t color, bool force) {
  constexpr int16_t barW = 42;
  constexpr int16_t barTop = 72;
  constexpr int16_t barH = 220;
  constexpr int16_t midY = barTop + barH / 2;
  constexpr int16_t labelY = 50;
  constexpr int16_t labelW = 72;
  constexpr int16_t innerXPad = 3;
  constexpr int16_t innerW = barW - innerXPad * 2;

  static int16_t lastAz = INT16_MIN;
  static int16_t lastEl = INT16_MIN;
  static int16_t lastCant = INT16_MIN;

  int16_t* last = &lastAz;
  if (label[0] == 'E') last = &lastEl;
  else if (label[0] == 'C') last = &lastCant;

  float normalized = clampFloat(valueDeg / rangeDeg, -1.0f, 1.0f);
  int16_t halfH = barH / 2 - 3;
  int16_t fillPx = (int16_t)lroundf(normalized * (float)halfH);

  if (!force && fillPx == *last) return;

  if (force || *last == INT16_MIN) {
    tft.fillRect(x, labelY, labelW, 14, ILI9488_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
    tft.setCursor(x + 4, labelY);
    tft.print(label);

    tft.fillRect(x + 10, barTop, barW, barH, ILI9488_BLACK);
    tft.drawRect(x + 10, barTop, barW, barH, ILI9488_DARKGREY);
    tft.drawLine(x + 8, midY, x + 10 + barW + 2, midY, ILI9488_WHITE);
  } else {
    if (*last > 0) {
      tft.fillRect(x + 10 + innerXPad, midY - *last, innerW, *last, ILI9488_BLACK);
    } else if (*last < 0) {
      tft.fillRect(x + 10 + innerXPad, midY, innerW, -*last, ILI9488_BLACK);
    } else {
      tft.drawLine(x + 10 + innerXPad, midY, x + 10 + innerXPad + innerW - 1, midY, ILI9488_BLACK);
    }
    tft.drawLine(x + 8, midY, x + 10 + barW + 2, midY, ILI9488_WHITE);
  }

  if (fillPx > 0) {
    tft.fillRect(x + 10 + innerXPad, midY - fillPx, innerW, fillPx, color);
  } else if (fillPx < 0) {
    tft.fillRect(x + 10 + innerXPad, midY, innerW, -fillPx, color);
  } else {
    tft.drawLine(x + 10 + innerXPad, midY, x + 10 + innerXPad + innerW - 1, midY, color);
  }

  *last = fillPx;
}

void drawOrientationBars(float yawDeg, float pitchDeg, float rollDeg, bool force) {
  if (force) {
    tft.fillScreen(ILI9488_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ILI9488_YELLOW, ILI9488_BLACK);
    tft.setCursor(8, 24);
    tft.print("QUAT VECTOR REL");
    tft.drawLine(0, 68, SCREEN_W - 1, 68, ILI9488_DARKGREY);
    tft.drawLine(160, 68, 160, SCREEN_H - 1, ILI9488_DARKGREY);
    tft.drawLine(320, 68, 320, SCREEN_H - 1, ILI9488_DARKGREY);
  }

  drawAngleBar(40, "AZ", yawDeg, 90.0f, ILI9488_CYAN, force);
  drawAngleBar(200, "EL", pitchDeg, 90.0f, ILI9488_GREEN, force);
  drawAngleBar(360, "CANT", rollDeg, 180.0f, ILI9488_MAGENTA, force);
}
