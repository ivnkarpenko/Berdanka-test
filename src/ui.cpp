#include "ui.h"

#include <Adafruit_GFX.h>
#include <ILI9488.h>
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
