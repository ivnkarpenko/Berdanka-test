#pragma once

#include <Arduino.h>

void drawCrossFull();
void drawCrossInRect(int16_t x, int16_t y, int16_t w, int16_t h);
void drawZeroCountdown(uint32_t remainingSec);
void drawIMUStatus(const char* text, uint16_t color);
void drawNetStatus(bool tcpConnected, bool udpConnected);
