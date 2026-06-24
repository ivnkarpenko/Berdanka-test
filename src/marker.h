#pragma once

#include <Arduino.h>

void eraseOldBox();
void updateBox(int16_t pitchRelQ, int16_t yawRelQ, bool onTarget, bool tcpConnected);
