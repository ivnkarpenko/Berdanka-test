#pragma once

#include <Arduino.h>
#include <math.h>
#include "config.h"

static inline int16_t quantizeDeg2(float a) {
  float q = (float)ANGLE_STEP_DEG * roundf(a / (float)ANGLE_STEP_DEG);
  return (int16_t)q;
}

static inline float wrapAngle180f(float a) {
  while (a <= -180.0f) a += 360.0f;
  while (a >   180.0f) a -= 360.0f;
  return a;
}

static inline int16_t wrapAngle180i(int16_t a) {
  int32_t v = (int32_t)a;
  while (v <= -180) v += 360;
  while (v >   180) v -= 360;
  return (int16_t)v;
}

static inline float deltaAngle180f(float current, float target) {
  return wrapAngle180f(current - target);
}

static inline int16_t deltaAngle180i(int16_t current, int16_t target) {
  int32_t d = (int32_t)current - (int32_t)target;
  while (d <= -180) d += 360;
  while (d >   180) d -= 360;
  return (int16_t)d;
}

static inline bool clipRect(int32_t x, int32_t y, int32_t w, int32_t h,
                            int16_t &ox, int16_t &oy, int16_t &ow, int16_t &oh) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;

  if (w <= 0 || h <= 0) return false;

  ox = (int16_t)x; oy = (int16_t)y; ow = (int16_t)w; oh = (int16_t)h;
  return true;
}
