#include "marker.h"

#include <math.h>
#include <ILI9488.h>
#include "angle_utils.h"
#include "app_state.h"
#include "ui.h"

void eraseOldBox() {
  if (!lastBoxValid) return;
  tft.fillRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH, ILI9488_BLACK);
  drawCrossInRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH);
  lastBoxValid = false;
}

static inline void fillRectIfPositive(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w > 0 && h > 0) tft.fillRect(x, y, w, h, color);
}

static inline int16_t min16(int16_t a, int16_t b) {
  return (a < b) ? a : b;
}

static inline int16_t max16(int16_t a, int16_t b) {
  return (a > b) ? a : b;
}

static bool updateFilledBoxDelta(int16_t nx, int16_t ny, int16_t nw, int16_t nh, uint16_t color) {
  if (!boxDeltaRenderEnabled) return false;
  if (!lastBoxValid || lastBoxHollow) return false;
  if (lastBoxColor != color) return false;

  int16_t ox = lastBoxX, oy = lastBoxY, ow = lastBoxW, oh = lastBoxH;
  int16_t ix = max16(ox, nx);
  int16_t iy = max16(oy, ny);
  int16_t ix2 = min16(ox + ow, nx + nw);
  int16_t iy2 = min16(oy + oh, ny + nh);

  if (ix >= ix2 || iy >= iy2) return false;

  // Erase parts that belonged only to the old filled box.
  fillRectIfPositive(ox, oy, ow, iy - oy, ILI9488_BLACK);
  fillRectIfPositive(ox, iy2, ow, (oy + oh) - iy2, ILI9488_BLACK);
  fillRectIfPositive(ox, iy, ix - ox, iy2 - iy, ILI9488_BLACK);
  fillRectIfPositive(ix2, iy, (ox + ow) - ix2, iy2 - iy, ILI9488_BLACK);

  // Fill only the new strips that are outside of the intersection.
  fillRectIfPositive(nx, ny, nw, iy - ny, color);
  fillRectIfPositive(nx, iy2, nw, (ny + nh) - iy2, color);
  fillRectIfPositive(nx, iy, ix - nx, iy2 - iy, color);
  fillRectIfPositive(ix2, iy, (nx + nw) - ix2, iy2 - iy, color);

  drawCrossInRect(ox, oy, ow, oh);
  drawCrossInRect(nx, ny, nw, nh);

  lastBoxX = nx; lastBoxY = ny; lastBoxW = nw; lastBoxH = nh;
  lastBoxColor = color;
  lastBoxHollow = false;
  lastBoxValid = true;
  return true;
}

static void getPixelsPerDegree(float &pxPerDegX, float &pxPerDegY) {
  static float cachedFovX = 0.0f;
  static float cachedFovY = 0.0f;
  static float cachedPxPerDegX = (float)SCREEN_W / DEFAULT_FOV_X_DEG;
  static float cachedPxPerDegY = (float)SCREEN_H / DEFAULT_FOV_Y_DEG;

  float fovX = (displayFovXDeg >= 5.0f) ? displayFovXDeg : DEFAULT_FOV_X_DEG;
  float fovY = (displayFovYDeg >= 5.0f) ? displayFovYDeg : DEFAULT_FOV_Y_DEG;

  if (fovX != cachedFovX) {
    cachedFovX = fovX;
    cachedPxPerDegX = (float)SCREEN_W / fovX;
  }
  if (fovY != cachedFovY) {
    cachedFovY = fovY;
    cachedPxPerDegY = (float)SCREEN_H / fovY;
  }

  pxPerDegX = cachedPxPerDegX;
  pxPerDegY = cachedPxPerDegY;
}

static void drawNewBox(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool hollow) {
  if (hollow) {
    tft.fillRect(x, y, w, h, ILI9488_BLACK);
    tft.drawRect(x, y, w, h, color);
  } else {
    tft.fillRect(x, y, w, h, color);
  }
  drawCrossInRect(x, y, w, h);

  lastBoxValid = true;
  lastBoxX = x; lastBoxY = y; lastBoxW = w; lastBoxH = h;
  lastBoxColor = color;
  lastBoxHollow = hollow;
}

void updateBox(int16_t pitchRelQ, int16_t yawRelQ, bool onTarget, bool tcpConnected) {
  float pxPerDegX, pxPerDegY;
  getPixelsPerDegree(pxPerDegX, pxPerDegY);

  int32_t centerX = (int32_t)lroundf((float)CX + (float)yawRelQ * pxPerDegX);
  int32_t centerY = (int32_t)lroundf((float)CY - (float)pitchRelQ * pxPerDegY);

  int32_t boxX = centerX - boxSizePx / 2;
  int32_t boxY = centerY - boxSizePx / 2;

  if (boxX > (int32_t)SCREEN_W - edgeVisiblePx) boxX = (int32_t)SCREEN_W - edgeVisiblePx;
  if (boxX < (int32_t)(-boxSizePx + edgeVisiblePx)) boxX = (int32_t)(-boxSizePx + edgeVisiblePx);

  if (boxY > (int32_t)SCREEN_H - edgeVisiblePx) boxY = (int32_t)SCREEN_H - edgeVisiblePx;
  if (boxY < (int32_t)(-boxSizePx + edgeVisiblePx)) boxY = (int32_t)(-boxSizePx + edgeVisiblePx);

  int16_t nx, ny, nw, nh;
  bool ok = clipRect(boxX, boxY, boxSizePx, boxSizePx, nx, ny, nw, nh);

  if (!ok) {
    eraseOldBox();
    return;
  }

  uint16_t color = onTarget ? ILI9488_GREEN : ILI9488_RED;
  bool hollow = !tcpConnected;

  if (lastBoxValid &&
      nx == lastBoxX && ny == lastBoxY && nw == lastBoxW && nh == lastBoxH) {

    if (color != lastBoxColor || hollow != lastBoxHollow) {
      if (hollow) {
        tft.fillRect(nx, ny, nw, nh, ILI9488_BLACK);
        tft.drawRect(nx, ny, nw, nh, color);
      } else {
        tft.fillRect(nx, ny, nw, nh, color);
      }
      drawCrossInRect(nx, ny, nw, nh);
      lastBoxColor = color;
      lastBoxHollow = hollow;
    }
    return;
  }

  if (!hollow && updateFilledBoxDelta(nx, ny, nw, nh, color)) {
    return;
  }

  eraseOldBox();
  drawNewBox(nx, ny, nw, nh, color, hollow);
}
