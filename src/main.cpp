#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <ILI9488.h>
#include "ICM_20948.h"
#include <WiFiS3.h>

// ================== WIFI ==================
// Previous network
// const char* WIFI_SSID = "JetsonAP";
// const char* WIFI_PASS = "12345678";
// const char* WIFI_SSID = "GABELLA";
// const char* WIFI_PASS = "J8f2829a";
// Current network (AP)
const char* WIFI_SSID = "cisco";
const char* WIFI_PASS = "cisco1234"; // 8+ chars required for WPA2
constexpr uint16_t SERVER_PORT = 3333;

WiFiServer server(SERVER_PORT);
WiFiClient client;

// ================== DISPLAY ==================
constexpr uint8_t TFT_CS   = 10;
constexpr uint8_t TFT_DC   = 9;
constexpr uint8_t TFT_RST  = 8;
constexpr uint8_t TFT_LED  = A0;

constexpr uint8_t TFT_ROTATION = 1;   // 480x320 landscape
constexpr int16_t SCREEN_W = 480;
constexpr int16_t SCREEN_H = 320;
constexpr int16_t CX = SCREEN_W / 2;
constexpr int16_t CY = SCREEN_H / 2;

ILI9488 tft(TFT_CS, TFT_DC, TFT_RST);

// ================== UI ==================
constexpr float   DEFAULT_FOV_X_DEG = 60.0f;
constexpr float   DEFAULT_FOV_Y_DEG = 80.0f;
constexpr int16_t DEFAULT_BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

constexpr uint8_t STATUS_TEXT_SIZE = 2;

// Display status positions
constexpr int16_t STATUS_X      = 10;
constexpr int16_t STATUS_Y_IMU  = 10;
constexpr int16_t STATUS_Y_TCP  = 36;

constexpr uint32_t TCP_STATUS_REFRESH_MS = 200;
constexpr uint32_t DEFAULT_TCP_POLL_MS = 80;
constexpr uint32_t TCP_ALIVE_TIMEOUT_MS = 10000;
constexpr uint32_t DEFAULT_BOX_REFRESH_MS = 33;     // ~30 FPS box redraw cap
constexpr uint32_t SERIAL_DEBUG_MS = 2000;  // reduce serial overhead
constexpr bool ENABLE_SERIAL_DEBUG = true;
constexpr bool ENABLE_PACKET_ACK = false;

// edge marker visible thickness (px)
constexpr int16_t DEFAULT_EDGE_VISIBLE_PX = 5;

uint32_t lastTcpStatusDrawMs = 0;
uint32_t lastTcpPollMs = 0;
uint32_t lastBoxDrawMs = 0;
uint32_t lastSerialDebugMs = 0;
uint32_t lastLoopTickMs = 0;
bool     lastTcpConnectedState = false;
bool     tcpConnectedCached = false;
uint32_t lastTcpAliveMs = 0;

char lastImuStatusLine[40] = "";
char lastTcpStatusLine[40] = "";
uint16_t lastImuStatusColor = 0xFFFF;
uint16_t lastTcpStatusColor = 0xFFFF;
float    displayFovXDeg = DEFAULT_FOV_X_DEG;
float    displayFovYDeg = DEFAULT_FOV_Y_DEG;
uint32_t tcpPollIntervalMs = DEFAULT_TCP_POLL_MS;
int16_t  boxSizePx = DEFAULT_BOX_SIZE;
uint32_t boxRefreshIntervalMs = DEFAULT_BOX_REFRESH_MS;
int16_t  edgeVisiblePx = DEFAULT_EDGE_VISIBLE_PX;
bool     boxRenderEnabled = true;
bool     boxDeltaRenderEnabled = true;

// ================== TARGET (from TCP client) ==================
int16_t spawnPitchQ = 0;
int16_t spawnYawQ   = 0;
bool    spawnSet    = false;

// ================== IMU ==================
ICM_20948_I2C imu;

// zero calibration
bool  zeroSet    = false;
float zeroRoll   = 0.0f;
float zeroPitch  = 0.0f;
float zeroYaw    = 0.0f;
uint32_t zeroArmMs = 0;
uint32_t lastZeroCountdownDrawMs = 0;

constexpr uint32_t ZERO_DELAY_MS = 10000;
constexpr uint32_t ZERO_COUNTDOWN_REFRESH_MS = 200;

// Anti-drift yaw
float lastYaw          = 0.0f;
unsigned long lastMove = 0;
const float yawDriftLimitDeg   = 0.05f;
const unsigned long yawHoldTime = 250;

// Freeze yaw near gimbal lock
float yawStable      = 0.0f;
bool  yawStableInit  = false;
const float pitchLockThreshold = 80.0f;

// ================== helpers ==================
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

// ---------- safe clip helpers ----------
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

// ================== Cross / status ==================
void drawCrossFull() {
  tft.drawLine(CX, 0,      CX, SCREEN_H, ILI9488_DARKGREY);
  tft.drawLine(0,  CY, SCREEN_W, CY,     ILI9488_DARKGREY);
}

// крест только внутри прямоугольника
void drawCrossInRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  // vertical x=CX
  if (CX >= x && CX < (x + w)) {
    int16_t y0 = y;
    int16_t y1 = y + h - 1;
    if (y0 < 0) y0 = 0;
    if (y1 > SCREEN_H - 1) y1 = SCREEN_H - 1;
    tft.drawLine(CX, y0, CX, y1, ILI9488_DARKGREY);
  }
  // horizontal y=CY
  if (CY >= y && CY < (y + h)) {
    int16_t x0 = x;
    int16_t x1 = x + w - 1;
    if (x0 < 0) x0 = 0;
    if (x1 > SCREEN_W - 1) x1 = SCREEN_W - 1;
    tft.drawLine(x0, CY, x1, CY, ILI9488_DARKGREY);
  }
}

void drawStatusLineIfChanged(int16_t x, int16_t y, const char* s,
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

void drawTCPStatus(bool tcpConnected) {
  char line[40];
  if (tcpConnected) {
    snprintf(line, sizeof(line), "TCP OK");
  } else {
    snprintf(line, sizeof(line), "TCP WAIT");
  }
  uint16_t color = tcpConnected ? ILI9488_GREEN : ILI9488_YELLOW;

  drawStatusLineIfChanged(STATUS_X, STATUS_Y_TCP, line,
                          color, lastTcpStatusLine, sizeof(lastTcpStatusLine),
                          lastTcpStatusColor, 28);
}

// ================== IMU init/read ==================
bool startIMU() {
  imu.begin(Wire, 0);
  if (imu.status != ICM_20948_Stat_Ok) return false;

  if (imu.initializeDMP() != ICM_20948_Stat_Ok) return false;
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) return false;

  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok) return false;
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) return false;
  if (imu.enableDMP()  != ICM_20948_Stat_Ok) return false;
  if (imu.resetDMP()   != ICM_20948_Stat_Ok) return false;
  if (imu.resetFIFO()  != ICM_20948_Stat_Ok) return false;

  return true;
}

void applyZeroCalibration(float roll, float pitch, float yaw, uint32_t nowMs) {
  zeroRoll = roll;
  zeroPitch = pitch;
  zeroYaw = yaw;
  zeroSet = true;

  // Rebase the local target to the new zero reference so the marker
  // does not jump away from center immediately after calibration.
  spawnPitchQ = 0;
  spawnYawQ = 0;
  spawnSet = false;

  lastYaw = 0.0f;
  lastMove = nowMs;
  yawStable = 0.0f;
  yawStableInit = true;

  drawIMUStatus("IMU OK", ILI9488_GREEN);
  Serial.println("Zero calibration applied.");
}

void updateZeroCountdown(uint32_t nowMs) {
  if (zeroSet) return;
  if (nowMs - lastZeroCountdownDrawMs < ZERO_COUNTDOWN_REFRESH_MS) return;

  uint32_t elapsed = nowMs - zeroArmMs;
  uint32_t remainingMs = (elapsed >= ZERO_DELAY_MS) ? 0 : (ZERO_DELAY_MS - elapsed);
  uint32_t remainingSec = (remainingMs + 999U) / 1000U;

  char line[40];
  snprintf(line, sizeof(line), "ZERO IN %lus", (unsigned long)remainingSec);
  drawIMUStatus(line, ILI9488_YELLOW);
  lastZeroCountdownDrawMs = nowMs;
}

bool readAnglesOnce(float &outRoll, float &outPitch, float &outYaw) {
  icm_20948_DMP_data_t d;
  ICM_20948_Status_e s = imu.readDMPdataFromFIFO(&d);

  if (s == ICM_20948_Stat_FIFONoDataAvail) return false;

  if (s != ICM_20948_Stat_Ok && s != ICM_20948_Stat_FIFOMoreDataAvail) {
    imu.resetFIFO();
    imu.resetDMP();
    return false;
  }

  if (!(d.header & DMP_header_bitmap_Quat6)) return false;

  float q1 = (float)d.Quat6.Data.Q1 / 1073741824.0f;
  float q2 = (float)d.Quat6.Data.Q2 / 1073741824.0f;
  float q3 = (float)d.Quat6.Data.Q3 / 1073741824.0f;

  float sum = q1*q1 + q2*q2 + q3*q3;
  if (sum > 1.0f) sum = 1.0f;
  if (sum < 0.0f) sum = 0.0f;
  float q0 = sqrtf(1.0f - sum);

  float roll  = atan2f(2.0f * (q0*q1 + q2*q3),
                       1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / PI;

  float pitch = asinf (2.0f * (q0*q2 - q1*q3)) * 180.0f / PI;

  float yaw   = atan2f(2.0f * (q0*q3 + q1*q2),
                       1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / PI;

  if (!zeroSet) {
    outRoll = roll;
    outPitch = pitch;
    outYaw = wrapAngle180f(-yaw);
    return true;
  }

  roll  -= zeroRoll;
  pitch -= zeroPitch;
  yaw    = zeroYaw - yaw;
  yaw    = wrapAngle180f(yaw);

  if (fabsf(deltaAngle180f(yaw, lastYaw)) > yawDriftLimitDeg) {
    lastMove = millis();
  } else {
    if (millis() - lastMove > yawHoldTime) yaw = lastYaw;
  }
  lastYaw = wrapAngle180f(yaw);

  if (!yawStableInit) {
    yawStable = yaw;
    yawStableInit = true;
  }
  if (fabs(pitch) < pitchLockThreshold) yawStable = yaw;

  outRoll  = roll;
  outPitch = pitch;
  outYaw   = wrapAngle180f(yawStable);
  return true;
}

// ================== NET parsing ==================
static String readLine(WiFiClient& c) {
  String line;
  while (c.available()) {
    char ch = (char)c.read();
    if (ch == '\r') continue;
    if (ch == '\n') break;
    line += ch;
    if (line.length() > 256) break;
  }
  return line;
}

static bool isCenterCommand(const String& s) {
  return s == "CMD:CENTER";
}

static bool isPingCommand(const String& s) {
  return s == "PING";
}

static bool parseMsgOnlyCommand(const String& s, String& msg) {
  const String prefix = "MSGONLY:";
  if (!s.startsWith(prefix)) return false;

  msg = s.substring(prefix.length());
  msg.trim();
  return msg.length() > 0;
}

static bool parsePositiveFloatTail(String tail, float& value) {
  tail.trim();
  if (tail.length() == 0) return false;

  bool hasDigit = false;
  bool hasDot = false;
  for (size_t i = 0; i < tail.length(); i++) {
    char c = tail[i];
    if (isDigit(c)) {
      hasDigit = true;
      continue;
    }
    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }
    return false;
  }

  if (!hasDigit) return false;
  value = tail.toFloat();
  return value > 0.0f;
}

static bool parseDisplayFovXCommand(const String& s, float& fovXDeg) {
  const String prefix = "CFG:FOV_X:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;
  if (value < 5.0f || value > 180.0f) return false;

  fovXDeg = value;
  return true;
}

static bool parseDisplayFovYCommand(const String& s, float& fovYDeg) {
  const String prefix = "CFG:FOV_Y:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;
  if (value < 5.0f || value > 180.0f) return false;

  fovYDeg = value;
  return true;
}

static bool parseDisplayDegPerPxCommand(const String& s, float& degPerPx) {
  const String prefix = "CFG:DEG_PER_PX:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;

  degPerPx = value;
  return true;
}

static bool parseTcpPollMsCommand(const String& s, uint32_t& tcpPollMs) {
  const String prefix = "CFG:TCP_POLL_MS:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;

  for (size_t i = 0; i < tail.length(); i++) {
    char c = tail[i];
    if (!isDigit(c)) return false;
  }

  uint32_t value = (uint32_t)tail.toInt();
  if (value < 20 || value > 2000) return false;
  tcpPollMs = value;
  return true;
}

static bool parseBoxSizeCommand(const String& s, int16_t& sizePx) {
  const String prefix = "CFG:BOX_SIZE:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;
  for (size_t i = 0; i < tail.length(); i++) {
    if (!isDigit(tail[i])) return false;
  }

  int value = tail.toInt();
  if (value < 8 || value > 200) return false;
  sizePx = (int16_t)value;
  return true;
}

static bool parseBoxRefreshMsCommand(const String& s, uint32_t& refreshMs) {
  const String prefix = "CFG:BOX_REFRESH_MS:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;
  for (size_t i = 0; i < tail.length(); i++) {
    if (!isDigit(tail[i])) return false;
  }

  uint32_t value = (uint32_t)tail.toInt();
  if (value < 10 || value > 1000) return false;
  refreshMs = value;
  return true;
}

static bool parseBoxRenderCommand(const String& s, bool& enabled) {
  const String prefix = "CFG:BOX_RENDER:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail == "1") {
    enabled = true;
    return true;
  }
  if (tail == "0") {
    enabled = false;
    return true;
  }
  return false;
}

static bool parseBoxDeltaRenderCommand(const String& s, bool& enabled) {
  const String prefix = "CFG:BOX_DELTA_RENDER:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail == "1") {
    enabled = true;
    return true;
  }
  if (tail == "0") {
    enabled = false;
    return true;
  }
  return false;
}

static bool parsePacket(const String& s, String& msg, float& x, float& y) {
  int iMsg = s.indexOf("MSG:");
  int iX   = s.indexOf(";X:");
  int iY   = s.indexOf(";Y:");
  if (iMsg != 0 || iX < 0 || iY < 0) return false;

  msg = s.substring(4, iX);
  String sx = s.substring(iX + 3, iY);
  String sy = s.substring(iY + 3);

  sx.trim(); sy.trim();
  if (sx.length() == 0 || sy.length() == 0) return false;

  for (size_t i = 0; i < sx.length(); i++) {
    char c = sx[i];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.')) return false;
  }
  for (size_t i = 0; i < sy.length(); i++) {
    char c = sy[i];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.')) return false;
  }

  x = sx.toFloat();
  y = sy.toFloat();
  return true;
}

void wifiConnectAndStartServer() {
  Serial.print("Starting AP: ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;
  while (status != WL_AP_LISTENING && status != WL_AP_CONNECTED) {
    status = WiFi.beginAP(WIFI_SSID, WIFI_PASS);
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("AP started.");

  server.begin();
  Serial.print("TCP server started on port ");
  Serial.println(SERVER_PORT);
}

// ================== BOX (single, safe, edge visible) ==================
bool     lastBoxValid = false;
int16_t  lastBoxX = 0, lastBoxY = 0, lastBoxW = 0, lastBoxH = 0;
uint16_t lastBoxColor = 0xFFFF;
bool     lastBoxHollow = false;

void eraseOldBox() {
  if (!lastBoxValid) return;
  tft.fillRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH, ILI9488_BLACK);
  drawCrossInRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH);
  lastBoxValid = false;
}

static inline void fillRectIfPositive(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w > 0 && h > 0) tft.fillRect(x, y, w, h, color);
}

bool updateFilledBoxDelta(int16_t nx, int16_t ny, int16_t nw, int16_t nh, uint16_t color) {
  if (!boxDeltaRenderEnabled) return false;
  if (!lastBoxValid || lastBoxHollow) return false;
  if (lastBoxColor != color) return false;

  int16_t ox = lastBoxX, oy = lastBoxY, ow = lastBoxW, oh = lastBoxH;
  int16_t ix = max(ox, nx);
  int16_t iy = max(oy, ny);
  int16_t ix2 = min<int16_t>(ox + ow, nx + nw);
  int16_t iy2 = min<int16_t>(oy + oh, ny + nh);

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

void drawNewBox(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool hollow) {
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
  float fovX = (displayFovXDeg >= 5.0f) ? displayFovXDeg : DEFAULT_FOV_X_DEG;
  float fovY = (displayFovYDeg >= 5.0f) ? displayFovYDeg : DEFAULT_FOV_Y_DEG;
  float pxPerDegX = (float)SCREEN_W / fovX;
  float pxPerDegY = (float)SCREEN_H / fovY;

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
    } else {
      drawCrossInRect(nx, ny, nw, nh);
    }
    return;
  }

  if (!hollow && updateFilledBoxDelta(nx, ny, nw, nh, color)) {
    return;
  }

  eraseOldBox();
  drawNewBox(nx, ny, nw, nh, color, hollow);
}

// ================== Arduino ==================
void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(TFT_ROTATION);

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.fillScreen(ILI9488_BLACK);
  drawCrossFull();
  drawTCPStatus(false);
  Serial.println("Target persistence disabled.");

  Wire.begin();
  Wire.setClock(400000);

  drawIMUStatus("IMU INIT...", ILI9488_YELLOW);
  Serial.println("IMU init started.");

  while (!startIMU()) {
    drawIMUStatus("IMU INIT FAIL", ILI9488_RED);
    Serial.println("IMU init fail, retry...");
    delay(200);
  }

  drawIMUStatus("IMU OK", ILI9488_GREEN);
  Serial.println("IMU OK.");
  zeroArmMs = millis();
  lastZeroCountdownDrawMs = 0;
  drawIMUStatus("ZERO IN 10s", ILI9488_YELLOW);
  delay(150);

  wifiConnectAndStartServer();
  drawTCPStatus(false);
  Serial.println("TCP status: WAIT.");
}

void loop() {
  uint32_t loopStartUs = micros();
  uint32_t nowMs = millis();
  uint32_t loopDtMs = (lastLoopTickMs == 0) ? 0 : (nowMs - lastLoopTickMs);
  lastLoopTickMs = nowMs;
  uint32_t netPollUs = 0;
  uint32_t netReadUs = 0;
  uint32_t boxDrawUs = 0;

  float roll = 0, pitch = 0, yaw = 0;
  bool got = false;
  uint32_t imuReadUs = 0;

  uint32_t t0 = micros();
  while (true) {
    float r, p, y;
    if (!readAnglesOnce(r, p, y)) break;
    roll = r; pitch = p; yaw = y;
    got = true;
    if (micros() - t0 > 2500) break;
  }
  imuReadUs = micros() - t0;
  if (!got) return;

  if (!zeroSet) {
    updateZeroCountdown(nowMs);
    if (nowMs - zeroArmMs >= ZERO_DELAY_MS) {
      applyZeroCalibration(roll, pitch, yaw, nowMs);
      roll = 0.0f;
      pitch = 0.0f;
      yaw = 0.0f;
    }
  }

  // swap roll/pitch
  float tmp = roll; roll = pitch; pitch = tmp;

  int16_t pQ = quantizeDeg2(pitch);
  int16_t yQ = wrapAngle180i(quantizeDeg2(yaw));

  // ===== NET accept =====
  if (nowMs - lastTcpPollMs >= tcpPollIntervalMs) {
    uint32_t tcpPollStartUs = micros();
    lastTcpPollMs = nowMs;

    bool connectedNow = (client && client.connected());
    if (!connectedNow) {
      WiFiClient newClient = server.available();
      if (newClient) {
        client = newClient;
        client.setTimeout(5);
        client.println("HELLO from UNO R4 WiFi");
        Serial.println("Client connected.");
        lastTcpAliveMs = nowMs;
        connectedNow = true;
      }
    }
    if (connectedNow && lastTcpAliveMs != 0 && (nowMs - lastTcpAliveMs > TCP_ALIVE_TIMEOUT_MS)) {
      client.stop();
      connectedNow = false;
      Serial.println("TCP heartbeat timeout.");
    }
    tcpConnectedCached = connectedNow;
    netPollUs = micros() - tcpPollStartUs;
  }

  // ===== NET read =====
  if (tcpConnectedCached && client.available()) {
    uint32_t tcpReadStartUs = micros();
    String line = readLine(client);
    line.trim();
    if (line.length() > 0) {
      float newDegPerPx = 0.0f;
      float newFovXDeg = 0.0f;
      float newFovYDeg = 0.0f;
      uint32_t newTcpPollMs = 0;
      int16_t newBoxSizePx = 0;
      uint32_t newBoxRefreshMs = 0;
      String msgOnly;
      bool newBoxRenderEnabled = false;
      bool newBoxDeltaRenderEnabled = false;

      if (isPingCommand(line)) {
        lastTcpAliveMs = nowMs;
      } else if (parseMsgOnlyCommand(line, msgOnly)) {
        lastTcpAliveMs = nowMs;
        char msgBuf[48];
        msgOnly.toCharArray(msgBuf, sizeof(msgBuf));
        msgBuf[30] = '\0';
        Serial.print("Msg updated: ");
        Serial.println(msgBuf);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;MSGONLY:");
          client.println(msgBuf);
        }
      } else if (parseDisplayFovXCommand(line, newFovXDeg)) {
        lastTcpAliveMs = nowMs;
        displayFovXDeg = newFovXDeg;
        eraseOldBox();
        Serial.print("Display FOV X updated: ");
        Serial.println(displayFovXDeg, 2);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;CFG:FOV_X:");
          client.println(displayFovXDeg, 2);
        }
      } else if (parseDisplayFovYCommand(line, newFovYDeg)) {
        lastTcpAliveMs = nowMs;
        displayFovYDeg = newFovYDeg;
        eraseOldBox();
        Serial.print("Display FOV Y updated: ");
        Serial.println(displayFovYDeg, 2);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;CFG:FOV_Y:");
          client.println(displayFovYDeg, 2);
        }
      } else if (parseTcpPollMsCommand(line, newTcpPollMs)) {
        lastTcpAliveMs = nowMs;
        tcpPollIntervalMs = newTcpPollMs;
        Serial.print("TCP poll interval updated: ");
        Serial.println(tcpPollIntervalMs);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;CFG:TCP_POLL_MS:");
          client.println(tcpPollIntervalMs);
        }
      } else if (parseBoxSizeCommand(line, newBoxSizePx)) {
        lastTcpAliveMs = nowMs;
        boxSizePx = newBoxSizePx;
        eraseOldBox();
        Serial.print("Box size updated: ");
        Serial.println(boxSizePx);
      } else if (parseBoxRefreshMsCommand(line, newBoxRefreshMs)) {
        lastTcpAliveMs = nowMs;
        boxRefreshIntervalMs = newBoxRefreshMs;
        Serial.print("Box refresh ms updated: ");
        Serial.println(boxRefreshIntervalMs);
      } else if (parseBoxRenderCommand(line, newBoxRenderEnabled)) {
        lastTcpAliveMs = nowMs;
        boxRenderEnabled = newBoxRenderEnabled;
        if (!boxRenderEnabled) eraseOldBox();
        Serial.print("Box render updated: ");
        Serial.println(boxRenderEnabled ? "ON" : "OFF");
      } else if (parseBoxDeltaRenderCommand(line, newBoxDeltaRenderEnabled)) {
        lastTcpAliveMs = nowMs;
        boxDeltaRenderEnabled = newBoxDeltaRenderEnabled;
        Serial.print("Box delta render updated: ");
        Serial.println(boxDeltaRenderEnabled ? "ON" : "OFF");
      } else if (parseDisplayDegPerPxCommand(line, newDegPerPx)) {
        lastTcpAliveMs = nowMs;
        displayFovXDeg = newDegPerPx * (float)SCREEN_W;
        displayFovYDeg = newDegPerPx * (float)SCREEN_H;
        eraseOldBox();
        Serial.print("Legacy display deg/px converted to FOV X/Y: ");
        Serial.print(displayFovXDeg, 2);
        Serial.print("/");
        Serial.println(displayFovYDeg, 2);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;CFG:DEG_PER_PX:");
          client.println(newDegPerPx, 4);
        }
      } else if (isCenterCommand(line)) {
        lastTcpAliveMs = nowMs;
        spawnPitchQ = pQ;
        spawnYawQ   = yQ;
        spawnSet    = true;

        Serial.print("Center command applied: pitch=");
        Serial.print(spawnPitchQ);
        Serial.print(" yaw=");
        Serial.println(spawnYawQ);

        if (ENABLE_PACKET_ACK) {
          client.println("ACK;CMD:CENTER");
        }
      } else {
        String msg;
        float x = 0.0f, y = 0.0f;

        if (parsePacket(line, msg, x, y)) {
          lastTcpAliveMs = nowMs;
          // Target packet X/Y are offsets from the current device direction.
          // X=0/Y=0 means the marker is centered at the current pose.
          spawnPitchQ = pQ + quantizeDeg2((float)x);
          spawnYawQ   = wrapAngle180i(yQ + quantizeDeg2((float)y));
          spawnSet    = true;

          char msgBuf[48];
          msg.toCharArray(msgBuf, sizeof(msgBuf));
          msgBuf[30] = '\0';
          Serial.println("Target updated.");

          // ACK can be disabled to reduce Wi-Fi blocking latency.
          if (ENABLE_PACKET_ACK) {
            client.print("ACK;MSG:");
            client.print(msg);
            client.print(";X:");
            client.print(x, 2);
            client.print(";Y:");
            client.println(y, 2);
          }
        } else {
          client.print("ERR;BAD_PACKET;");
          client.println(line);
        }
      }
    }
    netReadUs = micros() - tcpReadStartUs;
  }

  bool tcpConnected = tcpConnectedCached;
  if (tcpConnected != lastTcpConnectedState) {
    Serial.println(tcpConnected ? "TCP state -> OK" : "TCP state -> WAIT");
    lastTcpConnectedState = tcpConnected;
  }

  if (millis() - lastTcpStatusDrawMs >= TCP_STATUS_REFRESH_MS) {
    drawTCPStatus(tcpConnected);
    lastTcpStatusDrawMs = millis();
  }

  // ===== BOX draw =====
  int16_t targetPitchQ = spawnSet ? spawnPitchQ : 0;
  int16_t targetYawQ   = spawnSet ? spawnYawQ : 0;
  int16_t pitchRelQ = targetPitchQ - pQ;
  int16_t yawRelQ   = deltaAngle180i(targetYawQ, yQ);

  bool onTarget = (abs((int)pitchRelQ) <= TARGET_TOL_DEG) &&
                  (abs((int)yawRelQ)   <= TARGET_TOL_DEG);

  if (!boxRenderEnabled) {
    eraseOldBox();
  } else if (nowMs - lastBoxDrawMs >= boxRefreshIntervalMs) {
    uint32_t boxDrawStartUs = micros();
    updateBox(pitchRelQ, yawRelQ, onTarget, tcpConnected);
    lastBoxDrawMs = nowMs;
    boxDrawUs = micros() - boxDrawStartUs;
  }

  uint32_t loopUs = micros() - loopStartUs;

  if (ENABLE_SERIAL_DEBUG && (millis() - lastSerialDebugMs >= SERIAL_DEBUG_MS)) {
    Serial.print("DBG loop_dt_ms=");
    Serial.print(loopDtMs);
    Serial.print(" loop_us=");
    Serial.print(loopUs);
    Serial.print(" imu_us=");
    Serial.print(imuReadUs);
    Serial.print(" tcp_poll_us=");
    Serial.print(netPollUs);
    Serial.print(" tcp_poll_cfg_ms=");
    Serial.print(tcpPollIntervalMs);
    Serial.print(" fov_x=");
    Serial.print(displayFovXDeg, 1);
    Serial.print(" fov_y=");
    Serial.print(displayFovYDeg, 1);
    Serial.print(" tcp_read_us=");
    Serial.print(netReadUs);
    Serial.print(" box_us=");
    Serial.print(boxDrawUs);
    Serial.print(" box_cfg_on=");
    Serial.print(boxRenderEnabled ? 1 : 0);
    Serial.print(" box_cfg_delta=");
    Serial.print(boxDeltaRenderEnabled ? 1 : 0);
    Serial.print(" box_cfg_size=");
    Serial.print(boxSizePx);
    Serial.print(" box_cfg_refresh_ms=");
    Serial.print(boxRefreshIntervalMs);
    Serial.print(" tcp=");
    Serial.print(tcpConnected ? "OK" : "WAIT");
    Serial.print(" yaw=");
    Serial.print(yQ);
    Serial.print(" targetYaw=");
    Serial.print(targetYawQ);
    Serial.print(" yawRel=");
    Serial.println(yawRelQ);
    lastSerialDebugMs = millis();
  }
}
