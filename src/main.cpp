#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
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
constexpr float   DEFAULT_DEG_PER_PX = 1.0f / 3.0f;
constexpr int16_t BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

constexpr uint8_t TEXT_SIZE = 1;
constexpr uint8_t STATUS_TEXT_SIZE = 2;

// HUD positions
constexpr int16_t TXT_X_LABEL = 10;
constexpr int16_t TXT_X_VAL   = 80;
constexpr int16_t TXT_Y_ROLL  = 10;
constexpr int16_t TXT_Y_PITCH = 28;
constexpr int16_t TXT_Y_YAW   = 46;
constexpr int16_t TXT_Y_MSG   = 64;
constexpr int16_t TXT_Y_IP    = 82;
constexpr int16_t STATUS_X      = 10;
constexpr int16_t STATUS_Y_IMU  = 110;
constexpr int16_t STATUS_Y_TCP  = 136;

// HUD auto refresh
constexpr uint32_t HUD_REFRESH_MS = 10000;
constexpr uint32_t TCP_STATUS_REFRESH_MS = 200;
constexpr uint32_t TCP_POLL_MS = 80;
constexpr uint32_t TCP_ALIVE_TIMEOUT_MS = 2500;
constexpr uint32_t BOX_REFRESH_MS = 33;     // ~30 FPS box redraw cap
constexpr uint32_t SERIAL_DEBUG_MS = 2000;  // reduce serial overhead
constexpr bool ENABLE_SERIAL_DEBUG = true;
constexpr bool ENABLE_PACKET_ACK = false;
constexpr bool ENABLE_PERIODIC_HUD_REFRESH = false;

// edge marker visible thickness (px)
constexpr int16_t EDGE_VISIBLE_PX = 5;

// Values on screen
int16_t lastDispRoll  = 32767;
int16_t lastDispPitch = 32767;
int16_t lastDispYaw   = 32767;

char lastMsg[48] = "-";
char lastIP[20]  = "0.0.0.0";

uint32_t lastHudRefreshMs = 0;
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
float    displayDegPerPx = DEFAULT_DEG_PER_PX;

// ================== SPAWN (from Laptop) ==================
int16_t spawnPitchQ = 0;
int16_t spawnYawQ   = 0;
bool    spawnSet    = false;

struct PersistedTarget {
  uint32_t magic;
  int16_t pitchQ;
  int16_t yawQ;
  char msg[31];
};

constexpr uint32_t TARGET_MAGIC = 0x42524441UL; // "BRDA"
constexpr int EEPROM_TARGET_ADDR = 0;

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

// ================== Cross / HUD ==================
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

void drawStaticTextLabels() {
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);

  tft.setCursor(TXT_X_LABEL, TXT_Y_ROLL);  tft.print(F("Roll ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_PITCH); tft.print(F("Pitch ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_YAW);   tft.print(F("Yaw ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_MSG);   tft.print(F("Msg ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_IP);    tft.print(F("IP ="));
}

void drawValueIfChanged(int16_t x, int16_t y, int16_t v, int16_t &lastV) {
  if (v == lastV) return;

  char buf[12];
  snprintf(buf, sizeof(buf), "%+5d", (int)v);

  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);

  lastV = v;
}

void drawStringIfChanged(int16_t x, int16_t y, const char* s, char* last, size_t lastSz, int padWidth) {
  if (strncmp(s, last, lastSz) == 0) return;

  char buf[80];
  snprintf(buf, sizeof(buf), "%-*s", padWidth, s);

  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);

  strncpy(last, s, lastSz - 1);
  last[lastSz - 1] = '\0';
}

void saveTargetToEEPROM(const char* msg) {
  PersistedTarget target{};
  target.magic = TARGET_MAGIC;
  target.pitchQ = spawnPitchQ;
  target.yawQ = spawnYawQ;

  strncpy(target.msg, msg, sizeof(target.msg) - 1);
  target.msg[sizeof(target.msg) - 1] = '\0';

  EEPROM.put(EEPROM_TARGET_ADDR, target);
}

void clearTargetInEEPROM() {
  PersistedTarget target{};
  EEPROM.put(EEPROM_TARGET_ADDR, target);
}

bool loadTargetFromEEPROM() {
  PersistedTarget target{};
  EEPROM.get(EEPROM_TARGET_ADDR, target);

  if (target.magic != TARGET_MAGIC) return false;

  spawnPitchQ = quantizeDeg2((float)target.pitchQ);
  spawnYawQ = wrapAngle180i(target.yawQ);
  spawnSet = true;

  drawStringIfChanged(TXT_X_VAL, TXT_Y_MSG, target.msg, lastMsg, sizeof(lastMsg), 30);
  return true;
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

// Полный рефреш HUD (раз в 10 сек)
void refreshHUDForce(int16_t rollQ, int16_t pitchQ, int16_t yawQ) {
  drawStaticTextLabels();

  char buf[16];
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);

  snprintf(buf, sizeof(buf), "%+5d", (int)rollQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_ROLL);  tft.print(buf);

  snprintf(buf, sizeof(buf), "%+5d", (int)pitchQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_PITCH); tft.print(buf);

  snprintf(buf, sizeof(buf), "%+5d", (int)yawQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_YAW);   tft.print(buf);

  char m[80];  snprintf(m, sizeof(m),  "%-30s", lastMsg);
  tft.setCursor(TXT_X_VAL, TXT_Y_MSG); tft.print(m);

  char ip[80]; snprintf(ip, sizeof(ip), "%-18s", lastIP);
  tft.setCursor(TXT_X_VAL, TXT_Y_IP);  tft.print(ip);

  // Force redraw status lines in case moving graphics overwrote them.
  tft.setTextSize(STATUS_TEXT_SIZE);
  tft.setTextColor(lastImuStatusColor, ILI9488_BLACK);
  tft.setCursor(STATUS_X, STATUS_Y_IMU);
  tft.print("                            ");
  tft.setCursor(STATUS_X, STATUS_Y_IMU);
  tft.print(lastImuStatusLine);

  tft.setTextColor(lastTcpStatusColor, ILI9488_BLACK);
  tft.setCursor(STATUS_X, STATUS_Y_TCP);
  tft.print("                            ");
  tft.setCursor(STATUS_X, STATUS_Y_TCP);
  tft.print(lastTcpStatusLine);

  lastDispRoll  = rollQ;
  lastDispPitch = pitchQ;
  lastDispYaw   = yawQ;

  lastHudRefreshMs = millis();
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
  clearTargetInEEPROM();
  drawStringIfChanged(TXT_X_VAL, TXT_Y_MSG, "-", lastMsg, sizeof(lastMsg), 30);

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
    outYaw = wrapAngle180f(yaw);
    return true;
  }

  roll  -= zeroRoll;
  pitch -= zeroPitch;
  yaw   -= zeroYaw;
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

static bool parseDisplayDegPerPxCommand(const String& s, float& degPerPx) {
  const String prefix = "CFG:DEG_PER_PX:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;

  for (size_t i = 0; i < tail.length(); i++) {
    char c = tail[i];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.')) return false;
  }

  degPerPx = tail.toFloat();
  return degPerPx > 0.0f;
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

  IPAddress ip = WiFi.localIP();
  snprintf(lastIP, sizeof(lastIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

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
  float pxPerDeg = (displayDegPerPx > 0.0001f) ? (1.0f / displayDegPerPx) : (1.0f / DEFAULT_DEG_PER_PX);
  int32_t centerX = (int32_t)lroundf((float)CX + (float)yawRelQ * pxPerDeg);
  int32_t centerY = (int32_t)lroundf((float)CY + (float)pitchRelQ * pxPerDeg);

  int32_t boxX = centerX - BOX_SIZE / 2;
  int32_t boxY = centerY - BOX_SIZE / 2;

  if (boxX > (int32_t)SCREEN_W - EDGE_VISIBLE_PX) boxX = (int32_t)SCREEN_W - EDGE_VISIBLE_PX;
  if (boxX < (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX)) boxX = (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX);

  if (boxY > (int32_t)SCREEN_H - EDGE_VISIBLE_PX) boxY = (int32_t)SCREEN_H - EDGE_VISIBLE_PX;
  if (boxY < (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX)) boxY = (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX);

  int16_t nx, ny, nw, nh;
  bool ok = clipRect(boxX, boxY, BOX_SIZE, BOX_SIZE, nx, ny, nw, nh);

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
  drawStaticTextLabels();
  drawTCPStatus(false);

  if (loadTargetFromEEPROM()) {
    Serial.print("Target restored: pitch=");
    Serial.print(spawnPitchQ);
    Serial.print(" yaw=");
    Serial.println(spawnYawQ);
  } else {
    Serial.println("No saved target in EEPROM.");
  }

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

  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setTextSize(TEXT_SIZE);
  tft.setCursor(TXT_X_VAL, TXT_Y_IP);  tft.print(lastIP);
  tft.setCursor(TXT_X_VAL, TXT_Y_MSG); tft.print(lastMsg);

  lastHudRefreshMs = millis();
}

void loop() {
  uint32_t loopStartUs = micros();
  uint32_t nowMs = millis();
  uint32_t loopDtMs = (lastLoopTickMs == 0) ? 0 : (nowMs - lastLoopTickMs);
  lastLoopTickMs = nowMs;
  uint32_t netPollUs = 0;
  uint32_t netReadUs = 0;
  uint32_t boxDrawUs = 0;
  uint32_t hudRefreshUs = 0;

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

  int16_t rQ = quantizeDeg2(roll);
  int16_t pQ = quantizeDeg2(pitch);
  int16_t yQ = wrapAngle180i(quantizeDeg2(yaw));

  // обновляем числа (быстро)
  drawValueIfChanged(TXT_X_VAL, TXT_Y_ROLL,  rQ, lastDispRoll);
  drawValueIfChanged(TXT_X_VAL, TXT_Y_PITCH, pQ, lastDispPitch);
  drawValueIfChanged(TXT_X_VAL, TXT_Y_YAW,   yQ, lastDispYaw);

  // ===== NET accept =====
  if (nowMs - lastTcpPollMs >= TCP_POLL_MS) {
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

      if (isPingCommand(line)) {
        lastTcpAliveMs = nowMs;
      } else if (parseDisplayDegPerPxCommand(line, newDegPerPx)) {
        lastTcpAliveMs = nowMs;
        displayDegPerPx = newDegPerPx;
        Serial.print("Display deg/px updated: ");
        Serial.println(displayDegPerPx, 4);
        if (ENABLE_PACKET_ACK) {
          client.print("ACK;CFG:DEG_PER_PX:");
          client.println(displayDegPerPx, 4);
        }
      } else if (isCenterCommand(line)) {
        lastTcpAliveMs = nowMs;
        spawnPitchQ = pQ;
        spawnYawQ   = yQ;
        spawnSet    = true;

        drawStringIfChanged(TXT_X_VAL, TXT_Y_MSG, "CENTER", lastMsg, sizeof(lastMsg), 30);
        saveTargetToEEPROM("CENTER");
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
          // spawn (deg, step 2)
          spawnPitchQ = quantizeDeg2((float)x);
          spawnYawQ   = wrapAngle180i(quantizeDeg2((float)y));
          spawnSet    = true;

          // Msg на HUD
          char msgBuf[48];
          msg.toCharArray(msgBuf, sizeof(msgBuf));
          msgBuf[30] = '\0';
          drawStringIfChanged(TXT_X_VAL, TXT_Y_MSG, msgBuf, lastMsg, sizeof(lastMsg), 30);
          saveTargetToEEPROM(msgBuf);
          Serial.println("Target saved to EEPROM.");

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
  int16_t pitchRelQ = pQ - (spawnSet ? spawnPitchQ : 0);
  int16_t yawRelQ   = deltaAngle180i(yQ, (spawnSet ? spawnYawQ : 0));

  bool onTarget = (abs((int)pitchRelQ) <= TARGET_TOL_DEG) &&
                  (abs((int)yawRelQ)   <= TARGET_TOL_DEG);

  if (nowMs - lastBoxDrawMs >= BOX_REFRESH_MS) {
    uint32_t boxDrawStartUs = micros();
    updateBox(pitchRelQ, yawRelQ, onTarget, tcpConnected);
    lastBoxDrawMs = nowMs;
    boxDrawUs = micros() - boxDrawStartUs;
  }

  // ===== HUD refresh every 10s =====
  if (ENABLE_PERIODIC_HUD_REFRESH && (millis() - lastHudRefreshMs >= HUD_REFRESH_MS)) {
    uint32_t hudRefreshStartUs = micros();
    refreshHUDForce(rQ, pQ, yQ);
    hudRefreshUs = micros() - hudRefreshStartUs;
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
    Serial.print(" tcp_read_us=");
    Serial.print(netReadUs);
    Serial.print(" box_us=");
    Serial.print(boxDrawUs);
    Serial.print(" hud_us=");
    Serial.print(hudRefreshUs);
    Serial.print(" tcp=");
    Serial.print(tcpConnected ? "OK" : "WAIT");
    Serial.print(" yaw=");
    Serial.print(yQ);
    Serial.print(" spawnYaw=");
    Serial.print(spawnYawQ);
    Serial.print(" yawRel=");
    Serial.println(yawRelQ);
    lastSerialDebugMs = millis();
  }
}
