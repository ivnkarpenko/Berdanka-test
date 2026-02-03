#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "ICM_20948.h"
#include <WiFiS3.h>

// ================== WIFI ==================
// Previous network (kept for reference)
// const char* WIFI_SSID = "JetsonAP";
// const char* WIFI_PASS = "12345678";
// Current network
const char* WIFI_SSID = "GABELLA";
const char* WIFI_PASS = "J8f2829a";
constexpr uint16_t SERVER_PORT = 3333;

WiFiServer server(SERVER_PORT);
WiFiClient client;

// ================== UI (Processing will render) ==================
constexpr float   PX_PER_DEG = 3.0f;   // px per degree (reference)
constexpr int16_t BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

char lastMsg[48] = "-";
char lastIP[20]  = "0.0.0.0";

// ================== SPAWN (from Jetson) ==================
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

// Freeze yaw near gimbal lock
float yawStable      = 0.0f;
bool  yawStableInit  = false;
const float pitchLockThreshold = 80.0f;

// ================== helpers ==================
static inline int16_t quantizeDeg2(float a) {
  float q = (float)ANGLE_STEP_DEG * roundf(a / (float)ANGLE_STEP_DEG);
  return (int16_t)q;
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
    zeroRoll  = roll;
    zeroPitch = pitch;
    zeroYaw   = yaw;
    zeroSet   = true;
  }

  roll  -= zeroRoll;
  pitch -= zeroPitch;
  yaw   -= zeroYaw;

  if (!yawStableInit) {
    yawStable = yaw;
    yawStableInit = true;
  }
  if (fabs(pitch) < pitchLockThreshold) yawStable = yaw;

  outRoll  = roll;
  outPitch = pitch;
  outYaw   = yawStable;
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

// X/Y only digits (no sign) as in Tkinter
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

  // allow optional sign and decimal point
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
  Serial.print("Connecting WiFi to: ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(800);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");

  IPAddress ip = WiFi.localIP();
  unsigned long t0 = millis();
  while (ip[0] == 0 && (millis() - t0) < 15000) {
    delay(250);
    ip = WiFi.localIP();
  }

  snprintf(lastIP, sizeof(lastIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  server.begin();
  Serial.print("TCP server started on port ");
  Serial.println(SERVER_PORT);
}

// ================== Arduino ==================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  while (!startIMU()) {
    Serial.println("IMU init fail, retry...");
    delay(200);
  }

  wifiConnectAndStartServer();

  Serial.println("t_ms,roll,pitch,yaw,rollQ,pitchQ,yawQ,spawnSet,spawnPitchQ,spawnYawQ,onTarget,msg,ip");
}

void loop() {
  float roll = 0, pitch = 0, yaw = 0;
  bool got = false;

  uint32_t t0 = micros();
  while (true) {
    float r, p, y;
    if (!readAnglesOnce(r, p, y)) break;
    roll = r; pitch = p; yaw = y;
    got = true;
    if (micros() - t0 > 2500) break;
  }
  if (!got) return;

  // swap roll/pitch
  float tmp = roll; roll = pitch; pitch = tmp;

  int16_t rQ = quantizeDeg2(roll);
  int16_t pQ = quantizeDeg2(pitch);
  int16_t yQ = quantizeDeg2(yaw);

  // ===== NET accept =====
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      client.setTimeout(5);
      client.println("HELLO from UNO R4 WiFi");
      Serial.println("Client connected.");
    }
  }

  // ===== NET read =====
  if (client && client.connected() && client.available()) {
    String line = readLine(client);
    line.trim();
    if (line.length() > 0) {
      String msg;
      float x = 0.0f, y = 0.0f;

      if (parsePacket(line, msg, x, y)) {
        // spawn (deg, step 2)
        spawnPitchQ = quantizeDeg2((float)x);
        spawnYawQ   = quantizeDeg2((float)y);
        spawnSet    = true;

        // Msg for HUD
        char msgBuf[48];
        msg.toCharArray(msgBuf, sizeof(msgBuf));
        msgBuf[30] = '\0';
        strncpy(lastMsg, msgBuf, sizeof(lastMsg) - 1);
        lastMsg[sizeof(lastMsg) - 1] = '\0';

        // ACK
        client.print("ACK;MSG:");
        client.print(msg);
        client.print(";X:");
        client.print(x, 2);
        client.print(";Y:");
        client.println(y, 2);
      } else {
        client.print("ERR;BAD_PACKET;");
        client.println(line);
      }
    }
  }

  int16_t pitchRelQ = pQ - (spawnSet ? spawnPitchQ : 0);
  int16_t yawRelQ   = yQ - (spawnSet ? spawnYawQ   : 0);

  bool onTarget = (abs((int)pitchRelQ) <= TARGET_TOL_DEG) &&
                  (abs((int)yawRelQ)   <= TARGET_TOL_DEG);

  // ===== Serial output for Processing =====
  Serial.print(millis());
  Serial.print(',');
  Serial.print(roll, 2); Serial.print(',');
  Serial.print(pitch, 2); Serial.print(',');
  Serial.print(yaw, 2); Serial.print(',');
  Serial.print(rQ); Serial.print(',');
  Serial.print(pQ); Serial.print(',');
  Serial.print(yQ); Serial.print(',');
  Serial.print(spawnSet ? 1 : 0); Serial.print(',');
  Serial.print(spawnSet ? spawnPitchQ : 0); Serial.print(',');
  Serial.print(spawnSet ? spawnYawQ : 0); Serial.print(',');
  Serial.print(onTarget ? 1 : 0); Serial.print(',');
  Serial.print(lastMsg); Serial.print(',');
  Serial.println(lastIP);
}
