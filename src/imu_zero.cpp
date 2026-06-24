#include "imu_zero.h"

#include <math.h>
#include <Wire.h>
#include <ILI9488.h>
#include "app_state.h"
#include "angle_utils.h"
#include "ui.h"

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

void resetZeroWarmupSamples() {
  zeroWarmupRollSum = 0.0f;
  zeroWarmupPitchSum = 0.0f;
  zeroWarmupYawSinSum = 0.0f;
  zeroWarmupYawCosSum = 0.0f;
  zeroWarmupSampleCount = 0;
}

void beginZeroWarmup(uint32_t nowMs) {
  zeroSet = false;
  zeroArmMs = nowMs;
  lastZeroCountdownDrawMs = 0;
  lastZeroCountdownSec = 999;
  resetZeroWarmupSamples();

  // Wi-Fi startup can leave stale DMP frames in FIFO. Start the visible
  // 15-second warmup from a clean queue.
  imu.resetDMP();
  imu.resetFIFO();
}

void addZeroWarmupSample(float roll, float pitch, float yaw, uint32_t nowMs) {
  if (zeroSet) return;

  uint32_t elapsed = nowMs - zeroArmMs;
  uint32_t averageStartMs = (ZERO_DELAY_MS > ZERO_AVERAGE_WINDOW_MS) ?
                            (ZERO_DELAY_MS - ZERO_AVERAGE_WINDOW_MS) : 0;
  if (elapsed < averageStartMs) return;

  float yawRad = yaw * PI / 180.0f;
  zeroWarmupRollSum += roll;
  zeroWarmupPitchSum += pitch;
  zeroWarmupYawSinSum += sinf(yawRad);
  zeroWarmupYawCosSum += cosf(yawRad);
  if (zeroWarmupSampleCount < UINT16_MAX) zeroWarmupSampleCount++;
}

bool getZeroWarmupAverage(float &roll, float &pitch, float &yaw) {
  if (zeroWarmupSampleCount < ZERO_MIN_AVERAGE_SAMPLES) return false;

  float invCount = 1.0f / (float)zeroWarmupSampleCount;
  roll = zeroWarmupRollSum * invCount;
  pitch = zeroWarmupPitchSum * invCount;
  yaw = atan2f(zeroWarmupYawSinSum, zeroWarmupYawCosSum) * 180.0f / PI;
  yaw = wrapAngle180f(yaw);
  return true;
}

void applyZeroCalibration(float roll, float pitch, float yaw, uint32_t nowMs) {
  zeroRoll = roll;
  zeroPitch = pitch;
  zeroYaw = yaw;
  zeroSet = true;

  // Lock the initial target after the next display-coordinate transform.
  // This guarantees the marker starts exactly at the rendered center.
  spawnPitchQ = 0;
  spawnYawQ = 0;
  spawnSet = false;
  lockTargetToCurrentFrame = true;

  lastYaw = 0.0f;
  lastMove = nowMs;
  yawStable = 0.0f;
  yawStableInit = true;

  tft.fillScreen(ILI9488_BLACK);
  drawCrossFull();
  lastBoxValid = false;
  lastImuStatusLine[0] = '\0';
  lastTcpStatusLine[0] = '\0';
  lastImuStatusColor = 0xFFFF;
  lastTcpStatusColor = 0xFFFF;
  drawIMUStatus("IMU OK", ILI9488_GREEN);
  drawNetStatus(tcpConnectedCached, lastUdpAliveMs != 0 && (nowMs - lastUdpAliveMs <= UDP_ALIVE_TIMEOUT_MS));
  Serial.println("Zero calibration applied.");
}

void updateZeroCountdown(uint32_t nowMs) {
  if (zeroSet) return;
  if (nowMs - lastZeroCountdownDrawMs < ZERO_COUNTDOWN_REFRESH_MS) return;

  uint32_t elapsed = nowMs - zeroArmMs;
  uint32_t remainingMs = (elapsed >= ZERO_DELAY_MS) ? 0 : (ZERO_DELAY_MS - elapsed);
  uint32_t remainingSec = (remainingMs + 999U) / 1000U;

  if (remainingSec != lastZeroCountdownSec) {
    drawZeroCountdown(remainingSec);
    lastZeroCountdownSec = remainingSec;
  }
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
    // Store raw yaw for zero calibration. The display sign is applied after zero.
    outYaw = yaw;
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

bool readLatestAngles(float &roll, float &pitch, float &yaw,
                      uint32_t budgetUs, uint32_t &elapsedUs) {
  uint32_t t0 = micros();
  bool got = false;

  while (true) {
    float r, p, y;
    if (!readAnglesOnce(r, p, y)) break;
    roll = r;
    pitch = p;
    yaw = y;
    got = true;
    if (micros() - t0 > budgetUs) break;
  }

  elapsedUs = micros() - t0;
  return got;
}
