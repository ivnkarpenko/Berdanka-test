#include "imu_zero.h"

#include <math.h>
#include <Wire.h>
#include <ILI9488.h>
#include "app_state.h"
#include "angle_utils.h"
#include "ui.h"

static bool zeroStableWindowActive = false;
static uint32_t zeroStableStartMs = 0;
static float zeroStableMinRoll = 0.0f;
static float zeroStableMaxRoll = 0.0f;
static float zeroStableMinPitch = 0.0f;
static float zeroStableMaxPitch = 0.0f;
static float zeroStableMinYaw = 0.0f;
static float zeroStableMaxYaw = 0.0f;

static bool yawAutoZeroWindowActive = false;
static bool yawAutoZeroPrevValid = false;
static uint32_t yawAutoZeroWindowStartMs = 0;
static uint32_t yawAutoZeroLastUpdateMs = 0;
static float yawAutoZeroPrevRawRoll = 0.0f;
static float yawAutoZeroPrevRawPitch = 0.0f;
static float yawAutoZeroPrevRawYaw = 0.0f;
static float yawAutoZeroYawUnwrapped = 0.0f;
static float yawAutoZeroMinRoll = 0.0f;
static float yawAutoZeroMaxRoll = 0.0f;
static float yawAutoZeroMinPitch = 0.0f;
static float yawAutoZeroMaxPitch = 0.0f;
static float yawAutoZeroMinYaw = 0.0f;
static float yawAutoZeroMaxYaw = 0.0f;

static bool rawFrameFilterValid = false;
static bool rawFramePendingValid = false;
static float rawFrameLastRoll = 0.0f;
static float rawFrameLastPitch = 0.0f;
static float rawFrameLastYaw = 0.0f;
static float rawFramePendingRoll = 0.0f;
static float rawFramePendingPitch = 0.0f;
static float rawFramePendingYaw = 0.0f;

static float minFloat(float a, float b) {
  return (a < b) ? a : b;
}

static float maxFloat(float a, float b) {
  return (a > b) ? a : b;
}

static void resetZeroStableSums() {
  zeroWarmupRollSum = 0.0f;
  zeroWarmupPitchSum = 0.0f;
  zeroWarmupYawSinSum = 0.0f;
  zeroWarmupYawCosSum = 0.0f;
  zeroWarmupSampleCount = 0;
  zeroYawBiasRateDps = 0.0f;
}

static void resetYawAutoZeroState() {
  yawAutoZeroWindowActive = false;
  yawAutoZeroPrevValid = false;
  yawAutoZeroWindowStartMs = 0;
  yawAutoZeroLastUpdateMs = 0;
  yawAutoZeroPrevRawRoll = 0.0f;
  yawAutoZeroPrevRawPitch = 0.0f;
  yawAutoZeroPrevRawYaw = 0.0f;
  yawAutoZeroYawUnwrapped = 0.0f;
  yawAutoZeroMinRoll = 0.0f;
  yawAutoZeroMaxRoll = 0.0f;
  yawAutoZeroMinPitch = 0.0f;
  yawAutoZeroMaxPitch = 0.0f;
  yawAutoZeroMinYaw = 0.0f;
  yawAutoZeroMaxYaw = 0.0f;
  yawAutoZeroActive = false;
  yawAutoZeroStableMs = 0;
  yawAutoZeroUpdates = 0;
  yawAutoZeroCorrectionDeg = 0.0f;
}

static void resetRawFrameFilter() {
  rawFrameFilterValid = false;
  rawFramePendingValid = false;
  rawFrameLastRoll = 0.0f;
  rawFrameLastPitch = 0.0f;
  rawFrameLastYaw = 0.0f;
  rawFramePendingRoll = 0.0f;
  rawFramePendingPitch = 0.0f;
  rawFramePendingYaw = 0.0f;
}

static bool rawFrameStepTooLarge(float roll, float pitch, float yaw,
                                 float refRoll, float refPitch, float refYaw) {
  return fabsf(roll - refRoll) > IMU_RAW_DEGLITCH_STEP_DEG ||
         fabsf(pitch - refPitch) > IMU_RAW_DEGLITCH_STEP_DEG ||
         fabsf(deltaAngle180f(yaw, refYaw)) > IMU_RAW_DEGLITCH_STEP_DEG;
}

static void acceptRawFrame(float roll, float pitch, float yaw) {
  rawFrameLastRoll = roll;
  rawFrameLastPitch = pitch;
  rawFrameLastYaw = yaw;
  rawFrameFilterValid = true;
  rawFramePendingValid = false;
}

static bool acceptRawFrameOrDefer(float roll, float pitch, float yaw) {
  if (!rawFrameFilterValid) {
    acceptRawFrame(roll, pitch, yaw);
    return true;
  }

  if (!rawFrameStepTooLarge(roll, pitch, yaw,
                            rawFrameLastRoll, rawFrameLastPitch, rawFrameLastYaw)) {
    acceptRawFrame(roll, pitch, yaw);
    return true;
  }

  if (rawFramePendingValid &&
      !rawFrameStepTooLarge(roll, pitch, yaw,
                            rawFramePendingRoll, rawFramePendingPitch, rawFramePendingYaw)) {
    acceptRawFrame(roll, pitch, yaw);
    return true;
  }

  rawFramePendingRoll = roll;
  rawFramePendingPitch = pitch;
  rawFramePendingYaw = yaw;
  rawFramePendingValid = true;
  return false;
}

static void addZeroStableSample(float roll, float pitch, float yaw, float yawUnwrapped, uint32_t nowMs) {
  float yawRad = yaw * PI / 180.0f;
  zeroWarmupRollSum += roll;
  zeroWarmupPitchSum += pitch;
  zeroWarmupYawSinSum += sinf(yawRad);
  zeroWarmupYawCosSum += cosf(yawRad);

  (void)yawUnwrapped;
  (void)nowMs;
  if (zeroWarmupSampleCount < UINT16_MAX) zeroWarmupSampleCount++;
}

static void startZeroStableWindow(float roll, float pitch, float yaw, float yawUnwrapped, uint32_t nowMs) {
  resetZeroStableSums();
  zeroStableWindowActive = true;
  zeroStableStartMs = nowMs;
  zeroStableMinRoll = roll;
  zeroStableMaxRoll = roll;
  zeroStableMinPitch = pitch;
  zeroStableMaxPitch = pitch;
  zeroStableMinYaw = yawUnwrapped;
  zeroStableMaxYaw = yawUnwrapped;
  addZeroStableSample(roll, pitch, yaw, yawUnwrapped, nowMs);
}

static bool updateZeroStableWindow(float roll, float pitch, float yaw, float yawUnwrapped, uint32_t nowMs) {
  if (!zeroStableWindowActive) {
    startZeroStableWindow(roll, pitch, yaw, yawUnwrapped, nowMs);
    return true;
  }

  float nextMinRoll = minFloat(zeroStableMinRoll, roll);
  float nextMaxRoll = maxFloat(zeroStableMaxRoll, roll);
  float nextMinPitch = minFloat(zeroStableMinPitch, pitch);
  float nextMaxPitch = maxFloat(zeroStableMaxPitch, pitch);
  float nextMinYaw = minFloat(zeroStableMinYaw, yawUnwrapped);
  float nextMaxYaw = maxFloat(zeroStableMaxYaw, yawUnwrapped);

  if ((nextMaxRoll - nextMinRoll) > ZERO_STABLE_MAX_ROLL_SPAN_DEG ||
      (nextMaxPitch - nextMinPitch) > ZERO_STABLE_MAX_PITCH_SPAN_DEG ||
      (nextMaxYaw - nextMinYaw) > ZERO_STABLE_MAX_YAW_SPAN_DEG) {
    if (zeroWarmupRejectedCount < UINT16_MAX) zeroWarmupRejectedCount++;
    startZeroStableWindow(roll, pitch, yaw, yawUnwrapped, nowMs);
    return false;
  }

  zeroStableMinRoll = nextMinRoll;
  zeroStableMaxRoll = nextMaxRoll;
  zeroStableMinPitch = nextMinPitch;
  zeroStableMaxPitch = nextMaxPitch;
  zeroStableMinYaw = nextMinYaw;
  zeroStableMaxYaw = nextMaxYaw;
  addZeroStableSample(roll, pitch, yaw, yawUnwrapped, nowMs);
  return true;
}

static void startYawAutoZeroWindow(float rawRoll, float rawPitch, float rawYaw,
                                   float yawUnwrapped, uint32_t nowMs) {
  yawAutoZeroWindowActive = true;
  yawAutoZeroWindowStartMs = nowMs;
  yawAutoZeroMinRoll = rawRoll;
  yawAutoZeroMaxRoll = rawRoll;
  yawAutoZeroMinPitch = rawPitch;
  yawAutoZeroMaxPitch = rawPitch;
  yawAutoZeroMinYaw = yawUnwrapped;
  yawAutoZeroMaxYaw = yawUnwrapped;
  yawAutoZeroActive = false;
  yawAutoZeroStableMs = 0;
}

static bool updateYawAutoZeroWindow(float rawRoll, float rawPitch, float rawYaw,
                                    float yawUnwrapped, uint32_t nowMs) {
  (void)rawYaw;
  if (!yawAutoZeroWindowActive) {
    startYawAutoZeroWindow(rawRoll, rawPitch, rawYaw, yawUnwrapped, nowMs);
    return true;
  }

  float nextMinRoll = minFloat(yawAutoZeroMinRoll, rawRoll);
  float nextMaxRoll = maxFloat(yawAutoZeroMaxRoll, rawRoll);
  float nextMinPitch = minFloat(yawAutoZeroMinPitch, rawPitch);
  float nextMaxPitch = maxFloat(yawAutoZeroMaxPitch, rawPitch);
  float nextMinYaw = minFloat(yawAutoZeroMinYaw, yawUnwrapped);
  float nextMaxYaw = maxFloat(yawAutoZeroMaxYaw, yawUnwrapped);

  if ((nextMaxRoll - nextMinRoll) > YAW_AUTO_ZERO_MAX_ROLL_SPAN_DEG ||
      (nextMaxPitch - nextMinPitch) > YAW_AUTO_ZERO_MAX_PITCH_SPAN_DEG ||
      (nextMaxYaw - nextMinYaw) > YAW_AUTO_ZERO_MAX_YAW_SPAN_DEG) {
    startYawAutoZeroWindow(rawRoll, rawPitch, rawYaw, yawUnwrapped, nowMs);
    return false;
  }

  yawAutoZeroMinRoll = nextMinRoll;
  yawAutoZeroMaxRoll = nextMaxRoll;
  yawAutoZeroMinPitch = nextMinPitch;
  yawAutoZeroMaxPitch = nextMaxPitch;
  yawAutoZeroMinYaw = nextMinYaw;
  yawAutoZeroMaxYaw = nextMaxYaw;
  yawAutoZeroStableMs = nowMs - yawAutoZeroWindowStartMs;
  yawAutoZeroActive = yawAutoZeroStableMs >= YAW_AUTO_ZERO_STILL_WINDOW_MS;
  return true;
}

static void updateYawAutoZero(float rawRoll, float rawPitch, float rawYaw, uint32_t nowMs) {
  if (!YAW_AUTO_ZERO_ENABLE || zeroApplyMs == 0) return;

  if (nowMs - zeroApplyMs < YAW_AUTO_ZERO_START_DELAY_MS) {
    resetYawAutoZeroState();
    return;
  }

  if (!yawAutoZeroPrevValid) {
    yawAutoZeroPrevRawRoll = rawRoll;
    yawAutoZeroPrevRawPitch = rawPitch;
    yawAutoZeroPrevRawYaw = rawYaw;
    yawAutoZeroYawUnwrapped = rawYaw;
    yawAutoZeroPrevValid = true;
    startYawAutoZeroWindow(rawRoll, rawPitch, rawYaw, yawAutoZeroYawUnwrapped, nowMs);
    return;
  }

  float dRoll = rawRoll - yawAutoZeroPrevRawRoll;
  float dPitch = rawPitch - yawAutoZeroPrevRawPitch;
  float dYaw = deltaAngle180f(rawYaw, yawAutoZeroPrevRawYaw);
  yawAutoZeroPrevRawRoll = rawRoll;
  yawAutoZeroPrevRawPitch = rawPitch;
  yawAutoZeroPrevRawYaw = rawYaw;
  yawAutoZeroYawUnwrapped += dYaw;

  if (fabsf(dRoll) > YAW_AUTO_ZERO_MAX_ROLL_STEP_DEG ||
      fabsf(dPitch) > YAW_AUTO_ZERO_MAX_PITCH_STEP_DEG ||
      fabsf(dYaw) > YAW_AUTO_ZERO_MAX_YAW_STEP_DEG) {
    startYawAutoZeroWindow(rawRoll, rawPitch, rawYaw, yawAutoZeroYawUnwrapped, nowMs);
    return;
  }

  if (!updateYawAutoZeroWindow(rawRoll, rawPitch, rawYaw, yawAutoZeroYawUnwrapped, nowMs)) return;
  if (!yawAutoZeroActive) return;

  float zeroError = deltaAngle180f(rawYaw, zeroYaw);
  if (fabsf(zeroError) < YAW_AUTO_ZERO_DEADBAND_DEG ||
      fabsf(zeroError) > YAW_AUTO_ZERO_MAX_ERROR_DEG) {
    zeroYawBiasRateDps = 0.0f;
    return;
  }

  float correction = zeroError * YAW_AUTO_ZERO_GAIN;
  zeroYaw = wrapAngle180f(zeroYaw + correction);
  yawAutoZeroCorrectionDeg += correction;
  if (yawAutoZeroUpdates < UINT16_MAX) yawAutoZeroUpdates++;

  uint32_t dtMs = (yawAutoZeroLastUpdateMs == 0) ? 0 : (nowMs - yawAutoZeroLastUpdateMs);
  yawAutoZeroLastUpdateMs = nowMs;
  zeroYawBiasRateDps = (dtMs > 0) ? (correction * 1000.0f / (float)dtMs) : 0.0f;
}

static float stabilizeYaw(float yaw, float pitch, uint32_t nowMs) {
  if (!yawStableInit) {
    lastYaw = yaw;
    lastMove = nowMs;
    lastYawSampleMs = nowMs;
    yawStable = yaw;
    yawStableInit = true;
    return wrapAngle180f(yaw);
  }

  uint32_t dtMs = (lastYawSampleMs == 0) ? 0 : (nowMs - lastYawSampleMs);
  float yawStepDeg = fabsf(deltaAngle180f(yaw, lastYaw));
  float yawRateDps = (dtMs > 0) ? (yawStepDeg * 1000.0f / (float)dtMs) : 0.0f;

  if (yawRateDps > yawDriftLimitDps) {
    lastMove = nowMs;
  } else if (nowMs - lastMove > yawHoldTime) {
    yaw = lastYaw;
  }
  lastYawSampleMs = nowMs;
  lastYaw = wrapAngle180f(yaw);

  if (fabsf(pitch) < pitchLockThreshold) yawStable = yaw;
  return wrapAngle180f(yawStable);
}

static bool startIMUWithAd0(uint8_t ad0) {
  imu.begin(Wire, ad0);
  if (imu.status != ICM_20948_Stat_Ok) return false;

  if (imu.initializeDMP() != ICM_20948_Stat_Ok) return false;

  imuUsingQuat9 = false;
  imuQuatAccuracy = -1;
  if (IMU_USE_MAGNETIC_YAW) {
    ICM_20948_Status_e magStatus = imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR);
    ICM_20948_Status_e rateStatus = ICM_20948_Stat_Unknown;
    if (magStatus == ICM_20948_Stat_Ok) {
      rateStatus = imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0);
    }
    imuUsingQuat9 = (magStatus == ICM_20948_Stat_Ok) && (rateStatus == ICM_20948_Stat_Ok);
    if (!imuUsingQuat9) {
      imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR, false);
      if (!IMU_FALLBACK_TO_GAME_ROTATION) return false;
    }
  }

  if (!imuUsingQuat9) {
    if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) return false;
    if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok) return false;
  }

  if (imu.enableFIFO() != ICM_20948_Stat_Ok) return false;
  if (imu.enableDMP()  != ICM_20948_Stat_Ok) return false;
  if (imu.resetDMP()   != ICM_20948_Stat_Ok) return false;
  if (imu.resetFIFO()  != ICM_20948_Stat_Ok) return false;

  return true;
}

bool startIMU() {
  if (ENABLE_SERIAL_DEBUG) Serial.println("Trying ICM-20948 at 0x68 (AD0=0)");
  if (startIMUWithAd0(0)) return true;

  if (ENABLE_SERIAL_DEBUG) Serial.println("Trying ICM-20948 at 0x69 (AD0=1)");
  if (startIMUWithAd0(1)) return true;

  return false;
}

void resetZeroWarmupSamples() {
  resetZeroStableSums();
  zeroWarmupRejectedCount = 0;
  zeroWarmupPrevValid = false;
  zeroWarmupPrevRoll = 0.0f;
  zeroWarmupPrevPitch = 0.0f;
  zeroWarmupPrevYaw = 0.0f;
  zeroWarmupYawUnwrapped = 0.0f;
  zeroStableWindowActive = false;
  zeroStableStartMs = 0;
  zeroStableMinRoll = 0.0f;
  zeroStableMaxRoll = 0.0f;
  zeroStableMinPitch = 0.0f;
  zeroStableMaxPitch = 0.0f;
  zeroStableMinYaw = 0.0f;
  zeroStableMaxYaw = 0.0f;
  zeroApplyMs = 0;
  resetYawAutoZeroState();
  resetRawFrameFilter();
}

void beginZeroWarmup(uint32_t nowMs) {
  zeroSet = false;
  zeroArmMs = nowMs;
  lastZeroCountdownDrawMs = 0;
  lastZeroCountdownSec = 999;
  resetZeroWarmupSamples();

  // Wi-Fi startup can leave stale DMP frames in FIFO. Keep the DMP running so
  // its yaw estimate has the startup time to settle, but start zero from fresh
  // samples only.
  imu.resetFIFO();
}

void addZeroWarmupSample(float roll, float pitch, float yaw, uint32_t nowMs) {
  if (zeroSet) return;

  uint32_t elapsed = nowMs - zeroArmMs;
  if (!zeroWarmupPrevValid) {
    zeroWarmupPrevRoll = roll;
    zeroWarmupPrevPitch = pitch;
    zeroWarmupPrevYaw = yaw;
    zeroWarmupYawUnwrapped = yaw;
    zeroWarmupPrevValid = true;
    return;
  }

  float dRoll = roll - zeroWarmupPrevRoll;
  float dPitch = pitch - zeroWarmupPrevPitch;
  float dYaw = deltaAngle180f(yaw, zeroWarmupPrevYaw);
  zeroWarmupPrevRoll = roll;
  zeroWarmupPrevPitch = pitch;
  zeroWarmupPrevYaw = yaw;
  zeroWarmupYawUnwrapped += dYaw;

  if (elapsed < ZERO_SENSOR_SETTLE_MS) return;

  if (fabsf(dRoll) > ZERO_STABLE_MAX_ROLL_STEP_DEG ||
      fabsf(dPitch) > ZERO_STABLE_MAX_PITCH_STEP_DEG ||
      fabsf(dYaw) > ZERO_STABLE_MAX_YAW_STEP_DEG) {
    if (zeroWarmupRejectedCount < UINT16_MAX) zeroWarmupRejectedCount++;
    startZeroStableWindow(roll, pitch, yaw, zeroWarmupYawUnwrapped, nowMs);
    return;
  }

  updateZeroStableWindow(roll, pitch, yaw, zeroWarmupYawUnwrapped, nowMs);
}

uint32_t getZeroWarmupStableMs(uint32_t nowMs) {
  if (!zeroStableWindowActive) return 0;
  return nowMs - zeroStableStartMs;
}

bool isZeroWarmupReady(uint32_t nowMs) {
  if (!zeroStableWindowActive) return false;
  if (nowMs - zeroStableStartMs < ZERO_STABLE_WINDOW_MS) return false;
  if (zeroWarmupSampleCount < ZERO_MIN_AVERAGE_SAMPLES) return false;
  return true;
}

bool getZeroWarmupAverage(float &roll, float &pitch, float &yaw) {
  if (!isZeroWarmupReady(millis())) return false;

  float invCount = 1.0f / (float)zeroWarmupSampleCount;
  roll = zeroWarmupRollSum * invCount;
  pitch = zeroWarmupPitchSum * invCount;
  yaw = atan2f(zeroWarmupYawSinSum, zeroWarmupYawCosSum) * 180.0f / PI;
  yaw = wrapAngle180f(yaw);

  zeroYawBiasRateDps = 0.0f;
  return true;
}

void applyZeroCalibration(float roll, float pitch, float yaw, uint32_t nowMs) {
  zeroRoll = roll;
  zeroPitch = pitch;
  zeroYaw = yaw;
  zeroSet = true;
  zeroApplyMs = nowMs;
  resetYawAutoZeroState();
  imu.resetFIFO();

  // Lock the initial target after the next display-coordinate transform.
  // This guarantees the marker starts exactly at the rendered center.
  spawnPitchQ = 0;
  spawnYawQ = 0;
  spawnSet = false;
  lockTargetToCurrentFrame = true;

  lastYaw = 0.0f;
  lastMove = nowMs;
  lastYawSampleMs = nowMs;
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
  if (ENABLE_SERIAL_DEBUG) {
    Serial.print("Zero calibration applied. samples=");
    Serial.print(zeroWarmupSampleCount);
    Serial.print(" rejected=");
    Serial.print(zeroWarmupRejectedCount);
    Serial.print(" yaw_bias_dps=");
    Serial.println(zeroYawBiasRateDps, 5);
  }
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
    resetRawFrameFilter();
    return false;
  }

  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
  if (d.header & DMP_header_bitmap_Quat9) {
    q1 = (float)d.Quat9.Data.Q1 / 1073741824.0f;
    q2 = (float)d.Quat9.Data.Q2 / 1073741824.0f;
    q3 = (float)d.Quat9.Data.Q3 / 1073741824.0f;
    imuQuatAccuracy = d.Quat9.Data.Accuracy;
  } else if (d.header & DMP_header_bitmap_Quat6) {
    q1 = (float)d.Quat6.Data.Q1 / 1073741824.0f;
    q2 = (float)d.Quat6.Data.Q2 / 1073741824.0f;
    q3 = (float)d.Quat6.Data.Q3 / 1073741824.0f;
    imuQuatAccuracy = -1;
  } else {
    return false;
  }

  float sum = q1*q1 + q2*q2 + q3*q3;
  if (sum > 1.0f) sum = 1.0f;
  if (sum < 0.0f) sum = 0.0f;
  float q0 = sqrtf(1.0f - sum);
  imuQuatW = q0;
  imuQuatX = q1;
  imuQuatY = q2;
  imuQuatZ = q3;
  imuQuatValid = true;

  float roll  = atan2f(2.0f * (q0*q1 + q2*q3),
                       1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / PI;

  float pitch = asinf (2.0f * (q0*q2 - q1*q3)) * 180.0f / PI;

  float yaw   = atan2f(2.0f * (q0*q3 + q1*q2),
                       1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / PI;

  if (!acceptRawFrameOrDefer(roll, pitch, yaw)) return false;

  imuRawRollDeg = roll;
  imuRawPitchDeg = pitch;
  imuRawYawDeg = yaw;

  if (!zeroSet) {
    outRoll = roll;
    outPitch = pitch;
    // Store raw yaw for zero calibration. The display sign is applied after zero.
    outYaw = yaw;
    return true;
  }

  roll  -= zeroRoll;
  pitch -= zeroPitch;
  yaw = -deltaAngle180f(imuRawYawDeg, zeroYaw);
  yaw = wrapAngle180f(yaw);

  outRoll  = roll;
  outPitch = pitch;
  outYaw   = wrapAngle180f(yaw);
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
  if (got && zeroSet) {
    uint32_t nowMs = millis();
    updateYawAutoZero(imuRawRollDeg, imuRawPitchDeg, imuRawYawDeg, nowMs);
    yaw = wrapAngle180f(-deltaAngle180f(imuRawYawDeg, zeroYaw));
    yaw = stabilizeYaw(yaw, pitch, nowMs);
  }
  return got;
}
