#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ILI9488.h>

#include "angle_utils.h"
#include "app_state.h"
#include "config.h"
#include "imu_zero.h"
#include "marker.h"
#include "net.h"
#include "ui.h"

static bool quatVectorRefSet = false;
static float quatVectorRefR[9];

static bool getQuaternionVectorAngles(float &azDeg, float &elDeg, float &cantDeg);
static void resetQuaternionVectorReference();
static void projectTargetToScreen(float targetPitchDeg, float targetYawDeg,
                                  float currentPitchDeg, float currentYawDeg,
                                  float &pitchRelDeg, float &yawRelDeg);

static void mapImuToDisplayAngles(float imuRollDeg, float imuPitchDeg, float imuYawDeg,
                                  float &displayRollDeg, float &displayPitchDeg,
                                  float &displayYawDeg) {
  (void)imuRollDeg;
  (void)imuPitchDeg;

  float vectorAzDeg = 0.0f;
  float vectorElDeg = 0.0f;
  float vectorCantDeg = 0.0f;
  if (getQuaternionVectorAngles(vectorAzDeg, vectorElDeg, vectorCantDeg)) {
    displayPitchDeg = DISPLAY_PITCH_SIGN * vectorAzDeg;
    displayYawDeg   = zeroSet ? imuYawDeg : (DISPLAY_YAW_SIGN * vectorCantDeg);
    displayRollDeg  = DISPLAY_ROLL_SIGN  * vectorElDeg;
  } else {
    displayPitchDeg = 0.0f;
    displayYawDeg = 0.0f;
    displayRollDeg = 0.0f;
  }
}

static void quatToMatrix(float w, float x, float y, float z, float r[9]) {
  float n = sqrtf(w*w + x*x + y*y + z*z);
  if (n <= 0.0f) {
    w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
  } else {
    float inv = 1.0f / n;
    w *= inv; x *= inv; y *= inv; z *= inv;
  }

  r[0] = 1.0f - 2.0f * (y*y + z*z);
  r[1] = 2.0f * (x*y - w*z);
  r[2] = 2.0f * (x*z + w*y);

  r[3] = 2.0f * (x*y + w*z);
  r[4] = 1.0f - 2.0f * (x*x + z*z);
  r[5] = 2.0f * (y*z - w*x);

  r[6] = 2.0f * (x*z - w*y);
  r[7] = 2.0f * (y*z + w*x);
  r[8] = 1.0f - 2.0f * (x*x + y*y);
}

static float dot3(float ax, float ay, float az, float bx, float by, float bz) {
  return ax*bx + ay*by + az*bz;
}

static bool getQuaternionVectorAngles(float &azDeg, float &elDeg, float &cantDeg) {
  if (!imuQuatValid) return false;

  float curR[9];
  quatToMatrix(imuQuatW, imuQuatX, imuQuatY, imuQuatZ, curR);

  if (!quatVectorRefSet) {
    for (uint8_t i = 0; i < 9; i++) quatVectorRefR[i] = curR[i];
    quatVectorRefSet = true;
  }

  float rel[9];
  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      rel[row * 3 + col] =
        quatVectorRefR[row]     * curR[col] +
        quatVectorRefR[3 + row] * curR[3 + col] +
        quatVectorRefR[6 + row] * curR[6 + col];
    }
  }

  float courseRad = atan2f(rel[3], rel[0]);
  float c = cosf(courseRad);
  float s = sinf(courseRad);

  float zNoYawX =  c * rel[2] + s * rel[5];
  float zNoYawY = -s * rel[2] + c * rel[5];
  float zNoYawZ =  rel[8];

  azDeg = atan2f(zNoYawY, zNoYawZ) * 180.0f / PI;
  elDeg = atan2f(zNoYawX, sqrtf(zNoYawY*zNoYawY + zNoYawZ*zNoYawZ)) * 180.0f / PI;
  cantDeg = courseRad * 180.0f / PI;
  return true;
}

static void resetQuaternionVectorReference() {
  quatVectorRefSet = false;
}

static float applyDeadband(float valueDeg) {
  return (fabsf(valueDeg) < DISPLAY_DEADBAND_DEG) ? 0.0f : valueDeg;
}

static void filterDisplayAngles(float &rollDeg, float &pitchDeg, float &yawDeg, bool resetFilter) {
  static bool initialized = false;
  static float filteredRollDeg = 0.0f;
  static float filteredPitchDeg = 0.0f;
  static float filteredYawDeg = 0.0f;

  float alpha = DISPLAY_FILTER_ALPHA;
  if (alpha < 0.05f) alpha = 0.05f;
  if (alpha > 1.0f) alpha = 1.0f;

  if (!initialized || resetFilter) {
    filteredRollDeg = rollDeg;
    filteredPitchDeg = pitchDeg;
    filteredYawDeg = yawDeg;
    initialized = true;
  } else {
    filteredRollDeg += alpha * (rollDeg - filteredRollDeg);
    filteredPitchDeg += alpha * (pitchDeg - filteredPitchDeg);
    filteredYawDeg = wrapAngle180f(filteredYawDeg + alpha * deltaAngle180f(yawDeg, filteredYawDeg));
  }

  rollDeg = applyDeadband(filteredRollDeg);
  pitchDeg = applyDeadband(filteredPitchDeg);
  yawDeg = applyDeadband(filteredYawDeg);
}

static void vectorFromYawPitch(float yawDeg, float pitchDeg, float &x, float &y, float &z) {
  float yawRad = yawDeg * PI / 180.0f;
  float pitchRad = pitchDeg * PI / 180.0f;
  float cp = cosf(pitchRad);
  x = cp * sinf(yawRad);
  y = sinf(pitchRad);
  z = cp * cosf(yawRad);
}

static void currentBasisFromYawPitch(float yawDeg, float pitchDeg,
                                     float right[3], float up[3], float forward[3]) {
  vectorFromYawPitch(yawDeg, pitchDeg, forward[0], forward[1], forward[2]);

  float yawRad = yawDeg * PI / 180.0f;
  right[0] = cosf(yawRad);
  right[1] = 0.0f;
  right[2] = -sinf(yawRad);

  up[0] = forward[1] * right[2] - forward[2] * right[1];
  up[1] = forward[2] * right[0] - forward[0] * right[2];
  up[2] = forward[0] * right[1] - forward[1] * right[0];
}

static void projectTargetToScreen(float targetPitchDeg, float targetYawDeg,
                                  float currentPitchDeg, float currentYawDeg,
                                  float &pitchRelDeg, float &yawRelDeg) {
  float target[3];
  vectorFromYawPitch(targetYawDeg, targetPitchDeg, target[0], target[1], target[2]);

  float right[3];
  float up[3];
  float forward[3];
  currentBasisFromYawPitch(currentYawDeg, currentPitchDeg, right, up, forward);

  float tx = dot3(target[0], target[1], target[2], right[0], right[1], right[2]);
  float ty = dot3(target[0], target[1], target[2], up[0], up[1], up[2]);
  float tz = dot3(target[0], target[1], target[2], forward[0], forward[1], forward[2]);

  yawRelDeg = atan2f(tx, tz) * 180.0f / PI;
  pitchRelDeg = atan2f(ty, sqrtf(tx * tx + tz * tz)) * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  tft.begin();
  tft.setRotation(TFT_ROTATION);

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.fillScreen(ILI9488_BLACK);
  drawCrossFull();
  drawZeroCountdown((ZERO_DELAY_MS + 999U) / 1000U);
  updateBox(0, 0, true, false);
  if (ENABLE_SERIAL_DEBUG) Serial.println("Target persistence disabled.");

  Wire.begin();
  Wire.setClock(400000);

  drawIMUStatus("IMU INIT...", ILI9488_YELLOW);
  if (ENABLE_SERIAL_DEBUG) Serial.println("IMU init started.");

  while (!startIMU()) {
    drawIMUStatus("IMU INIT FAIL", ILI9488_RED);
    if (ENABLE_SERIAL_DEBUG) Serial.println("IMU init fail, retry...");
    delay(IMU_INIT_RETRY_MS);
  }

  drawIMUStatus("IMU OK", ILI9488_GREEN);
  if (ENABLE_SERIAL_DEBUG) Serial.println("IMU OK.");

  if (ORIENTATION_DEBUG_MODE) {
    drawOrientationBars(0.0f, 0.0f, 0.0f, true);
    Serial.println("ORIENTATION DEBUG MODE: QUAT VECTOR REL, no Euler yaw for screen");
    return;
  }

  wifiConnectAndStartServer();
  drawNetStatus(false, false);
  if (ENABLE_SERIAL_DEBUG) Serial.println("NET status: WAIT.");

  beginZeroWarmup(millis());
  drawZeroCountdown((ZERO_DELAY_MS + 999U) / 1000U);
  updateBox(0, 0, true, false);
  delay(150);
}

void loop() {
  uint32_t loopStartUs = micros();
  uint32_t nowMs = millis();
  uint32_t loopDtMs = (lastLoopTickMs == 0) ? 0 : (nowMs - lastLoopTickMs);
  lastLoopTickMs = nowMs;
  uint32_t netPollUs = 0;
  uint32_t netReadUs = 0;
  uint32_t boxDrawUs = 0;

  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  uint32_t imuReadUs = 0;
  bool got = readLatestAngles(roll, pitch, yaw, 2500, imuReadUs);
  if (!got) {
    if (!zeroSet) updateZeroCountdown(nowMs);
    return;
  }

  float rawRollDeg = imuRawRollDeg;
  float rawPitchDeg = imuRawPitchDeg;
  float rawYawDeg = imuRawYawDeg;

  if (ORIENTATION_DEBUG_MODE) {
    static uint32_t lastOrientationDrawMs = 0;
    static uint32_t lastOrientationSerialMs = 0;
    if (nowMs - lastOrientationDrawMs >= ORIENTATION_DEBUG_REFRESH_MS) {
      float vectorAzDeg = 0.0f;
      float vectorElDeg = 0.0f;
      float vectorCantDeg = 0.0f;
      bool vectorOk = getQuaternionVectorAngles(vectorAzDeg, vectorElDeg, vectorCantDeg);
      if (vectorOk) {
        drawOrientationBars(vectorAzDeg, vectorElDeg, vectorCantDeg, false);
      }
      if (nowMs - lastOrientationSerialMs >= ORIENTATION_DEBUG_SERIAL_MS) {
        Serial.print("ANGLES,yaw=");
        Serial.print(rawYawDeg, 2);
        Serial.print(",pitch=");
        Serial.print(rawPitchDeg, 2);
        Serial.print(",roll=");
        Serial.print(rawRollDeg, 2);
        Serial.print(",quat_acc=");
        Serial.print(imuQuatAccuracy);
        if (vectorOk) {
          Serial.print(" | VECTOR,az=");
          Serial.print(vectorAzDeg, 2);
          Serial.print(",el=");
          Serial.print(vectorElDeg, 2);
          Serial.print(",cant=");
          Serial.print(vectorCantDeg, 2);
        }
        Serial.println();
        lastOrientationSerialMs = nowMs;
      }
      lastOrientationDrawMs = nowMs;
    }
    return;
  }

  if (!zeroSet) {
    addZeroWarmupSample(roll, pitch, yaw, nowMs);
    updateZeroCountdown(nowMs);

    float warmupRollDeg = 0.0f;
    float warmupPitchDeg = 0.0f;
    float warmupYawDeg = 0.0f;
    mapImuToDisplayAngles(roll, pitch, yaw, warmupRollDeg, warmupPitchDeg, warmupYawDeg);
    int16_t warmupPitchQ = quantizeDeg2(-warmupPitchDeg);
    int16_t warmupYawQ = wrapAngle180i(quantizeDeg2(-warmupYawDeg));
    if (boxRenderEnabled && nowMs - lastBoxDrawMs >= boxRefreshIntervalMs) {
      updateBox(warmupPitchQ, warmupYawQ, false, false);
      lastBoxDrawMs = nowMs;
    }

    if (ENABLE_IMU_SERIAL_LOG && (millis() - lastSerialDebugMs >= IMU_SERIAL_LOG_MS)) {
      bool zeroReady = isZeroWarmupReady(nowMs);
      uint32_t zeroStableMs = getZeroWarmupStableMs(nowMs);
      Serial.print("IMU,ms=");
      Serial.print(nowMs);
      Serial.print(",state=warmup,zero=0,raw_roll=");
      Serial.print(rawRollDeg, 3);
      Serial.print(",raw_pitch=");
      Serial.print(rawPitchDeg, 3);
      Serial.print(",raw_yaw=");
      Serial.print(rawYawDeg, 3);
      Serial.print(",zero_samples=");
      Serial.print(zeroWarmupSampleCount);
      Serial.print(",zero_rej=");
      Serial.print(zeroWarmupRejectedCount);
      Serial.print(",zero_ready=");
      Serial.print(zeroReady ? 1 : 0);
      Serial.print(",zero_stable_ms=");
      Serial.print(zeroStableMs);
      Serial.print(",yaw_auto_zero=");
      Serial.print(yawAutoZeroActive ? 1 : 0);
      Serial.print(",yaw_still_ms=");
      Serial.print(yawAutoZeroStableMs);
      Serial.print(",yaw_auto_zero_deg=");
      Serial.print(yawAutoZeroCorrectionDeg, 3);
      Serial.print(",yaw_auto_zero_updates=");
      Serial.print(yawAutoZeroUpdates);
      Serial.print(",yaw_bias_dps=");
      Serial.print(zeroYawBiasRateDps, 6);
      Serial.print(",quat_acc=");
      Serial.println(imuQuatAccuracy);
      lastSerialDebugMs = millis();
    }
    if (nowMs - zeroArmMs >= ZERO_DELAY_MS) {
      float zeroRollAvg = 0.0f;
      float zeroPitchAvg = 0.0f;
      float zeroYawAvg = 0.0f;
      if (!getZeroWarmupAverage(zeroRollAvg, zeroPitchAvg, zeroYawAvg)) {
        uint32_t stableMs = getZeroWarmupStableMs(nowMs);
        bool stillSettling = ((nowMs - zeroArmMs) < ZERO_SENSOR_SETTLE_MS) ||
                             (stableMs < ZERO_STABLE_WINDOW_MS);
        drawIMUStatus(stillSettling ? "IMU SETTLE" : "KEEP STILL", ILI9488_YELLOW);
        return;
      }

      resetQuaternionVectorReference();
      applyZeroCalibration(zeroRollAvg, zeroPitchAvg, zeroYawAvg, nowMs);
      roll = 0.0f;
      pitch = 0.0f;
      yaw = 0.0f;
      targetBasisPitchQ = 0;
      targetBasisYawQ = 0;
      spawnPitchQ = targetBasisPitchQ;
      spawnYawQ = targetBasisYawQ;
      spawnSet = true;
    } else {
      return;
    }
  }

  float displayRollDeg = 0.0f;
  float displayPitchDeg = 0.0f;
  float displayYawDeg = 0.0f;
  mapImuToDisplayAngles(roll, pitch, yaw, displayRollDeg, displayPitchDeg, displayYawDeg);
  roll = displayRollDeg;
  pitch = displayPitchDeg;
  yaw = displayYawDeg;
  filterDisplayAngles(roll, pitch, yaw, lockTargetToCurrentFrame);

  int16_t pQ = quantizeDeg2(pitch);
  int16_t yQ = wrapAngle180i(quantizeDeg2(yaw));

  if (lockTargetToCurrentFrame) {
    spawnPitchQ = pQ;
    spawnYawQ = yQ;
    spawnSet = true;
    lockTargetToCurrentFrame = false;
  }

  NetState netState = updateNetwork(pQ, yQ, nowMs, netPollUs, netReadUs);

  int16_t targetPitchQ = spawnSet ? spawnPitchQ : pQ;
  int16_t targetYawQ   = spawnSet ? spawnYawQ : yQ;
  float pitchRelDeg = 0.0f;
  float yawRelDeg = 0.0f;
  projectTargetToScreen((float)targetPitchQ, (float)targetYawQ, pitch, yaw,
                        pitchRelDeg, yawRelDeg);
  int16_t pitchRelQ = quantizeDeg2(pitchRelDeg);
  int16_t yawRelQ = wrapAngle180i(quantizeDeg2(yawRelDeg));

  bool onTarget = (abs((int)pitchRelQ) <= TARGET_TOL_DEG) &&
                  (abs((int)yawRelQ)   <= TARGET_TOL_DEG);

  if (!boxRenderEnabled) {
    eraseOldBox();
  } else if (nowMs - lastBoxDrawMs >= boxRefreshIntervalMs) {
    uint32_t boxDrawStartUs = micros();
    updateBox(pitchRelQ, yawRelQ, onTarget, netState.tcpConnected || netState.udpConnected);
    lastBoxDrawMs = nowMs;
    boxDrawUs = micros() - boxDrawStartUs;
  }

  uint32_t loopUs = micros() - loopStartUs;

  if (ENABLE_IMU_SERIAL_LOG && (millis() - lastSerialDebugMs >= IMU_SERIAL_LOG_MS)) {
    Serial.print("IMU,ms=");
    Serial.print(nowMs);
    Serial.print(",state=run,zero=1,raw_roll=");
    Serial.print(rawRollDeg, 3);
    Serial.print(",raw_pitch=");
    Serial.print(rawPitchDeg, 3);
    Serial.print(",raw_yaw=");
    Serial.print(rawYawDeg, 3);
    Serial.print(",pitch=");
    Serial.print(displayPitchDeg, 3);
    Serial.print(",yaw=");
    Serial.print(displayYawDeg, 3);
    Serial.print(",pQ=");
    Serial.print(pQ);
    Serial.print(",yQ=");
    Serial.print(yQ);
    Serial.print(",zero_samples=");
    Serial.print(zeroWarmupSampleCount);
    Serial.print(",zero_rej=");
    Serial.print(zeroWarmupRejectedCount);
    Serial.print(",zero_stable_ms=");
    Serial.print(getZeroWarmupStableMs(nowMs));
    Serial.print(",yaw_auto_zero=");
    Serial.print(yawAutoZeroActive ? 1 : 0);
    Serial.print(",yaw_still_ms=");
    Serial.print(yawAutoZeroStableMs);
    Serial.print(",yaw_auto_zero_deg=");
    Serial.print(yawAutoZeroCorrectionDeg, 3);
    Serial.print(",yaw_auto_zero_updates=");
    Serial.print(yawAutoZeroUpdates);
    Serial.print(",yaw_bias_dps=");
    Serial.print(zeroYawBiasRateDps, 6);
    Serial.print(",quat_acc=");
    Serial.println(imuQuatAccuracy);
    lastSerialDebugMs = millis();
  }

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
    Serial.print(" zero_samples=");
    Serial.print(zeroWarmupSampleCount);
    Serial.print(" zero_rej=");
    Serial.print(zeroWarmupRejectedCount);
    Serial.print(" yaw_bias_dps=");
    Serial.print(zeroYawBiasRateDps, 5);
    Serial.print(" yaw_auto_zero=");
    Serial.print(yawAutoZeroActive ? 1 : 0);
    Serial.print(" yaw_auto_zero_deg=");
    Serial.print(yawAutoZeroCorrectionDeg, 3);
    Serial.print(" net=");
    if (netState.tcpConnected) Serial.print("TCP");
    else if (netState.udpConnected) Serial.print("UDP");
    else Serial.print("WAIT");
    Serial.print(" yaw=");
    Serial.print(yQ);
    Serial.print(" targetYaw=");
    Serial.print(targetYawQ);
    Serial.print(" yawRel=");
    Serial.println(yawRelQ);
    lastSerialDebugMs = millis();
  }
}
