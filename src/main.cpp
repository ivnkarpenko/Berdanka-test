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

void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(TFT_ROTATION);

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.fillScreen(ILI9488_BLACK);
  drawCrossFull();
  drawZeroCountdown(15);
  updateBox(0, 0, true, false);
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

  wifiConnectAndStartServer();
  drawNetStatus(false, false);
  Serial.println("NET status: WAIT.");

  beginZeroWarmup(millis());
  drawZeroCountdown(15);
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

  if (!zeroSet) {
    addZeroWarmupSample(roll, pitch, yaw, nowMs);
    updateZeroCountdown(nowMs);
    if (nowMs - zeroArmMs >= ZERO_DELAY_MS) {
      float avgRoll, avgPitch, avgYaw;
      if (getZeroWarmupAverage(avgRoll, avgPitch, avgYaw)) {
        roll = avgRoll;
        pitch = avgPitch;
        yaw = avgYaw;
      }
      applyZeroCalibration(roll, pitch, yaw, nowMs);
      roll = 0.0f;
      pitch = 0.0f;
      yaw = 0.0f;
    } else {
      return;
    }
  }

  float tmp = roll; roll = pitch; pitch = tmp;

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
  int16_t pitchRelQ = targetPitchQ - pQ;
  int16_t yawRelQ   = deltaAngle180i(targetYawQ, yQ);

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
