#pragma once

#include <Arduino.h>
#include <ILI9488.h>
#include "ICM_20948.h"
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "config.h"

extern WiFiServer server;
extern WiFiClient client;
extern WiFiUDP udp;

extern ILI9488 tft;

extern uint32_t lastTcpStatusDrawMs;
extern uint32_t lastTcpPollMs;
extern uint32_t lastBoxDrawMs;
extern uint32_t lastSerialDebugMs;
extern uint32_t lastLoopTickMs;
extern bool     lastTcpConnectedState;
extern bool     lastUdpConnectedState;
extern bool     tcpConnectedCached;
extern uint32_t lastTcpAliveMs;
extern uint32_t lastUdpAliveMs;

extern char lastImuStatusLine[40];
extern char lastTcpStatusLine[40];
extern uint16_t lastImuStatusColor;
extern uint16_t lastTcpStatusColor;
extern float    displayFovXDeg;
extern float    displayFovYDeg;
extern uint32_t tcpPollIntervalMs;
extern int16_t  boxSizePx;
extern uint32_t boxRefreshIntervalMs;
extern int16_t  edgeVisiblePx;
extern bool     boxRenderEnabled;
extern bool     boxDeltaRenderEnabled;

extern int16_t spawnPitchQ;
extern int16_t spawnYawQ;
extern bool    spawnSet;
extern bool    lockTargetToCurrentFrame;
extern int16_t targetBasisPitchQ;
extern int16_t targetBasisYawQ;

extern bool     lastBoxValid;
extern int16_t  lastBoxX;
extern int16_t  lastBoxY;
extern int16_t  lastBoxW;
extern int16_t  lastBoxH;
extern uint16_t lastBoxColor;
extern bool     lastBoxHollow;

extern ICM_20948_I2C imu;

extern bool  zeroSet;
extern float zeroRoll;
extern float zeroPitch;
extern float zeroYaw;
extern uint32_t zeroArmMs;
extern uint32_t lastZeroCountdownDrawMs;
extern uint32_t lastZeroCountdownSec;
extern float zeroWarmupRollSum;
extern float zeroWarmupPitchSum;
extern float zeroWarmupYawSinSum;
extern float zeroWarmupYawCosSum;
extern uint16_t zeroWarmupSampleCount;

extern float lastYaw;
extern unsigned long lastMove;
extern float yawStable;
extern bool  yawStableInit;
