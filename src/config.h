#pragma once

#include <Arduino.h>

// ================== WIFI ==================
// Previous network
// constexpr const char* WIFI_SSID = "JetsonAP";
// constexpr const char* WIFI_PASS = "12345678";
// constexpr const char* WIFI_SSID = "GABELLA";
// constexpr const char* WIFI_PASS = "J8f2829a";
// Current network (AP)
constexpr const char* WIFI_SSID = "cisco";
constexpr const char* WIFI_PASS = "cisco1234"; // 8+ chars required for WPA2
constexpr uint16_t SERVER_PORT = 3333;

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

// ================== UI ==================
constexpr float   DEFAULT_FOV_X_DEG = 43.60f;
constexpr float   DEFAULT_FOV_Y_DEG = 33.40f;
constexpr int16_t DEFAULT_BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

constexpr uint8_t STATUS_TEXT_SIZE = 2;

constexpr int16_t STATUS_X      = 10;
constexpr int16_t STATUS_Y_IMU  = 10;
constexpr int16_t STATUS_Y_TCP  = 36;

constexpr uint32_t TCP_STATUS_REFRESH_MS = 200;
constexpr uint32_t DEFAULT_TCP_POLL_MS = 80;
constexpr uint32_t TCP_ALIVE_TIMEOUT_MS = 10000;
constexpr uint32_t UDP_ALIVE_TIMEOUT_MS = 2500;
constexpr uint32_t DEFAULT_BOX_REFRESH_MS = 33;  // ~30 FPS box redraw cap
constexpr uint32_t SERIAL_DEBUG_MS = 2000;       // reduce serial overhead
constexpr bool ENABLE_SERIAL_DEBUG = true;
constexpr bool ENABLE_PACKET_ACK = false;

constexpr int16_t DEFAULT_EDGE_VISIBLE_PX = 5;

// ================== IMU ZERO ==================
constexpr uint32_t ZERO_DELAY_MS = 15000;
constexpr uint32_t ZERO_COUNTDOWN_REFRESH_MS = 200;
constexpr uint32_t ZERO_AVERAGE_WINDOW_MS = 2500;
constexpr uint16_t ZERO_MIN_AVERAGE_SAMPLES = 20;

// Anti-drift yaw
constexpr float yawDriftLimitDeg = 0.05f;
constexpr unsigned long yawHoldTime = 250;

// Freeze yaw near gimbal lock
constexpr float pitchLockThreshold = 80.0f;
