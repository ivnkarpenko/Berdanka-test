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
// Camera calibration target: 640x480 IMX477 1/2" 4:3 with an 8 mm lens.
// The full camera frame is normalized onto the full 480x320 TFT:
// display_x = camera_x * 480/640, display_y = camera_y * 320/480.
constexpr float   DEFAULT_FOV_X_DEG = 43.60f;
constexpr float   DEFAULT_FOV_Y_DEG = 33.40f;
constexpr int16_t DEFAULT_BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

constexpr uint8_t STATUS_TEXT_SIZE = 2;

constexpr int16_t STATUS_X      = 10;
constexpr int16_t STATUS_Y_IMU  = 10;
constexpr int16_t STATUS_Y_TCP  = 36;

constexpr bool ORIENTATION_DEBUG_MODE = false;
constexpr uint32_t ORIENTATION_DEBUG_REFRESH_MS = 25;
constexpr uint32_t ORIENTATION_DEBUG_SERIAL_MS = 200;

// ================== IMU MOUNTING ==================
// Arduino UNO R4 mounting:
// - IMU +Y goes through the display.
// - IMU +X points right across the display.
// - IMU +Z points up.
// Runtime uses DMP GAME_ROTATION_VECTOR / Quat6. After zero calibration,
// horizontal screen yaw uses the zeroed/stabilized yaw from imu_zero.cpp to
// avoid raw Quat6 course drift. The marker's vertical screen offset is driven
// by the device roll component around IMU +X, not by forward/back pitch.
constexpr float DISPLAY_PITCH_SIGN = -1.0f;
constexpr float DISPLAY_YAW_SIGN   = -1.0f;
constexpr float DISPLAY_ROLL_SIGN  = -1.0f;
constexpr float DISPLAY_FILTER_ALPHA = 0.65f;  // 1.0 = no smoothing
constexpr float DISPLAY_DEADBAND_DEG = 0.20f;

constexpr uint32_t TCP_STATUS_REFRESH_MS = 200;
constexpr uint32_t DEFAULT_TCP_POLL_MS = 80;
constexpr uint32_t TCP_ALIVE_TIMEOUT_MS = 10000;
constexpr uint32_t UDP_ALIVE_TIMEOUT_MS = 2500;
constexpr uint32_t DEFAULT_BOX_REFRESH_MS = 33;  // ~30 FPS box redraw cap
constexpr uint32_t IMU_SERIAL_LOG_MS = 100;      // 10 Hz IMU-only log
constexpr uint32_t SERIAL_DEBUG_MS = 2000;       // disabled unless ENABLE_SERIAL_DEBUG=true
constexpr uint32_t IMU_INIT_RETRY_MS = 1000;
constexpr bool ENABLE_IMU_SERIAL_LOG = true;
constexpr bool ENABLE_SERIAL_DEBUG = false;
constexpr bool ENABLE_PACKET_ACK = false;

constexpr int16_t DEFAULT_EDGE_VISIBLE_PX = 5;

// ================== IMU ZERO ==================
constexpr bool IMU_USE_MAGNETIC_YAW = false;
constexpr bool IMU_FALLBACK_TO_GAME_ROTATION = true;
// Reject single-frame DMP quaternion glitches without blocking real movement:
// a large jump is accepted only if the next frame confirms it.
constexpr float IMU_RAW_DEGLITCH_STEP_DEG = 5.0f;
constexpr uint32_t ZERO_DELAY_MS = 0;
constexpr uint32_t ZERO_COUNTDOWN_REFRESH_MS = 200;
constexpr uint32_t ZERO_SENSOR_SETTLE_MS = 1000;
// Quat6 yaw/tilt can walk several degrees while the DMP converges after boot.
// Require a real stationary window before capturing zero, otherwise the target
// visibly drifts while the filter settles.
constexpr uint32_t ZERO_STABLE_WINDOW_MS = 2500;
constexpr uint16_t ZERO_MIN_AVERAGE_SAMPLES = 20;
constexpr float ZERO_STABLE_MAX_ROLL_STEP_DEG = 0.45f;
constexpr float ZERO_STABLE_MAX_PITCH_STEP_DEG = 0.45f;
constexpr float ZERO_STABLE_MAX_YAW_STEP_DEG = 0.45f;
constexpr float ZERO_STABLE_MAX_ROLL_SPAN_DEG = 0.60f;
constexpr float ZERO_STABLE_MAX_PITCH_SPAN_DEG = 0.60f;
constexpr float ZERO_STABLE_MAX_YAW_SPAN_DEG = 0.60f;

// Quat6 has no absolute yaw reference. While the device is still, slowly
// retarget the local zero to the current raw yaw so gyro bias cannot walk the
// marker sideways after startup.
constexpr bool YAW_AUTO_ZERO_ENABLE = true;
constexpr uint32_t YAW_AUTO_ZERO_START_DELAY_MS = 300;
constexpr uint32_t YAW_AUTO_ZERO_STILL_WINDOW_MS = 650;
constexpr float YAW_AUTO_ZERO_MAX_ROLL_STEP_DEG = 0.35f;
constexpr float YAW_AUTO_ZERO_MAX_PITCH_STEP_DEG = 0.35f;
constexpr float YAW_AUTO_ZERO_MAX_YAW_STEP_DEG = 0.55f;
constexpr float YAW_AUTO_ZERO_MAX_ROLL_SPAN_DEG = 0.70f;
constexpr float YAW_AUTO_ZERO_MAX_PITCH_SPAN_DEG = 0.70f;
constexpr float YAW_AUTO_ZERO_MAX_YAW_SPAN_DEG = 1.60f;
constexpr float YAW_AUTO_ZERO_MAX_ERROR_DEG = 8.0f;
constexpr float YAW_AUTO_ZERO_DEADBAND_DEG = 0.05f;
constexpr float YAW_AUTO_ZERO_GAIN = 0.12f;

// Anti-drift yaw
constexpr float yawDriftLimitDeg = 0.05f;
constexpr float yawDriftLimitDps = 1.0f;
constexpr unsigned long yawHoldTime = 250;

// Freeze yaw near gimbal lock
constexpr float pitchLockThreshold = 80.0f;
