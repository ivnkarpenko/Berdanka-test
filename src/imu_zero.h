#pragma once

#include <Arduino.h>

bool startIMU();
void beginZeroWarmup(uint32_t nowMs);
void resetZeroWarmupSamples();
void addZeroWarmupSample(float roll, float pitch, float yaw, uint32_t nowMs);
bool getZeroWarmupAverage(float &roll, float &pitch, float &yaw);
void applyZeroCalibration(float roll, float pitch, float yaw, uint32_t nowMs);
void updateZeroCountdown(uint32_t nowMs);
bool readAnglesOnce(float &outRoll, float &outPitch, float &outYaw);
bool readLatestAngles(float &roll, float &pitch, float &yaw,
                      uint32_t budgetUs, uint32_t &elapsedUs);
