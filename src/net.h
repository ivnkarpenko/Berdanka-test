#pragma once

#include <Arduino.h>
#include <WiFiS3.h>

struct NetState {
  bool tcpConnected;
  bool udpConnected;
};

void wifiConnectAndStartServer();
void processNetCommand(String line, int16_t pQ, int16_t yQ, uint32_t nowMs, WiFiClient* replyClient);
NetState updateNetwork(int16_t pQ, int16_t yQ, uint32_t nowMs,
                       uint32_t &netPollUs, uint32_t &netReadUs);
