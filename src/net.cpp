#include "net.h"

#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include "angle_utils.h"
#include "app_state.h"
#include "marker.h"
#include "ui.h"

static char tcpRxBuf[257];
static uint16_t tcpRxLen = 0;
static bool tcpRxOverflow = false;

static void resetTcpRxBuffer() {
  tcpRxLen = 0;
  tcpRxOverflow = false;
}

static bool readTcpLine(WiFiClient& c, String& line) {
  while (c.available()) {
    char ch = (char)c.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (tcpRxOverflow) {
        resetTcpRxBuffer();
        line = "";
        return false;
      }
      tcpRxBuf[tcpRxLen] = '\0';
      line = String(tcpRxBuf);
      resetTcpRxBuffer();
      return true;
    }

    if (tcpRxLen < sizeof(tcpRxBuf) - 1) {
      tcpRxBuf[tcpRxLen++] = ch;
    } else {
      tcpRxOverflow = true;
    }
  }
  return false;
}

static bool isCenterCommand(const String& s) {
  return s == "CMD:CENTER";
}

static bool isPingCommand(const String& s) {
  return s == "PING";
}

static bool parseMsgOnlyCommand(const String& s, String& msg) {
  const String prefix = "MSGONLY:";
  if (!s.startsWith(prefix)) return false;

  msg = s.substring(prefix.length());
  msg.trim();
  return msg.length() > 0;
}

static bool parsePositiveFloatTail(String tail, float& value) {
  tail.trim();
  if (tail.length() == 0) return false;

  bool hasDigit = false;
  bool hasDot = false;
  for (size_t i = 0; i < tail.length(); i++) {
    char c = tail[i];
    if (isDigit(c)) {
      hasDigit = true;
      continue;
    }
    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }
    return false;
  }

  if (!hasDigit) return false;
  value = tail.toFloat();
  return value > 0.0f;
}

static bool parseDisplayFovXCommand(const String& s, float& fovXDeg) {
  const String prefix = "CFG:FOV_X:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;
  if (value < 5.0f || value > 180.0f) return false;

  fovXDeg = value;
  return true;
}

static bool parseDisplayFovYCommand(const String& s, float& fovYDeg) {
  const String prefix = "CFG:FOV_Y:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;
  if (value < 5.0f || value > 180.0f) return false;

  fovYDeg = value;
  return true;
}

static bool parseDisplayDegPerPxCommand(const String& s, float& degPerPx) {
  const String prefix = "CFG:DEG_PER_PX:";
  if (!s.startsWith(prefix)) return false;

  float value = 0.0f;
  if (!parsePositiveFloatTail(s.substring(prefix.length()), value)) return false;

  degPerPx = value;
  return true;
}

static bool parseTcpPollMsCommand(const String& s, uint32_t& tcpPollMs) {
  const String prefix = "CFG:TCP_POLL_MS:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;

  for (size_t i = 0; i < tail.length(); i++) {
    char c = tail[i];
    if (!isDigit(c)) return false;
  }

  uint32_t value = (uint32_t)tail.toInt();
  if (value < 20 || value > 2000) return false;
  tcpPollMs = value;
  return true;
}

static bool parseBoxSizeCommand(const String& s, int16_t& sizePx) {
  const String prefix = "CFG:BOX_SIZE:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;
  for (size_t i = 0; i < tail.length(); i++) {
    if (!isDigit(tail[i])) return false;
  }

  int value = tail.toInt();
  if (value < 8 || value > 200) return false;
  sizePx = (int16_t)value;
  return true;
}

static bool parseBoxRefreshMsCommand(const String& s, uint32_t& refreshMs) {
  const String prefix = "CFG:BOX_REFRESH_MS:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail.length() == 0) return false;
  for (size_t i = 0; i < tail.length(); i++) {
    if (!isDigit(tail[i])) return false;
  }

  uint32_t value = (uint32_t)tail.toInt();
  if (value < 10 || value > 1000) return false;
  refreshMs = value;
  return true;
}

static bool parseBoxRenderCommand(const String& s, bool& enabled) {
  const String prefix = "CFG:BOX_RENDER:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail == "1") {
    enabled = true;
    return true;
  }
  if (tail == "0") {
    enabled = false;
    return true;
  }
  return false;
}

static bool parseBoxDeltaRenderCommand(const String& s, bool& enabled) {
  const String prefix = "CFG:BOX_DELTA_RENDER:";
  if (!s.startsWith(prefix)) return false;

  String tail = s.substring(prefix.length());
  tail.trim();
  if (tail == "1") {
    enabled = true;
    return true;
  }
  if (tail == "0") {
    enabled = false;
    return true;
  }
  return false;
}

static bool parseSignedFloat(String s, float& value) {
  s.trim();
  if (s.length() == 0) return false;

  bool hasDigit = false;
  bool hasDot = false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isDigit(c)) {
      hasDigit = true;
      continue;
    }
    if ((c == '-' || c == '+') && i == 0) continue;
    if (c == '.' && !hasDot) {
      hasDot = true;
      continue;
    }
    return false;
  }

  if (!hasDigit) return false;
  value = s.toFloat();
  return true;
}

static bool parsePacket(const String& s, String& msg, float& x, float& y) {
  int iMsg = s.indexOf("MSG:");
  int iX   = s.indexOf(";X:");
  int iY   = s.indexOf(";Y:");
  if (iMsg != 0 || iX < 0 || iY < 0 || iY <= iX) return false;

  msg = s.substring(4, iX);
  String sx = s.substring(iX + 3, iY);
  String sy = s.substring(iY + 3);

  return parseSignedFloat(sx, x) && parseSignedFloat(sy, y);
}

void processNetCommand(String line, int16_t pQ, int16_t yQ, uint32_t nowMs, WiFiClient* replyClient) {
  line.trim();
  if (line.length() == 0) return;

  float newDegPerPx = 0.0f;
  float newFovXDeg = 0.0f;
  float newFovYDeg = 0.0f;
  uint32_t newTcpPollMs = 0;
  int16_t newBoxSizePx = 0;
  uint32_t newBoxRefreshMs = 0;
  String msgOnly;
  bool newBoxRenderEnabled = false;
  bool newBoxDeltaRenderEnabled = false;
  bool fromTcp = (replyClient != nullptr);

  if (isPingCommand(line)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
  } else if (parseMsgOnlyCommand(line, msgOnly)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    char msgBuf[48];
    msgOnly.toCharArray(msgBuf, sizeof(msgBuf));
    msgBuf[30] = '\0';
    Serial.print("Msg updated: ");
    Serial.println(msgBuf);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->print("ACK;MSGONLY:");
      replyClient->println(msgBuf);
    }
  } else if (parseDisplayFovXCommand(line, newFovXDeg)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    displayFovXDeg = newFovXDeg;
    eraseOldBox();
    Serial.print("Display FOV X updated: ");
    Serial.println(displayFovXDeg, 2);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->print("ACK;CFG:FOV_X:");
      replyClient->println(displayFovXDeg, 2);
    }
  } else if (parseDisplayFovYCommand(line, newFovYDeg)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    displayFovYDeg = newFovYDeg;
    eraseOldBox();
    Serial.print("Display FOV Y updated: ");
    Serial.println(displayFovYDeg, 2);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->print("ACK;CFG:FOV_Y:");
      replyClient->println(displayFovYDeg, 2);
    }
  } else if (parseTcpPollMsCommand(line, newTcpPollMs)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    tcpPollIntervalMs = newTcpPollMs;
    Serial.print("TCP poll interval updated: ");
    Serial.println(tcpPollIntervalMs);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->print("ACK;CFG:TCP_POLL_MS:");
      replyClient->println(tcpPollIntervalMs);
    }
  } else if (parseBoxSizeCommand(line, newBoxSizePx)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    boxSizePx = newBoxSizePx;
    eraseOldBox();
    Serial.print("Box size updated: ");
    Serial.println(boxSizePx);
  } else if (parseBoxRefreshMsCommand(line, newBoxRefreshMs)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    boxRefreshIntervalMs = newBoxRefreshMs;
    Serial.print("Box refresh ms updated: ");
    Serial.println(boxRefreshIntervalMs);
  } else if (parseBoxRenderCommand(line, newBoxRenderEnabled)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    boxRenderEnabled = newBoxRenderEnabled;
    if (!boxRenderEnabled) eraseOldBox();
    Serial.print("Box render updated: ");
    Serial.println(boxRenderEnabled ? "ON" : "OFF");
  } else if (parseBoxDeltaRenderCommand(line, newBoxDeltaRenderEnabled)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    boxDeltaRenderEnabled = newBoxDeltaRenderEnabled;
    Serial.print("Box delta render updated: ");
    Serial.println(boxDeltaRenderEnabled ? "ON" : "OFF");
  } else if (parseDisplayDegPerPxCommand(line, newDegPerPx)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    displayFovXDeg = newDegPerPx * (float)SCREEN_W;
    displayFovYDeg = newDegPerPx * (float)SCREEN_H;
    eraseOldBox();
    Serial.print("Legacy display deg/px converted to FOV X/Y: ");
    Serial.print(displayFovXDeg, 2);
    Serial.print("/");
    Serial.println(displayFovYDeg, 2);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->print("ACK;CFG:DEG_PER_PX:");
      replyClient->println(newDegPerPx, 4);
    }
  } else if (isCenterCommand(line)) {
    if (fromTcp) lastTcpAliveMs = nowMs;
    else lastUdpAliveMs = nowMs;
    spawnPitchQ = pQ;
    spawnYawQ   = yQ;
    spawnSet    = true;
    Serial.print("Center command applied: pitch=");
    Serial.print(spawnPitchQ);
    Serial.print(" yaw=");
    Serial.println(spawnYawQ);
    if (ENABLE_PACKET_ACK && replyClient) {
      replyClient->println("ACK;CMD:CENTER");
    }
  } else {
    String msg;
    float x = 0.0f, y = 0.0f;

    if (parsePacket(line, msg, x, y)) {
      if (fromTcp) lastTcpAliveMs = nowMs;
      else lastUdpAliveMs = nowMs;
      // Target packet X/Y are offsets from the current device direction.
      // X=0/Y=0 means the marker is centered at the current pose.
      spawnPitchQ = pQ + quantizeDeg2((float)x);
      spawnYawQ   = wrapAngle180i(yQ + quantizeDeg2((float)y));
      spawnSet    = true;
      if (ENABLE_PACKET_ACK && replyClient) {
        replyClient->print("ACK;MSG:");
        replyClient->print(msg);
        replyClient->print(";X:");
        replyClient->print(x, 2);
        replyClient->print(";Y:");
        replyClient->println(y, 2);
      }
    } else if (replyClient) {
      replyClient->print("ERR;BAD_PACKET;");
      replyClient->println(line);
    }
  }
}

void wifiConnectAndStartServer() {
  Serial.print("Starting AP: ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;
  while (status != WL_AP_LISTENING && status != WL_AP_CONNECTED) {
    status = WiFi.beginAP(WIFI_SSID, WIFI_PASS);
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("AP started.");

  server.begin();
  Serial.print("TCP server started on port ");
  Serial.println(SERVER_PORT);

  udp.begin(SERVER_PORT);
  Serial.print("UDP server started on port ");
  Serial.println(SERVER_PORT);
}

NetState updateNetwork(int16_t pQ, int16_t yQ, uint32_t nowMs,
                       uint32_t &netPollUs, uint32_t &netReadUs) {
  if (nowMs - lastTcpPollMs >= tcpPollIntervalMs) {
    uint32_t tcpPollStartUs = micros();
    lastTcpPollMs = nowMs;

    bool connectedNow = (client && client.connected());
    if (!connectedNow) {
      WiFiClient newClient = server.available();
      if (newClient) {
        client = newClient;
        client.setTimeout(5);
        resetTcpRxBuffer();
        client.println("HELLO from UNO R4 WiFi");
        Serial.println("Client connected.");
        lastTcpAliveMs = nowMs;
        connectedNow = true;
      }
    }
    if (connectedNow && lastTcpAliveMs != 0 && (nowMs - lastTcpAliveMs > TCP_ALIVE_TIMEOUT_MS)) {
      client.stop();
      resetTcpRxBuffer();
      connectedNow = false;
      Serial.println("TCP heartbeat timeout.");
    }
    tcpConnectedCached = connectedNow;
    netPollUs = micros() - tcpPollStartUs;
  }

  if (tcpConnectedCached && client.available()) {
    uint32_t tcpReadStartUs = micros();
    while (client.available()) {
      String line;
      if (!readTcpLine(client, line)) break;
      processNetCommand(line, pQ, yQ, nowMs, &client);
    }
    netReadUs = micros() - tcpReadStartUs;
  }

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    uint32_t udpReadStartUs = micros();
    char packet[257];
    int len = udp.read(packet, sizeof(packet) - 1);
    if (len > 0) {
      packet[len] = '\0';
      processNetCommand(String(packet), pQ, yQ, nowMs, nullptr);
    }
    netReadUs += micros() - udpReadStartUs;
  }

  NetState state;
  state.tcpConnected = tcpConnectedCached;
  state.udpConnected = (lastUdpAliveMs != 0 && (nowMs - lastUdpAliveMs <= UDP_ALIVE_TIMEOUT_MS));

  if (state.tcpConnected != lastTcpConnectedState || state.udpConnected != lastUdpConnectedState) {
    Serial.print("NET state -> ");
    if (state.tcpConnected) Serial.println("TCP OK");
    else if (state.udpConnected) Serial.println("UDP OK");
    else Serial.println("WAIT");
    lastTcpConnectedState = state.tcpConnected;
    lastUdpConnectedState = state.udpConnected;
  }

  if (millis() - lastTcpStatusDrawMs >= TCP_STATUS_REFRESH_MS) {
    drawNetStatus(state.tcpConnected, state.udpConnected);
    lastTcpStatusDrawMs = millis();
  }

  return state;
}
