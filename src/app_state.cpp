#include "app_state.h"

WiFiServer server(SERVER_PORT);
WiFiClient client;
WiFiUDP udp;

ILI9488 tft(TFT_CS, TFT_DC, TFT_RST);

uint32_t lastTcpStatusDrawMs = 0;
uint32_t lastTcpPollMs = 0;
uint32_t lastBoxDrawMs = 0;
uint32_t lastSerialDebugMs = 0;
uint32_t lastLoopTickMs = 0;
bool     lastTcpConnectedState = false;
bool     lastUdpConnectedState = false;
bool     tcpConnectedCached = false;
uint32_t lastTcpAliveMs = 0;
uint32_t lastUdpAliveMs = 0;

char lastImuStatusLine[40] = "";
char lastTcpStatusLine[40] = "";
uint16_t lastImuStatusColor = 0xFFFF;
uint16_t lastTcpStatusColor = 0xFFFF;
float    displayFovXDeg = DEFAULT_FOV_X_DEG;
float    displayFovYDeg = DEFAULT_FOV_Y_DEG;
uint32_t tcpPollIntervalMs = DEFAULT_TCP_POLL_MS;
int16_t  boxSizePx = DEFAULT_BOX_SIZE;
uint32_t boxRefreshIntervalMs = DEFAULT_BOX_REFRESH_MS;
int16_t  edgeVisiblePx = DEFAULT_EDGE_VISIBLE_PX;
bool     boxRenderEnabled = true;
bool     boxDeltaRenderEnabled = true;

int16_t spawnPitchQ = 0;
int16_t spawnYawQ   = 0;
bool    spawnSet    = false;
bool    lockTargetToCurrentFrame = false;

bool     lastBoxValid = false;
int16_t  lastBoxX = 0;
int16_t  lastBoxY = 0;
int16_t  lastBoxW = 0;
int16_t  lastBoxH = 0;
uint16_t lastBoxColor = 0xFFFF;
bool     lastBoxHollow = false;

ICM_20948_I2C imu;

bool  zeroSet    = false;
float zeroRoll   = 0.0f;
float zeroPitch  = 0.0f;
float zeroYaw    = 0.0f;
uint32_t zeroArmMs = 0;
uint32_t lastZeroCountdownDrawMs = 0;
uint32_t lastZeroCountdownSec = 999;
float zeroWarmupRollSum = 0.0f;
float zeroWarmupPitchSum = 0.0f;
float zeroWarmupYawSinSum = 0.0f;
float zeroWarmupYawCosSum = 0.0f;
uint16_t zeroWarmupSampleCount = 0;

float lastYaw          = 0.0f;
unsigned long lastMove = 0;
float yawStable        = 0.0f;
bool  yawStableInit    = false;
