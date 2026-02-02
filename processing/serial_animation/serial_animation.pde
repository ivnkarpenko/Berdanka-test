import processing.serial.*;

Serial port;
String status = "";
int lastMs = 0;

// Data from Arduino
float roll, pitch, yaw;
int rollQ, pitchQ, yawQ;
int spawnPitchQ, spawnYawQ;
int spawnSet = 0;
int onTarget = 0;
String lastMsg = "-";
String lastIP = "0.0.0.0";

// UI (matches original logic)
final int SCREEN_W = 480;
final int SCREEN_H = 320;
final int SCALE = 2;
final int CX = SCREEN_W / 2;
final int CY = SCREEN_H / 2;
final float PX_PER_DEG = 3.0;
final int BOX_SIZE = 50;
final int EDGE_VISIBLE_PX = 5;

void setup() {
  surface.setSize(SCREEN_W * SCALE, SCREEN_H * SCALE);
  frameRate(60);
  smooth(4);
  textFont(createFont("Georgia", 16));

  String[] ports = Serial.list();
  if (ports.length == 0) {
    status = "No serial ports found.";
  } else {
    int idx = pickPort(ports);
    port = new Serial(this, ports[idx], 115200);
    port.bufferUntil('\n');
    status = "Port: " + ports[idx] + " @115200";
  }
}

void draw() {
  scale(SCALE);
  background(0);

  drawCross();
  drawBox();
  drawHUD();

  resetMatrix();
  fill(220);
  text(status, 12, height - 12);
}

void drawCross() {
  stroke(90);
  line(CX, 0, CX, SCREEN_H);
  line(0, CY, SCREEN_W, CY);
}

void drawHUD() {
  fill(255);
  textSize(12);
  text("Roll  = " + nf(rollQ, 1, 0), 10, 16);
  text("Pitch = " + nf(pitchQ, 1, 0), 10, 32);
  text("Yaw   = " + nf(yawQ, 1, 0), 10, 48);
  text("Msg   = " + lastMsg, 10, 64);
  text("IP    = " + lastIP, 10, 80);
}

void drawBox() {
  int pitchRelQ = pitchQ - (spawnSet == 1 ? spawnPitchQ : 0);
  int yawRelQ = yawQ - (spawnSet == 1 ? spawnYawQ : 0);

  int centerX = round(CX + yawRelQ * PX_PER_DEG);
  int centerY = round(CY + pitchRelQ * PX_PER_DEG);

  int boxX = centerX - BOX_SIZE / 2;
  int boxY = centerY - BOX_SIZE / 2;

  if (boxX > SCREEN_W - EDGE_VISIBLE_PX) boxX = SCREEN_W - EDGE_VISIBLE_PX;
  if (boxX < -BOX_SIZE + EDGE_VISIBLE_PX) boxX = -BOX_SIZE + EDGE_VISIBLE_PX;
  if (boxY > SCREEN_H - EDGE_VISIBLE_PX) boxY = SCREEN_H - EDGE_VISIBLE_PX;
  if (boxY < -BOX_SIZE + EDGE_VISIBLE_PX) boxY = -BOX_SIZE + EDGE_VISIBLE_PX;

  int nx = max(0, boxX);
  int ny = max(0, boxY);
  int nw = min(SCREEN_W, boxX + BOX_SIZE) - nx;
  int nh = min(SCREEN_H, boxY + BOX_SIZE) - ny;
  if (nw <= 0 || nh <= 0) return;

  noStroke();
  if (onTarget == 1) fill(0, 255, 0);
  else fill(255, 0, 0);
  rect(nx, ny, nw, nh);

  // redraw cross inside box area
  stroke(90);
  if (CX >= nx && CX < (nx + nw)) {
    line(CX, ny, CX, ny + nh - 1);
  }
  if (CY >= ny && CY < (ny + nh)) {
    line(nx, CY, nx + nw - 1, CY);
  }
}

void serialEvent(Serial p) {
  while (p.available() > 0) {
    String line = p.readStringUntil('\n');
    if (line == null) return;
    line = trim(line);
    if (line.length() == 0 || line.startsWith("t_ms")) continue;

    String[] parts = split(line, ',');
    if (parts.length < 12) continue;

    try {
      lastMs = int(parts[0]);
      roll = float(parts[1]);
      pitch = float(parts[2]);
      yaw = float(parts[3]);
      rollQ = int(parts[4]);
      pitchQ = int(parts[5]);
      yawQ = int(parts[6]);
      spawnSet = int(parts[7]);
      spawnPitchQ = int(parts[8]);
      spawnYawQ = int(parts[9]);
      onTarget = int(parts[10]);
      lastMsg = parts[11];
      if (parts.length >= 13) {
        lastIP = parts[12];
      }
    } catch (Exception e) {
      // ignore parse errors
    }
  }
}

int pickPort(String[] ports) {
  for (int i = 0; i < ports.length; i++) {
    String p = ports[i].toLowerCase();
    if (p.contains("ttyacm") || p.contains("ttyusb") || p.contains("usbmodem") || p.startsWith("com")) {
      return i;
    }
  }
  return max(0, ports.length - 1);
}
