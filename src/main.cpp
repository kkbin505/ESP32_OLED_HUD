#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <common/mavlink.h>

// #define DEMO_MODE

// TFT SPI pins
#define TFT_CS   10
#define TFT_DC   46
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_MISO 13
#define TFT_RST  -1
#define TFT_BL   45

// Touch I2C pins
#define TOUCH_SDA  16
#define TOUCH_SCL  15
#define TOUCH_RST  18
#define TOUCH_INT  17

// MAVLink on UART0 hardware (GPIO 43 RX / 44 TX, native — no pin conflict)
HardwareSerial mavSerial(0);

// Screen (portrait 240×320)
#define SCREEN_W   240
#define SCREEN_H   320

// Layout zones (top → bottom)
#define HDG_H      25    // heading tape
#define HORIZON_H  210   // artificial horizon (full width)
#define INFO_H     45    // speed + altitude panel
#define SLIDE_H    40    // slide-to-arm bar (bottom 40 px)

#define HUD_CX     (SCREEN_W / 2)
#define HUD_CY     (HDG_H + HORIZON_H / 2)   // = 130
#define INFO_Y     (HDG_H + HORIZON_H)        // = 235
#define SLIDE_Y    (INFO_Y + INFO_H)          // = 280

// Colors (RGB565)
#define COLOR_SKY      0x4C9F
#define COLOR_GROUND   0xFC00
#define COLOR_SIDEBAR  0x07E0
#define COLOR_HDG_BG   0x4208
#define COLOR_DARK     0x1082  // very dark grey (slider bg)

Adafruit_ILI9341  tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
Adafruit_FT6206   touch;

// Full-screen canvas: 240×320×2 = 153,600 bytes on heap
GFXcanvas16 canvas(SCREEN_W, SCREEN_H);

float   roll = 0, pitch = 0, groundspeed = 0, altitude = 0;
int     heading = 0;

// MAVLink state
bool     isArmed    = false;
uint32_t customMode = 0;
uint8_t  mavType    = 0;

// Slider touch state: -1 = not touching
int sliderX      = -1;
int sliderStartX = -1;  // where the gesture began (must start from correct side)

// Pending arm/disarm: set when command is sent, cleared on HEARTBEAT confirmation
// 0 = idle, 1 = arming pending, -1 = disarming pending
int8_t        armPending      = 0;
unsigned long armPendingMs    = 0;
const unsigned long ARM_TIMEOUT = 4000; // give up after 4 s

bool          signalReceived  = false;
unsigned long lastMsgTime     = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStreamReqMs = 0;
const unsigned long SIGNAL_TIMEOUT = 1000;

// Send GCS heartbeat so ArduPilot starts streaming telemetry
void sendHeartbeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(
    255, 190,   // sysid=255 (GCS), compid=190
    &msg,
    MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
    0, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}

// Request ATTITUDE (EXTRA1) and VFR_HUD (EXTRA2) streams from ArduPilot
void requestStreams()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  // EXTRA1: ATTITUDE at 10 Hz
  mavlink_msg_request_data_stream_pack(
    255, 190, &msg,
    1, 1,                       // target sysid / compid
    MAV_DATA_STREAM_EXTRA1, 10, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);

  // EXTRA2: VFR_HUD at 5 Hz
  mavlink_msg_request_data_stream_pack(
    255, 190, &msg,
    1, 1,
    MAV_DATA_STREAM_EXTRA2, 5, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}

// Send ARM / DISARM command and set pending state
void sendArmDisarm(bool arm)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    255, 190, &msg, 1, 1,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0, arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
  mavSerial.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
  armPending   = arm ? 1 : -1;
  armPendingMs = millis();
}

// Flight mode name for ArduCopter / ArduPlane
const char* getModeName()
{
  // ArduCopter
  if (mavType == MAV_TYPE_QUADROTOR || mavType == MAV_TYPE_HEXAROTOR ||
      mavType == MAV_TYPE_OCTOROTOR  || mavType == MAV_TYPE_TRICOPTER ||
      mavType == MAV_TYPE_HELICOPTER) {
    switch (customMode) {
      case 0:  return "STABILIZE";
      case 2:  return "ALT HOLD";
      case 3:  return "AUTO";
      case 4:  return "GUIDED";
      case 5:  return "LOITER";
      case 6:  return "RTL";
      case 9:  return "LAND";
      case 13: return "POSHOLD";
      case 14: return "BRAKE";
      default: break;
    }
  } else { // ArduPlane / default
    switch (customMode) {
      case 0:  return "MANUAL";
      case 5:  return "FBWA";
      case 6:  return "FBWB";
      case 10: return "AUTO";
      case 11: return "RTL";
      case 12: return "LOITER";
      case 15: return "GUIDED";
      case 17: return "QSTAB";
      case 18: return "QHOVER";
      case 19: return "QLOITER";
      default: break;
    }
  }
  static char buf[10];
  snprintf(buf, sizeof(buf), "M:%lu", customMode);
  return buf;
}

// ── MAVLink ──────────────────────────────────────────────
void handleMavlink(mavlink_message_t &msg)
{
  lastMsgTime = millis();
  switch (msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT: {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    bool newArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    // Clear pending when FC confirms the new state
    if (armPending == 1  && newArmed)  armPending = 0;
    if (armPending == -1 && !newArmed) armPending = 0;
    isArmed    = newArmed;
    customMode = hb.custom_mode;
    mavType    = hb.type;
    break;
  }
  case MAVLINK_MSG_ID_ATTITUDE: {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);
    roll    =  att.roll  * 57.2958f;
    pitch   =  att.pitch * 57.2958f;
    heading = (int)(att.yaw * 57.2958f + 360) % 360;
    break;
  }
  case MAVLINK_MSG_ID_VFR_HUD: {
    mavlink_vfr_hud_t hud;
    mavlink_msg_vfr_hud_decode(&msg, &hud);
    groundspeed = hud.groundspeed * 3.6f;
    altitude    = hud.alt;
    break;
  }
  default: break;
  }
}

// ── Heading tape (into canvas) ────────────────────────────
void drawHeadingTape(int hdg)
{
  canvas.fillRect(0, 0, SCREEN_W, HDG_H, COLOR_HDG_BG);
  const int pxPerDeg = 4;
  for (int delta = -38; delta <= 38; delta++) {
    int deg = ((hdg + delta) % 360 + 360) % 360;
    int x   = HUD_CX + delta * pxPerDeg;
    if (x < 1 || x > SCREEN_W - 2) continue;
    if (deg % 10 == 0) {
      canvas.drawFastVLine(x, 0, 12, ILI9341_WHITE);
      char buf[4];
      snprintf(buf, sizeof(buf), "%03d", deg);
      canvas.setTextSize(1);
      canvas.setTextColor(ILI9341_WHITE, COLOR_HDG_BG);
      canvas.setCursor(x - 9, 13);
      canvas.print(buf);
    } else if (deg % 5 == 0) {
      canvas.drawFastVLine(x, 0, 7, ILI9341_LIGHTGREY);
    }
  }
  char buf[4];
  snprintf(buf, sizeof(buf), "%03d", hdg);
  canvas.fillRect(HUD_CX - 14, 0, 28, HDG_H, ILI9341_WHITE);
  canvas.setTextSize(1);
  canvas.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
  canvas.setCursor(HUD_CX - 10, 8);
  canvas.print(buf);
}

// ── Artificial horizon + pitch ladder (into canvas) ───────
void drawHorizon(float rollDeg, float pitchDeg)
{
  float rollRad = -rollDeg * (M_PI / 180.0f);
  float cosR    = cosf(rollRad);
  float sinR    = sinf(rollRad);
  int   pitchPx = (int)(pitchDeg * 3.0f);

  int cx = HUD_CX;
  int cy = HUD_CY + pitchPx;

  const int xL = 0;
  const int xR = SCREEN_W;

  // Scanline fill over the horizon area only (HDG_H to INFO_Y)
  for (int y = HDG_H; y < INFO_Y; y++) {
    if (fabsf(sinR) < 0.02f) {
      canvas.drawFastHLine(xL, y, xR - xL, (y < cy) ? COLOR_GROUND : COLOR_SKY);
    } else {
      int split = cx + (int)((float)(y - cy) * cosR / sinR);
      split = constrain(split, xL, xR);
      if (sinR < 0) {
        if (split > xL) canvas.drawFastHLine(xL,   y, split - xL, COLOR_GROUND);
        if (split < xR) canvas.drawFastHLine(split, y, xR - split, COLOR_SKY);
      } else {
        if (split > xL) canvas.drawFastHLine(xL,   y, split - xL, COLOR_SKY);
        if (split < xR) canvas.drawFastHLine(split, y, xR - split, COLOR_GROUND);
      }
    }
  }

  // Horizon line (2 px thick white)
  const int hw = 600;
  int x1 = cx - (int)(hw * cosR);
  int y1 = cy - (int)(hw * sinR);
  int x2 = cx + (int)(hw * cosR);
  int y2 = cy + (int)(hw * sinR);
  canvas.drawLine(x1, y1, x2, y2, ILI9341_WHITE);
  canvas.drawLine(x1 + (int)sinR, y1 + (int)cosR,
                  x2 + (int)sinR, y2 + (int)cosR, ILI9341_WHITE);

  // Pitch ladder ±20°, every 5°
  canvas.setTextSize(1);
  for (int deg = -20; deg <= 20; deg += 5) {
    if (deg == 0) continue;
    // "Up" direction perpendicular to horizon: (sinR, -cosR)
    int plx = cx + (int)(deg * 3.0f * sinR);
    int ply = cy - (int)(deg * 3.0f * cosR);
    if (ply < HDG_H + 4 || ply > INFO_Y - 4) continue;

    int lineLen = (deg % 10 == 0) ? 30 : 18;
    int ax = plx - (int)(lineLen * cosR);
    int ay = ply - (int)(lineLen * sinR);
    int bx = plx + (int)(lineLen * cosR);
    int by = ply + (int)(lineLen * sinR);
    canvas.drawLine(ax, ay, bx, by, ILI9341_WHITE);

    if (deg % 10 == 0) {
      char buf[5];
      snprintf(buf, sizeof(buf), "%+d", deg);
      uint16_t bg = (ply > cy) ? COLOR_SKY : COLOR_GROUND;
      canvas.setTextColor(ILI9341_WHITE, bg);
      canvas.setCursor(bx + 3, by - 3);
      canvas.print(buf);
    }
  }

  // Aircraft symbol (fixed at screen centre)
  canvas.drawLine(HUD_CX - 20, HUD_CY, HUD_CX - 8,  HUD_CY,     ILI9341_YELLOW);
  canvas.drawLine(HUD_CX + 8,  HUD_CY, HUD_CX + 20, HUD_CY,     ILI9341_YELLOW);
  canvas.drawLine(HUD_CX - 8,  HUD_CY, HUD_CX,      HUD_CY + 6, ILI9341_YELLOW);
  canvas.drawLine(HUD_CX,      HUD_CY + 6, HUD_CX + 8, HUD_CY,  ILI9341_YELLOW);
  canvas.fillCircle(HUD_CX, HUD_CY, 2, ILI9341_YELLOW);
}

// ── Bottom info panel (45 px): speed + altitude + FC status ─
void drawInfoPanel(float spd, float alt)
{
  canvas.fillRect(0, INFO_Y, SCREEN_W, INFO_H, ILI9341_BLACK);
  // Top border: red when armed, green when disarmed
  uint16_t borderCol = isArmed ? ILI9341_RED : COLOR_SIDEBAR;
  canvas.drawFastHLine(0, INFO_Y,     SCREEN_W, borderCol);
  canvas.drawFastHLine(0, INFO_Y + 1, SCREEN_W, borderCol);
  canvas.drawFastVLine(SCREEN_W / 2, INFO_Y + 2, INFO_H - 4, ILI9341_DARKGREY);

  // ARM / DISARM badge (top-right corner)
  canvas.setTextSize(1);
  if (isArmed) {
    canvas.fillRect(SCREEN_W - 38, INFO_Y + 3, 34, 12, ILI9341_RED);
    canvas.setTextColor(ILI9341_WHITE, ILI9341_RED);
    canvas.setCursor(SCREEN_W - 35, INFO_Y + 5);
    canvas.print("ARMED");
  } else {
    canvas.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
    canvas.setCursor(SCREEN_W - 42, INFO_Y + 5);
    canvas.print("DISARMED");
  }

  const int numY  = INFO_Y + 6;
  const int lblY  = INFO_Y + 26;

  // Speed
  canvas.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  canvas.setTextSize(2);
  char buf[6];
  snprintf(buf, sizeof(buf), "%3.0f", spd);
  canvas.setCursor(4, numY);
  canvas.print(buf);
  canvas.setTextSize(1);
  canvas.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  canvas.setCursor(4, lblY);
  canvas.print("SPD Km/h");

  // Altitude
  canvas.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  canvas.setTextSize(2);
  char buf2[7];
  snprintf(buf2, sizeof(buf2), "%4.0f", alt);
  canvas.setCursor(SCREEN_W / 2 + 4, numY);
  canvas.print(buf2);
  canvas.setTextSize(1);
  canvas.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  canvas.setCursor(SCREEN_W / 2 + 4, lblY);
  canvas.print("ALT  m");
}

// ── Slide-to-ARM bar (40 px at bottom) ───────────────────
void drawSlider()
{
  // Timeout: if FC doesn't confirm within ARM_TIMEOUT, give up
  if (armPending != 0 && millis() - armPendingMs > ARM_TIMEOUT)
    armPending = 0;

  const int bx = 4, by = SLIDE_Y + 4;
  const int bw = SCREEN_W - 8, bh = SLIDE_H - 8;
  const int tw = 38;

  // Background colour: flash orange while pending
  bool   pending   = armPending != 0;
  bool   flashOn   = pending && ((millis() / 200) % 2 == 0);
  uint16_t bgCol   = flashOn ? 0xFD20 : COLOR_DARK; // orange flash
  canvas.fillRect(0, SLIDE_Y, SCREEN_W, SLIDE_H, bgCol);
  canvas.drawRoundRect(bx, by, bw, bh, 5,
    pending ? 0xFD20 : ILI9341_DARKGREY);

  // Label
  canvas.setTextSize(1);
  if (pending) {
    canvas.setTextColor(ILI9341_WHITE, bgCol);
    canvas.setCursor(HUD_CX - 24, SLIDE_Y + 15);
    canvas.print(armPending == 1 ? "ARMING..." : "DISARMING...");
  } else if (isArmed) {
    canvas.setTextColor(ILI9341_RED, COLOR_DARK);
    const char* m = getModeName();
    canvas.setCursor(HUD_CX - (int)(strlen(m) * 3), SLIDE_Y + 15);
    canvas.print(m);
  } else {
    canvas.setTextColor(ILI9341_DARKGREY, COLOR_DARK);
    canvas.setCursor(HUD_CX - 30, SLIDE_Y + 15);
    canvas.print("slide to ARM >");
  }

  // Thumb
  int maxX = bx + bw - tw - 2;
  int thumbX;
  if (sliderX < 0)
    thumbX = isArmed ? maxX : bx + 2;
  else
    thumbX = constrain(sliderX - tw / 2, bx + 2, maxX);

  uint16_t thumbCol = pending  ? 0xFD20 :
                      isArmed  ? ILI9341_RED : ILI9341_GREEN;
  canvas.fillRoundRect(thumbX, by + 2, tw, bh - 4, 4, thumbCol);
  canvas.setTextColor(ILI9341_WHITE, thumbCol);
  canvas.setCursor(thumbX + tw / 2 - 3, by + bh / 2 - 4);
  if (pending)       canvas.print("~");
  else if (isArmed)  canvas.print("<");
  else               canvas.print(">");
}

// ── Signal lost (direct to display, not canvas) ───────────
void drawSignalLost()
{
  tft.fillScreen(ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(30, 90);
  tft.print("SIGNAL LOST");
}

// Push the full canvas to the display in one SPI burst
void pushFrame()
{
  tft.startWrite();
  tft.setAddrWindow(0, 0, SCREEN_W, SCREEN_H);
  tft.writePixels(canvas.getBuffer(), SCREEN_W * SCREEN_H);
  tft.endWrite();
}

// ── setup ─────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  mavSerial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Touch I2C
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  touch.begin(40, &Wire);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  tft.begin(40000000);
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);

  lastMsgTime = millis();
}

// ── loop ──────────────────────────────────────────────────
void loop()
{
#ifdef DEMO_MODE
  unsigned long t = millis();
  groundspeed = 80.0f  + sinf(t / 2000.0f) * 20.0f;
  altitude    = 250.0f + sinf(t / 2500.0f) * 30.0f;
  roll        = sinf(t / 2000.0f) * 30.0f;
  pitch       = sinf(t / 2500.0f) * 15.0f;
  heading     = (t / 100) % 360;
  signalReceived = true;
#else
  // Send GCS heartbeat every 1 s so ArduPilot enables telemetry streams
  unsigned long now = millis();
  if (now - lastHeartbeatMs >= 1000) {
    sendHeartbeat();
    lastHeartbeatMs = now;
  }
  // Re-request streams every 5 s (also covers ArduPilot reboot)
  if (now - lastStreamReqMs >= 5000) {
    requestStreams();
    lastStreamReqMs = now;
  }

  mavlink_message_t msg;
  mavlink_status_t  status;
  while (mavSerial.available()) {
    uint8_t c = mavSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      handleMavlink(msg);
  }
  signalReceived = (millis() - lastMsgTime <= SIGNAL_TIMEOUT);
#endif

  // Touch: slide-to-arm gesture tracking
  // Must start from the correct side and slide across — tap triggers nothing.
  if (touch.touched()) {
    TS_Point p = touch.getPoint();
    if (p.y >= SLIDE_Y) {
      const int bx = 4, bw = SCREEN_W - 8;
      const int zoneLeft  = bx + bw * 1 / 4;  // valid ARM start zone  (left 25%)
      const int zoneRight = bx + bw * 3 / 4;  // valid DISARM start zone (right 25%)
      const int trigArm   = bx + bw * 2 / 3;  // ARM trigger at 2/3
      const int trigDis   = bx + bw * 1 / 3;  // DISARM trigger at 1/3

      if (sliderStartX < 0) {
        // First contact — record start; only accept if in valid start zone
        if (!isArmed && p.x <= zoneLeft)  sliderStartX = p.x;  // ARM: start from left
        if (isArmed  && p.x >= zoneRight) sliderStartX = p.x;  // DISARM: start from right
      }

      if (sliderStartX >= 0) {
        sliderX = p.x;
        // Trigger only if started from correct side AND reached threshold
        if (!isArmed && sliderStartX <= zoneLeft && sliderX >= trigArm) {
          sendArmDisarm(true);
          sliderX = sliderStartX = -1;
        } else if (isArmed && sliderStartX >= zoneRight && sliderX <= trigDis) {
          sendArmDisarm(false);
          sliderX = sliderStartX = -1;
        }
      }
    } else {
      sliderX = sliderStartX = -1;
    }
  } else {
    sliderX = sliderStartX = -1;  // finger lifted: reset
  }

  if (signalReceived) {
    drawHorizon(roll, pitch);
    drawInfoPanel(groundspeed, altitude);
    drawHeadingTape(heading);
    drawSlider();
    pushFrame();
  } else {
    drawSignalLost();
  }
}
