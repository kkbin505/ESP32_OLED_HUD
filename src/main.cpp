#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <D:\LiZhen\Github\c_library_v2\common\mavlink.h>

// #define DEMO_MODE  // 打开此行启用模拟模式

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_SCL_PIN 13
#define I2C_SDA_PIN 12
#define RX1_PIN 9
#define TX1_PIN 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 飞行数据
float roll = 0;
float pitch = 0;
float sim_speed = 50; // m/s
float sim_alt = 100;  // m
int heading = 0;
int s1 = 0, s2 = 0, flight = 0;

bool signalReceived = false;
unsigned long lastMsgTime = 0;
const unsigned long SIGNAL_TIMEOUT = 1000; // 1秒无消息判为丢失

// MAVLink 处理函数
void handleMavlink(mavlink_message_t &msg)
{
  lastMsgTime = millis(); // 收到任何有效 MAVLink 消息，更新时间
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_ATTITUDE:
  {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);
    roll = att.roll * 57.2958; // rad -> deg
    pitch = att.pitch * 57.2958;
    heading = int(att.yaw * 57.2958) % 360;
    break;
  }
  case MAVLINK_MSG_ID_VFR_HUD:
  {
    mavlink_vfr_hud_t hud;
    mavlink_msg_vfr_hud_decode(&msg, &hud);
    sim_speed = hud.groundspeed;
    sim_alt = hud.alt;
    break;
  }
  case MAVLINK_MSG_ID_HEARTBEAT:
    break;
  default:
    break;
  }
}

// HUD 绘制函数
void drawHUD(float roll, float pitch, float groundspeed, float altitude, int heading)
{
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH / 2) - 15, 0);
  display.printf("HDG:%03d", heading);

  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2 + 8;
  float rollRad = -roll * M_PI / 180.0;
  float pitchOffset = pitch * 0.8;

  int x1 = cx - 60 * cos(rollRad);
  int y1 = cy + pitchOffset + 60 * sin(rollRad);
  int x2 = cx + 60 * cos(rollRad);
  int y2 = cy + pitchOffset - 60 * sin(rollRad);
  display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);

  // 中心十字
  display.drawLine(cx - 5, cy, cx + 5, cy, SSD1306_WHITE);
  display.drawLine(cx, cy - 5, cx, cy + 5, SSD1306_WHITE);

  // 左侧速度标尺
  int speedBaseY = SCREEN_HEIGHT / 2;
  for (int i = -2; i <= 2; i++)
  {
    int speedMark = (int)groundspeed + (i * 10);
    int y = speedBaseY + (i * 10);
    display.drawLine(0, y, 8, y, SSD1306_WHITE);
    display.setCursor(10, y - 3);
    display.printf("%d", speedMark);
  }

  // 右侧高度标尺
  int altBaseY = SCREEN_HEIGHT / 2;
  for (int i = -2; i <= 2; i++)
  {
    int altMark = (int)altitude + (i * 10);
    int y = altBaseY + (i * 10);
    display.drawLine(SCREEN_WIDTH - 8, y, SCREEN_WIDTH, y, SSD1306_WHITE);
    display.setCursor(SCREEN_WIDTH - 28, y - 3);
    display.printf("%d", altMark);
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH - 6 * 3 * 2) / 2, SCREEN_HEIGHT / 2 - 60);
  if (pitch >= 0)
    display.print("+");
  display.print("P:");
  display.print((int)pitch);
  display.print("o");
}

// Signal Lost 绘制
void drawSignalLost()
{
  display.setTextSize(2);
  display.setCursor((SCREEN_WIDTH / 2) - 60, 0);
  display.printf("Signal Lost!!");
}

// ------------------- setup -------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  lastMsgTime = millis();
}

// ------------------- loop -------------------
void loop()
{
  display.clearDisplay();

#ifdef DEMO_MODE
  // 模拟飞行数据
  sim_speed = 50 + sin(millis() / 2000.0) * 10;
  sim_alt = 100 + sin(millis() / 2500.0) * 20;
  roll = sin(millis() / 2000.0) * 30;
  pitch = sin(millis() / 2500.0) * 15;
  heading = (millis() / 100) % 360;
  signalReceived = true;
#else
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available())
  {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      handleMavlink(msg);
    }
  }

  while (Serial1.available())
  {
    uint8_t c = Serial1.read();
    Serial.write(c); // 先把收到的原始字节打印出来
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      handleMavlink(msg);
    }
  }

      // 超时判断
      if (millis() - lastMsgTime > SIGNAL_TIMEOUT)
  {
    signalReceived = false;
  }
  else
  {
    signalReceived = true;
  }
#endif

  if (signalReceived)
  {
    drawHUD(roll, pitch, sim_speed, sim_alt, heading);
  }
  else
  {
    drawSignalLost();
  }

  display.display();
  delay(50);
}
