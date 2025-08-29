/*
  CRSF decode from:
  https://github.com/CapnBry/CRServoF
  OLED:
  adafruit/Adafruit SSD1306@^2.5.9
  adafruit/Adafruit GFX Library@^1.11.9

*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CrsfSerial.h>
#include <crsf_protocol.h>

#define CRSF

// #define DEMO_MODE
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_SCL_PIN 13
#define I2C_SDA_PIN 12
#define RX1_PIN 8
#define TX1_PIN 9
#define SERVO_MIN 1000
#define SERVO_CENTER 1500
#define SERVO_MAX 2000
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long lastOledUpdate = 0;
const uint16_t OLED_UPDATE_MS = 100; // 10 Hz 更新

// 模拟数据
float roll = 0;
float pitch = 0;
float sim_speed = 50; // m/s
float sim_alt = 100;  // m
int heading = 0;

//  float pitch = 0, roll = 0, yaw = 0;
int s1 = 0, s2 = 0, flight = 0;

String inputLine = "";

bool signalReceived = false;

// CRSF
void crsfLinkUp();
void crsfLinkDown();
void onCrsfChannels();

#ifdef CRSF

#define CRSF_RX_PIN 8
#define CRSF_TX_PIN 9
// crsf解码
int crsfChannelValues[CRSF_NUM_CHANNELS + 1];
CrsfSerial crsf(Serial1);
// CRSF failsafe:
// 失控保护值数组，索引与通道枚举值对应（忽略索引0）
unsigned int channelFailsafe[CRSF_NUM_CHANNELS + 1] = {
    0,            // 索引0未使用（通道从1开始）
    SERVO_CENTER, // CH_ROLL (1) - 横滚
    SERVO_CENTER, // CH_PITCH (2) - 俯仰
    SERVO_MIN,    // CH_THROTTLE (3) - 油门（失控时最小，确保安全）
    SERVO_CENTER, // CH_YAW (4) - 偏航
    SERVO_CENTER, // CH_ARM (5) - 解锁（用于ExpressLRS的油门切断）
    SERVO_MAX,    // CH_AUX1 (6) - 模式切换，默认为Angle模式
    SERVO_CENTER, // CH_AUX2 (7) - LED控制
    SERVO_CENTER, // CH_AUX3 (8) - Buzzer
    SERVO_CENTER, // CH_AUX4 (9) - 待定义
    SERVO_CENTER, // CH_AUX5 (10) - 待定义
    SERVO_CENTER, // CH_AUX6 (11) - 待定义
    SERVO_CENTER, // CH_AUX7 (12) - 待定义
    SERVO_CENTER, // CH_AUX8 (13) - 待定义
    SERVO_CENTER, // CH_AUX9 (14) - 待定义
    SERVO_CENTER, // CH_AUX10 (15) - 待定义
    SERVO_CENTER  // CH_AUX11 (16) - 待定义
};
unsigned long lastCrsfPacketTime = 0;  // 上一包时间
unsigned long lastFramerateUpdate = 0; // OLED 刷新计时
float crsfFreshRate = 0;               // 显示值
float crsfFreshRateMax = 0;            // 当前秒的最高刷新率
#endif

void drawHUD(float roll, float pitch, float groundspeed, float altitude, int heading)
{

  // 顶部航向
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH / 2) - 15, 0);
  display.printf("HDG:%03d", heading);

  // 中心点
  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2 + 8;
  //   // 屏幕中心Y坐标
  int centerY = SCREEN_HEIGHT / 2;

  // 地平线 (一根线)
  float rollRad = roll * M_PI / 180.0;
  float pitchOffset = pitch * 0.8; // 像素偏移比例
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

  //   // 计算偏移量，pitch 每度对应多少像素，这个比例你可以调节
  //   // 例如每度对应 2 像素，高度 64，最大 +/- 32度会移动64像素
  //   float pixelsPerDegree = 2.0f;

  //   // pitch 反向移动线条：pitch正，线条往下移（加像素）
  //   int lineY = centerY + (int)(pitch * pixelsPerDegree);

  //   // 限制线条不跑出屏幕
  //   if (lineY < 0) lineY = 0;
  //   if (lineY > SCREEN_HEIGHT - 1) lineY = SCREEN_HEIGHT - 1;

  //   display.clearDisplay();

  //   // 画水平线，代表pitch=0的位置
  //   display.drawLine(0, lineY, SCREEN_WIDTH, lineY, SSD1306_WHITE);

  // 显示当前pitch数字，固定在屏幕中间（你也可以调整位置）
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH - 6 * 3 * 2) / 2, centerY - 60);
  if (pitch >= 0)
    display.print("+");
  display.print("P:");
  display.print((int)pitch);
  display.print("o");
}

void drawSignalLost()
{

  // 顶部航向
  display.setTextSize(2);
  display.setCursor((SCREEN_WIDTH / 2) - 60, 0);
  display.printf("Singal Lost!!");

  // 中心点
  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2 + 8;
  //   // 屏幕中心Y坐标
  int centerY = SCREEN_HEIGHT / 2;
}

void parseData(const String &data)
{

  // 使用正则或者简单字符串查找解析，示例用String的indexOf和substring
  int p1 = data.indexOf("P:");
  int p2 = data.indexOf(",R:");
  int p3 = data.indexOf(",Y:");
  int p4 = data.indexOf(",S1:");
  int p5 = data.indexOf(",S2:");
  int p6 = data.indexOf(",Flight:");

  if (p1 != -1 && p2 != -1 && p3 != -1 && p4 != -1 && p5 != -1 && p6 != -1)
  {
    pitch = data.substring(p1 + 2, p2).toFloat();
    roll = data.substring(p2 + 3, p3).toFloat();
    heading = data.substring(p3 + 3, p4).toFloat();
    s1 = data.substring(p4 + 4, p5).toInt();
    s2 = data.substring(p5 + 4, p6).toInt();
    flight = data.substring(p6 + 8).toInt();

    Serial.printf("Parsed pitch=%.2f, roll=%.2f, yaw=%.2f, s1=%d, s2=%d, flight=%d\n",
                  pitch, roll, heading, s1, s2, flight);
    signalReceived = true;
  }
  else
  {
    Serial.println("Parse error: format mismatch");
    signalReceived = false;
  }
}

void displayCRSF();

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 把 I2C 提速到 400kHz（默认通常是 100kHz）
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

#ifdef CRSF
  Serial1.setRxBufferSize(1024);
  Serial1.begin(420000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  Serial.println("Serial1 started for CRSF");
  // 初始化 failsafe（1-based）
  for (uint8_t i = 0; i <= CRSF_NUM_CHANNELS; ++i)
  {
    if (i == 0)
      channelFailsafe[i] = 0;
    else if (i == 3)
      channelFailsafe[i] = SERVO_MIN; // throttle 最小
    else
      channelFailsafe[i] = SERVO_CENTER;
    // 初始化工作数组
    crsfChannelValues[i] = channelFailsafe[i];
  }
  // 初始化CRSF
  crsf.onLinkUp = crsfLinkUp;
  crsf.onLinkDown = crsfLinkDown;
  crsf.onPacketChannels = onCrsfChannels;
  crsf.begin();
  Serial.println("CRSF begin called");
#else
  // 初始化UART1，设置TX和RX引脚
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

#endif
  display.setRotation(0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.printf("Waiting for data");
  display.display();
}

void loop()
{
  display.clearDisplay();
#ifdef DEMO_MODE

  // 模拟动画
  sim_speed = 50 + sin(millis() / 2000.0) * 10; // 40~60 m/s
  sim_alt = 100 + sin(millis() / 2500.0) * 20;  // 80~120 m
// 解析：
#else
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c == '\n')
    {
      // 收到一行完整数据，开始解析
      Serial.print("Received: ");
      Serial.println(inputLine);
      parseData(inputLine);
      inputLine = "";
    }
    else if (c != '\r')
    {
      inputLine += c; // 拼接数据行
    }
  }

#endif

#ifdef CRSF
  crsf.loop();
  displayCRSF();
#else

  if (signalReceived == true)
  {

    drawHUD(roll, pitch, sim_speed, sim_alt, heading);
  }
  else
  {

    drawSignalLost();
  }
  display.display();
#endif

  // delay(50);
}

#ifdef CRSF
void displayCRSF()
{
  display.setCursor(70, 0); // OLED 最下面一行
  unsigned long now = millis();
  if (now - lastFramerateUpdate >= 5000) // 每5秒更新一次
  {
    crsfFreshRate = crsfFreshRateMax; // 显示当前秒的最高刷新率
    crsfFreshRateMax = 0;             // 重置下一秒的最大值
    lastFramerateUpdate = now;

  }
  display.printf("%.fHz", crsfFreshRate);

  if (now - lastOledUpdate < OLED_UPDATE_MS)
    return; // 控制刷新频率
  lastOledUpdate = now;
  for (int i = 0; i < 8; i++)
  {
    // 映射到 0 ~ 100 （条形宽度）
    // int barWidth = map(val, CRSF_ELIMIT_US_MIN, CRSF_ELIMIT_US_MAX, 0, 80);
    int channnelVal = crsfChannelValues[i + 1];

    int y = i * 8; // 每条占 8 像素高度
    display.setCursor(0, y);
    display.printf("CH%d:", i + 1);

    String strNum = String(channnelVal);

    // 计算字符串像素宽度
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(strNum, 0, 0, &x1, &y1, &w, &h);

    // 让它靠右对齐，屏幕宽度减去字符串宽度
    int x = SCREEN_WIDTH - w - 64;

    display.setCursor(x, y);
    display.printf("%d", channnelVal); // val 是 int;
    Serial.println(channnelVal);

    // 画条
    // display.drawRect(30, y, 80, 7, SSD1306_WHITE);       // 外框
    // display.fillRect(30, y, barWidth, 7, SSD1306_WHITE); // 填充
    // Serial.println("Printing");
  }
  // 在底部显示快速调试信息

  display.display(); // 只调用一次，提交本次所有改动
}

void crsfLinkUp()
{
  Serial.println("ELRS OK");
}

void crsfLinkDown()
{
  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
  {
    crsfChannelValues[i] = channelFailsafe[i];
  }
  Serial.println("ELRS LOSt");
}

void onCrsfChannels()
{
  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++)
  {
    crsfChannelValues[i] = crsf.getChannel(i);
  }
  // 计算刷新率
  unsigned long now = millis();
  if (lastCrsfPacketTime > 0)
  {
    float dt = (now - lastCrsfPacketTime) / 1000.0; // 秒
    float instRate = 1.0 / dt;
    if (instRate > crsfFreshRateMax)
      crsfFreshRateMax = instRate;
  }
  lastCrsfPacketTime = now;
}
#endif