#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// #define DEMO_MODE

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_SCL_PIN 13
#define I2C_SDA_PIN 12
#define RX1_PIN 8
#define TX1_PIN 9
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 模拟数据
float roll = 0;
float pitch = 0;
float sim_speed = 50; // m/s
float sim_alt = 100;  // m
int heading = 0;

String inputLine = "";

void drawHUD(float roll, float pitch, float groundspeed, float altitude, int heading)
{

  // 顶部航向
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH / 2) - 15, 0);
  display.printf("HDG:%03d", heading);

  // 中心点
  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2 + 8;

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
}

void drawPitch(float pitch)
{
  //   // 屏幕中心Y坐标
  int centerY = SCREEN_HEIGHT / 2;

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

void parseData(const String &data)
{
  float pitch = 0, roll = 0, yaw = 0;
  int s1 = 0, s2 = 0, flight = 0;

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
    yaw = data.substring(p3 + 3, p4).toFloat();
    s1 = data.substring(p4 + 4, p5).toInt();
    s2 = data.substring(p5 + 4, p6).toInt();
    flight = data.substring(p6 + 8).toInt();

    Serial.printf("Parsed pitch=%.2f, roll=%.2f, yaw=%.2f, s1=%d, s2=%d, flight=%d\n",
                  pitch, roll, yaw, s1, s2, flight);
  }
  else
  {
    Serial.println("Parse error: format mismatch");
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // 初始化UART1，设置TX和RX引脚
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
}

void loop()
{
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

  display.clearDisplay();
  drawPitch(pitch);
  drawHUD(roll, pitch, sim_speed, sim_alt, heading);
  display.display();

  delay(50);
}