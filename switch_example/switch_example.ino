// 简化版 ESP8266 开关示例（Arduino 框架）
// 参考现有引脚映射、NeoPixel LED 定义与采样式消抖方案，并集成 WiFi + Web UI

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <PubSubClient.h>

// ======================= 基本配置 =======================
#define NUM_CHANNELS 2

#if NUM_CHANNELS == 1
  const int CHANNEL_PIN_MAP[] = {0};
#elif NUM_CHANNELS == 2
  const int CHANNEL_PIN_MAP[] = {0, 2};
#elif NUM_CHANNELS == 3
  const int CHANNEL_PIN_MAP[] = {0, 1, 2};
#else
  #error "Invalid NUM_CHANNELS defined. Must be 1, 2, or 3."
#endif

// 继电器与按键引脚（沿用原示例的物理引脚）
const int RELAY_PINS[]  = {12, 13, 5};
const int BUTTON_PINS[] = {14, 0, 2};

// NeoPixel LED 配置（沿用原示例）
#define NEOPIXEL_PIN 4
#define NUM_PIXELS   3
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ======================= WiFi 配置 =======================
const char* WIFI_SSID     = "Linksys-2.4G-wifi5";
const char* WIFI_PASSWORD = "19811201";
// 运行时可修改的 WiFi 凭据（支持 AP 配网）
String wifiSsid = WIFI_SSID;
String wifiPass = WIFI_PASSWORD;
bool apMode = false; // 当 WiFi 连接失败启动 AP 配网

ESP8266WebServer server(80);

// ======================= MQTT 配置 =======================
const char* MQTT_HOST = "192.168.1.250";
const uint16_t MQTT_PORT = 1883;
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
// 运行时可修改的 MQTT 设置
String mqttHostStr = MQTT_HOST;
uint16_t mqttPortVal = MQTT_PORT;
String mqttUser = "";
String mqttPass = "";
// 设备标识（可持久化修改）
String deviceName = "esp-switch";
String deviceType = "switch";
String deviceMacNoColon;  // 例如 5CCF7F02E3D8
String mqttRootTopic;     // 例如 smarthome/switch/esp-switch-5CCF7F02E3D8
unsigned long lastMqttReconnectAttempt = 0;

// ======================= 消抖参数 =======================
const int BUTTON_SAMPLE_COUNT = 5;                 // 总采样次数
const unsigned long BUTTON_SAMPLE_INTERVAL_MS = 10; // 采样间隔(ms)
const int BUTTON_STABLE_THRESHOLD = 4;             // 判定为稳定按下所需低电平次数

// ======================= 全局状态 =======================
bool relayStates[NUM_CHANNELS] = {false};
volatile int buttonPressedIndex = -1;  // 由中断置位：0/1/2，对应物理索引

int samplingPhysicalIndex = -1;
int buttonSampleLowCount = 0;
int buttonSampleCounter  = 0;
unsigned long lastSampleMs = 0;

// ======================= 模式状态 =======================
bool curtainMode = false; // 窗帘互斥模式：打开一个通道时自动关闭对侧通道（0 与 最后一个）

// ======================= LED 打开颜色与亮度 =======================
uint8_t ledOnR = 0;    // 默认开：R=0,G=255,B=0
uint8_t ledOnG = 255;
uint8_t ledOnB = 0;
uint8_t ledOnBrightness = 7; // 0-255，应用于打开像素的缩放

// ======================= LED 关闭颜色 =======================
uint8_t ledOffR = 255; // 默认关：R=255,G=0,B=255
uint8_t ledOffG = 0;
uint8_t ledOffB = 255;
uint8_t ledOffBrightness = 7; // 0-255，应用于关闭像素的缩放

// ======================= 照度与 Daytime 阈值 =======================
// 订阅照度 MQTT 值后，对比 daytime 阈值：
// 若照度 > 阈值，且通道处于关闭状态，则该通道 LED 熄灭；否则显示关闭颜色
long illuminationValue = -1;      // 最近一次从 MQTT 接收的照度值（-1 表示未知）
uint16_t daytimeThreshold = 500;  // UI 可配置的阈值（默认 500）
const char* SENSOR_ILLUM_TOPIC = "smarthome/sensor/outloor_illumination"; // 订阅的传感器主题

// ======================= 连接指示颜色 =======================
// WiFi 未连接 / MQTT 未连接：红色；AP 配网：蓝色
const uint8_t CONN_RED_R = 255;
const uint8_t CONN_RED_G = 0;
const uint8_t CONN_RED_B = 0;
const uint8_t CONN_BLUE_R = 0;
const uint8_t CONN_BLUE_G = 0;
const uint8_t CONN_BLUE_B = 255;


// ======================= EEPROM 持久化 =======================
#define EEPROM_SIZE     512
#define EEPROM_MAGIC    0xA7
#define EEPROM_VERSION  4

struct LedConfig {
  uint8_t magic;
  uint8_t version;
  uint8_t onR, onG, onB, onBrightness;
  uint8_t offR, offG, offB, offBrightness;
  char name[24];
  char type[16];
  char wifiSsid[32];
  char wifiPass[64];
  char mqttHost[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[64];
  uint16_t daytime;   // Daytime 阈值
  uint16_t checksum;
};

// 兼容旧版（v3）结构：不含 daytime 字段
struct LedConfigV3 {
  uint8_t magic;
  uint8_t version;
  uint8_t onR, onG, onB, onBrightness;
  uint8_t offR, offG, offB, offBrightness;
  char name[24];
  char type[16];
  char wifiSsid[32];
  char wifiPass[64];
  char mqttHost[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[64];
  uint16_t checksum;
};

LedConfig gLedCfg;

// 调试：最近一次 EEPROM 读取状态（用于网页与串口输出）
bool eepLoadOk = false;
uint8_t eepMagic = 0;
uint8_t eepVersion = 0;
uint16_t eepStoredChecksum = 0;
uint16_t eepComputedChecksum = 0;

uint16_t calcChecksum(const LedConfig &c) {
  uint32_t sum = 0;
  sum += c.magic; sum += c.version;
  sum += c.onR; sum += c.onG; sum += c.onB; sum += c.onBrightness;
  sum += c.offR; sum += c.offG; sum += c.offB; sum += c.offBrightness;
  for (size_t i = 0; i < sizeof(c.name); i++) sum += (uint8_t)c.name[i];
  for (size_t i = 0; i < sizeof(c.type); i++) sum += (uint8_t)c.type[i];
  for (size_t i = 0; i < sizeof(c.wifiSsid); i++) sum += (uint8_t)c.wifiSsid[i];
  for (size_t i = 0; i < sizeof(c.wifiPass); i++) sum += (uint8_t)c.wifiPass[i];
  for (size_t i = 0; i < sizeof(c.mqttHost); i++) sum += (uint8_t)c.mqttHost[i];
  sum += c.mqttPort;
  for (size_t i = 0; i < sizeof(c.mqttUser); i++) sum += (uint8_t)c.mqttUser[i];
  for (size_t i = 0; i < sizeof(c.mqttPass); i++) sum += (uint8_t)c.mqttPass[i];
  sum += c.daytime;
  return (uint16_t)(sum & 0xFFFF);
}

void applyLedConfigToRuntime(const LedConfig &c) {
  ledOnR = c.onR; ledOnG = c.onG; ledOnB = c.onB; ledOnBrightness = c.onBrightness;
  ledOffR = c.offR; ledOffG = c.offG; ledOffB = c.offB; ledOffBrightness = c.offBrightness;
  deviceName = String(c.name);
  deviceType = String(c.type);
  wifiSsid = String(c.wifiSsid);
  wifiPass = String(c.wifiPass);
  mqttHostStr = String(c.mqttHost);
  mqttPortVal = c.mqttPort;
  mqttUser = String(c.mqttUser);
  mqttPass = String(c.mqttPass);
  daytimeThreshold = c.daytime;
  if (daytimeThreshold > 2000) daytimeThreshold = 2000;
  updateLedStates();
}

bool loadLedConfig() {
  EEPROM.get(0, gLedCfg);
  // 记录调试字段
  eepMagic = gLedCfg.magic;
  eepVersion = gLedCfg.version;
  eepStoredChecksum = gLedCfg.checksum;
  eepComputedChecksum = calcChecksum(gLedCfg);
  // 正常路径：版本匹配
  if (gLedCfg.magic == EEPROM_MAGIC && gLedCfg.version == EEPROM_VERSION && calcChecksum(gLedCfg) == gLedCfg.checksum) {
    applyLedConfigToRuntime(gLedCfg);
    eepLoadOk = true;
    Serial.printf("EEPROM load OK: magic=0x%02X ver=%u checksum=%u calc=%u\n", eepMagic, eepVersion, eepStoredChecksum, eepComputedChecksum);
    return true;
  }
  // 容错：若版本匹配但校验失败（可能因旧版容量过小导致未完整提交），尽量恢复 LED 颜色与亮度
  if (gLedCfg.magic == EEPROM_MAGIC && gLedCfg.version == EEPROM_VERSION) {
    // 将已读取的颜色与亮度应用到运行时，以便 UI 读取到近似正确的值
    ledOnR = gLedCfg.onR; ledOnG = gLedCfg.onG; ledOnB = gLedCfg.onB; ledOnBrightness = gLedCfg.onBrightness;
    ledOffR = gLedCfg.offR; ledOffG = gLedCfg.offG; ledOffB = gLedCfg.offB; ledOffBrightness = gLedCfg.offBrightness;
    // 其他字符串配置可能不完整，保持现有运行时默认值
    updateLedStates();
    // 立刻重写一次完整配置，修复校验与持久化
    saveLedConfig();
    eepLoadOk = true;
    Serial.printf("EEPROM fallback: checksum mismatch stored=%u calc=%u; restored LED and rewrote config\n", eepStoredChecksum, eepComputedChecksum);
    return true;
  }
  // 兼容迁移：旧版 v3
  if (gLedCfg.magic == EEPROM_MAGIC && gLedCfg.version == 3) {
    LedConfigV3 oldCfg;
    EEPROM.get(0, oldCfg);
    // 计算旧版校验
    uint32_t sum = 0;
    sum += oldCfg.magic; sum += oldCfg.version;
    sum += oldCfg.onR; sum += oldCfg.onG; sum += oldCfg.onB; sum += oldCfg.onBrightness;
    sum += oldCfg.offR; sum += oldCfg.offG; sum += oldCfg.offB; sum += oldCfg.offBrightness;
    for (size_t i = 0; i < sizeof(oldCfg.name); i++) sum += (uint8_t)oldCfg.name[i];
    for (size_t i = 0; i < sizeof(oldCfg.type); i++) sum += (uint8_t)oldCfg.type[i];
    for (size_t i = 0; i < sizeof(oldCfg.wifiSsid); i++) sum += (uint8_t)oldCfg.wifiSsid[i];
    for (size_t i = 0; i < sizeof(oldCfg.wifiPass); i++) sum += (uint8_t)oldCfg.wifiPass[i];
    for (size_t i = 0; i < sizeof(oldCfg.mqttHost); i++) sum += (uint8_t)oldCfg.mqttHost[i];
    sum += oldCfg.mqttPort;
    for (size_t i = 0; i < sizeof(oldCfg.mqttUser); i++) sum += (uint8_t)oldCfg.mqttUser[i];
    for (size_t i = 0; i < sizeof(oldCfg.mqttPass); i++) sum += (uint8_t)oldCfg.mqttPass[i];
    uint16_t oldChecksum = (uint16_t)(sum & 0xFFFF);
    if (oldChecksum != oldCfg.checksum) {
      return false; // 校验失败，不迁移
    }
    // 组装为新结构并写回 EEPROM
    LedConfig migrated;
    memset(&migrated, 0, sizeof(migrated));
    migrated.magic = EEPROM_MAGIC;
    migrated.version = EEPROM_VERSION;
    migrated.onR = oldCfg.onR; migrated.onG = oldCfg.onG; migrated.onB = oldCfg.onB; migrated.onBrightness = oldCfg.onBrightness;
    migrated.offR = oldCfg.offR; migrated.offG = oldCfg.offG; migrated.offB = oldCfg.offB; migrated.offBrightness = oldCfg.offBrightness;
    memcpy(migrated.name, oldCfg.name, sizeof(migrated.name));
    memcpy(migrated.type, oldCfg.type, sizeof(migrated.type));
    memcpy(migrated.wifiSsid, oldCfg.wifiSsid, sizeof(migrated.wifiSsid));
    memcpy(migrated.wifiPass, oldCfg.wifiPass, sizeof(migrated.wifiPass));
    memcpy(migrated.mqttHost, oldCfg.mqttHost, sizeof(migrated.mqttHost));
    migrated.mqttPort = oldCfg.mqttPort;
    memcpy(migrated.mqttUser, oldCfg.mqttUser, sizeof(migrated.mqttUser));
    memcpy(migrated.mqttPass, oldCfg.mqttPass, sizeof(migrated.mqttPass));
    migrated.daytime = daytimeThreshold; // 迁移时赋默认阈值
    migrated.checksum = calcChecksum(migrated);
    // 写回并应用
    EEPROM.put(0, migrated);
    EEPROM.commit();
    gLedCfg = migrated;
    applyLedConfigToRuntime(gLedCfg);
    eepLoadOk = true;
    Serial.println("EEPROM migrated v3->v4 and applied.");
    return true;
  }
  eepLoadOk = false;
  Serial.printf("EEPROM load FAIL: magic=0x%02X ver=%u checksum=%u calc=%u\n", eepMagic, eepVersion, eepStoredChecksum, eepComputedChecksum);
  return false;
}

void saveLedConfig() {
  gLedCfg.magic = EEPROM_MAGIC;
  gLedCfg.version = EEPROM_VERSION;
  gLedCfg.onR = ledOnR; gLedCfg.onG = ledOnG; gLedCfg.onB = ledOnB; gLedCfg.onBrightness = ledOnBrightness;
  gLedCfg.offR = ledOffR; gLedCfg.offG = ledOffG; gLedCfg.offB = ledOffB; gLedCfg.offBrightness = ledOffBrightness;
  memset(gLedCfg.name, 0, sizeof(gLedCfg.name));
  memset(gLedCfg.type, 0, sizeof(gLedCfg.type));
  memset(gLedCfg.wifiSsid, 0, sizeof(gLedCfg.wifiSsid));
  memset(gLedCfg.wifiPass, 0, sizeof(gLedCfg.wifiPass));
  memset(gLedCfg.mqttHost, 0, sizeof(gLedCfg.mqttHost));
  gLedCfg.mqttPort = mqttPortVal;
  memset(gLedCfg.mqttUser, 0, sizeof(gLedCfg.mqttUser));
  memset(gLedCfg.mqttPass, 0, sizeof(gLedCfg.mqttPass));
  deviceName.toCharArray(gLedCfg.name, sizeof(gLedCfg.name));
  deviceType.toCharArray(gLedCfg.type, sizeof(gLedCfg.type));
  wifiSsid.toCharArray(gLedCfg.wifiSsid, sizeof(gLedCfg.wifiSsid));
  wifiPass.toCharArray(gLedCfg.wifiPass, sizeof(gLedCfg.wifiPass));
  mqttHostStr.toCharArray(gLedCfg.mqttHost, sizeof(gLedCfg.mqttHost));
  mqttUser.toCharArray(gLedCfg.mqttUser, sizeof(gLedCfg.mqttUser));
  mqttPass.toCharArray(gLedCfg.mqttPass, sizeof(gLedCfg.mqttPass));
  gLedCfg.daytime = daytimeThreshold;
  gLedCfg.checksum = calcChecksum(gLedCfg);
  EEPROM.put(0, gLedCfg);
  EEPROM.commit();
  Serial.printf(
    "EEPROM save: on #%02X%02X%02X b=%u, off #%02X%02X%02X b=%u, name=%s, daytime=%u, checksum=%u (size=%u)\n",
    gLedCfg.onR, gLedCfg.onG, gLedCfg.onB, gLedCfg.onBrightness,
    gLedCfg.offR, gLedCfg.offG, gLedCfg.offB, gLedCfg.offBrightness,
    gLedCfg.name, gLedCfg.daytime, gLedCfg.checksum, (unsigned)sizeof(LedConfig)
  );
}

// ======================= 中断函数 =======================
void ICACHE_RAM_ATTR handleInterrupt_0() { buttonPressedIndex = 0; }
void ICACHE_RAM_ATTR handleInterrupt_1() { buttonPressedIndex = 1; }
void ICACHE_RAM_ATTR handleInterrupt_2() { buttonPressedIndex = 2; }

// ======================= 辅助函数 =======================
void attachButtonInterruptForIndex(int idx) {
  switch (idx) {
    case 0: attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[0]), handleInterrupt_0, FALLING); break;
    case 1: attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[1]), handleInterrupt_1, FALLING); break;
    case 2: attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[2]), handleInterrupt_2, FALLING); break;
  }
}

void setRelayState(int channelIndex, bool newState) {
  if (channelIndex < 0 || channelIndex >= NUM_CHANNELS) return;
  relayStates[channelIndex] = newState;
  int physicalIndex = CHANNEL_PIN_MAP[channelIndex];
  digitalWrite(RELAY_PINS[physicalIndex], newState);

  // 在互斥模式下：当打开左端(0)或右端(NUM_CHANNELS-1)通道时，关闭对侧通道
  if (curtainMode && newState && NUM_CHANNELS >= 2) {
    if (channelIndex == 0) {
      int paired = NUM_CHANNELS - 1;
      if (relayStates[paired]) {
        relayStates[paired] = false;
        int pairedPhysical = CHANNEL_PIN_MAP[paired];
        digitalWrite(RELAY_PINS[pairedPhysical], LOW);
      }
    } else if (channelIndex == NUM_CHANNELS - 1) {
      int paired = 0;
      if (relayStates[paired]) {
        relayStates[paired] = false;
        int pairedPhysical = CHANNEL_PIN_MAP[paired];
        digitalWrite(RELAY_PINS[pairedPhysical], LOW);
      }
    }
  }
  updateLedStates();
}

void updateLedStates() {
  // 在 MQTT 连接之前，LED 不对开关状态做反应（由连接指示接管）
  if (!mqttClient.connected()) {
    return;
  }
  pixels.clear();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int physical_index = CHANNEL_PIN_MAP[i];
    if (relayStates[i]) {
      // 继电器 ON -> 使用可配置颜色与亮度
      uint8_t r = (uint16_t)ledOnR * ledOnBrightness / 255;
      uint8_t g = (uint16_t)ledOnG * ledOnBrightness / 255;
      uint8_t b = (uint16_t)ledOnB * ledOnBrightness / 255;
      pixels.setPixelColor(physical_index, pixels.Color(r, g, b));
    } else {
      // 继电器 OFF -> 根据照度与 daytime 阈值决定显示关闭颜色或熄灭
      bool hasIllum = (illuminationValue >= 0);
      bool isDaytime = hasIllum && (illuminationValue > daytimeThreshold);
      if (isDaytime) {
        // 白天：关闭时 LED 熄灭
        pixels.setPixelColor(physical_index, pixels.Color(0, 0, 0));
      } else {
        // 夜间或未知：显示关闭颜色与亮度
        uint8_t r = (uint16_t)ledOffR * ledOffBrightness / 255;
        uint8_t g = (uint16_t)ledOffG * ledOffBrightness / 255;
        uint8_t b = (uint16_t)ledOffB * ledOffBrightness / 255;
        pixels.setPixelColor(physical_index, pixels.Color(r, g, b));
      }
    }
  }
  pixels.show();
}

// ======================= 连接状态指示 =======================
// - WiFi 未连接：第一个 LED 快闪
// - WiFi 已连接但 MQTT 未连接：第一个 LED 慢闪
// - MQTT 连接后：由 updateLedStates 正常渲染
unsigned long lastBlinkMs = 0;
bool blinkOn = false;
const uint16_t FAST_BLINK_MS = 200;  // 快闪 200ms
const uint16_t SLOW_BLINK_MS = 800;  // 慢闪 800ms
// WiFi 健康检查与 AP 回退
unsigned long lastWiFiHealthCheckMs = 0;
unsigned long wifiLostSinceMs = 0;
const unsigned long WIFI_RETRY_WINDOW_MS = 15000; // 断线后尝试 15 秒，失败则开启 AP

void renderConnectivityLed() {
  // 仅在 MQTT 未连接时进行连接状态指示
  if (mqttClient.connected()) {
    return;
  }

  unsigned long now = millis();
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  // AP 配网下使用独立的闪烁周期
  uint16_t interval = apMode ? 400 : (wifiConnected ? SLOW_BLINK_MS : FAST_BLINK_MS);

  if (now - lastBlinkMs >= interval) {
    lastBlinkMs = now;
    blinkOn = !blinkOn;

    // 清空所有 LED，避免在连接前对开关状态有任何显示
    pixels.clear();

    // 使用第一个通道对应的物理 LED 作为指示灯
    int idx0 = CHANNEL_PIN_MAP[0];

    // 指示色：AP 配网 -> 蓝色；否则（未连 WiFi 或未连 MQTT）-> 红色
    uint8_t baseR = apMode ? CONN_BLUE_R : CONN_RED_R;
    uint8_t baseG = apMode ? CONN_BLUE_G : CONN_RED_G;
    uint8_t baseB = apMode ? CONN_BLUE_B : CONN_RED_B;
    uint8_t r = (uint16_t)baseR * ledOnBrightness / 255;
    uint8_t g = (uint16_t)baseG * ledOnBrightness / 255;
    uint8_t b = (uint16_t)baseB * ledOnBrightness / 255;

    if (blinkOn) {
      pixels.setPixelColor(idx0, pixels.Color(r, g, b));
    } else {
      pixels.setPixelColor(idx0, pixels.Color(0, 0, 0));
    }
    pixels.show();
  }
}

// ======================= MQTT 辅助函数 =======================
void mqttPublishFullState();
void publishHaDiscovery();

void computeMqttRootTopic() {
  deviceMacNoColon = WiFi.macAddress();
  deviceMacNoColon.replace(":", "");
  mqttRootTopic = String("smarthome/") + deviceType + "/" + deviceName + "-" + deviceMacNoColon;
}

void mqttOnMessage(char* topic, byte* payload, unsigned int length) {
  // 订阅命令：<root>/cmd/<channel>，payload: on/off/toggle
  String t = String(topic);
  String cmdPrefix = mqttRootTopic + String("/cmd/");
  if (t.startsWith(cmdPrefix)) {
    String chStr = t.substring(cmdPrefix.length());
    int ch = chStr.toInt();
    if (ch >= 0 && ch < NUM_CHANNELS) {
      String p; p.reserve(length + 1);
      for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
      p.trim(); p.toLowerCase();
      Serial.print("MQTT cmd ch="); Serial.print(ch); Serial.print(" payload="); Serial.println(p);
      if (p == "on") {
        setRelayState(ch, true);
      } else if (p == "off") {
        setRelayState(ch, false);
      } else if (p == "toggle") {
        setRelayState(ch, !relayStates[ch]);
      }
      mqttPublishFullState();
      return;
    }
  }
  // 处理 LED On Light 的 JSON 命令：<root>/led_on/set
  {
    String ledOnCmd = mqttRootTopic + String("/led_on/set");
    if (t == ledOnCmd) {
      String p; p.reserve(length + 1);
      for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
      // 解析 brightness
      int brIndex = p.indexOf("\"brightness\"");
      if (brIndex >= 0) {
        int colon = p.indexOf(':', brIndex); if (colon > 0) {
          int i = colon + 1; while (i < (int)p.length() && (p[i] == ' ' || p[i] == '"')) i++;
          int val = 0; while (i < (int)p.length() && isDigit(p[i])) { val = val * 10 + (p[i]-'0'); i++; }
          if (val < 0) val = 0; if (val > 255) val = 255; ledOnBrightness = (uint8_t)val;
        }
      }
      // 解析 color {r,g,b}
      int colIndex = p.indexOf("\"color\"");
      if (colIndex >= 0) {
        // 查找 r
        int rKey = p.indexOf("\"r\"", colIndex);
        int gKey = p.indexOf("\"g\"", colIndex);
        int bKey = p.indexOf("\"b\"", colIndex);
        auto parseComp = [&](int keyPos) -> int {
          if (keyPos < 0) return -1;
          int c = p.indexOf(':', keyPos); if (c < 0) return -1;
          int i = c + 1; while (i < (int)p.length() && (p[i] == ' ' || p[i] == '"')) i++;
          int v = 0; bool any=false; while (i < (int)p.length() && isDigit(p[i])) { v = v*10 + (p[i]-'0'); i++; any=true; }
          if (!any) return -1; if (v < 0) v = 0; if (v > 255) v = 255; return v;
        };
        int r = parseComp(rKey), g = parseComp(gKey), b = parseComp(bKey);
        if (r >= 0 && g >= 0 && b >= 0) { ledOnR = (uint8_t)r; ledOnG = (uint8_t)g; ledOnB = (uint8_t)b; }
      }
      updateLedStates();
      saveLedConfig();
      mqttPublishFullState();
      // 发布 LED On Light 独立状态
      if (mqttClient.connected()) {
        String stTopic = mqttRootTopic + "/led_on/state";
        String json = String("{\"state\":\"ON\",\"brightness\":") + String((int)ledOnBrightness) + ",\"color_mode\":\"rgb\",\"color\":{\"r\":" + String((int)ledOnR) + ",\"g\":" + String((int)ledOnG) + ",\"b\":" + String((int)ledOnB) + "}}";
        mqttClient.publish(stTopic.c_str(), json.c_str(), true);
      }
      return;
    }
  }
  // 处理 LED Off Light 的 JSON 命令：<root>/led_off/set
  {
    String ledOffCmd = mqttRootTopic + String("/led_off/set");
    if (t == ledOffCmd) {
      String p; p.reserve(length + 1);
      for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
      int brIndex = p.indexOf("\"brightness\"");
      if (brIndex >= 0) {
        int colon = p.indexOf(':', brIndex); if (colon > 0) {
          int i = colon + 1; while (i < (int)p.length() && (p[i] == ' ' || p[i] == '"')) i++;
          int val = 0; while (i < (int)p.length() && isDigit(p[i])) { val = val * 10 + (p[i]-'0'); i++; }
          if (val < 0) val = 0; if (val > 255) val = 255; ledOffBrightness = (uint8_t)val;
        }
      }
      int colIndex = p.indexOf("\"color\"");
      if (colIndex >= 0) {
        int rKey = p.indexOf("\"r\"", colIndex);
        int gKey = p.indexOf("\"g\"", colIndex);
        int bKey = p.indexOf("\"b\"", colIndex);
        auto parseComp = [&](int keyPos) -> int {
          if (keyPos < 0) return -1;
          int c = p.indexOf(':', keyPos); if (c < 0) return -1;
          int i = c + 1; while (i < (int)p.length() && (p[i] == ' ' || p[i] == '"')) i++;
          int v = 0; bool any=false; while (i < (int)p.length() && isDigit(p[i])) { v = v*10 + (p[i]-'0'); i++; any=true; }
          if (!any) return -1; if (v < 0) v = 0; if (v > 255) v = 255; return v;
        };
        int r = parseComp(rKey), g = parseComp(gKey), b = parseComp(bKey);
        if (r >= 0 && g >= 0 && b >= 0) { ledOffR = (uint8_t)r; ledOffG = (uint8_t)g; ledOffB = (uint8_t)b; }
      }
      updateLedStates();
      saveLedConfig();
      mqttPublishFullState();
      if (mqttClient.connected()) {
        String stTopic = mqttRootTopic + "/led_off/state";
        String json = String("{\"state\":\"ON\",\"brightness\":") + String((int)ledOffBrightness) + ",\"color_mode\":\"rgb\",\"color\":{\"r\":" + String((int)ledOffR) + ",\"g\":" + String((int)ledOffG) + ",\"b\":" + String((int)ledOffB) + "}}";
        mqttClient.publish(stTopic.c_str(), json.c_str(), true);
      }
      return;
    }
  }
  // 处理 Daytime 阈值设置命令：<root>/daytime/set（payload 为数字）
  {
    String daytimeCmd = mqttRootTopic + String("/daytime/set");
    if (t == daytimeCmd) {
      String p; p.reserve(length + 1);
      for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
      p.trim();
      long v = p.toInt();
      if (v < 0) v = 0; if (v > 2000) v = 2000;
      daytimeThreshold = (uint16_t)v;
      saveLedConfig();
      updateLedStates();
      mqttPublishFullState();
      if (mqttClient.connected()) {
        String stTopic = mqttRootTopic + "/daytime/state";
        String s = String((unsigned)daytimeThreshold);
        mqttClient.publish(stTopic.c_str(), s.c_str(), true);
      }
      return;
    }
  }
  // 打印其他消息以便调试
  // 处理照度传感器消息
  if (t == String(SENSOR_ILLUM_TOPIC)) {
    String p; p.reserve(length + 1);
    for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
    p.trim();
    double v = p.toFloat();
    long iv = (long)v;
    illuminationValue = iv;
    Serial.print("Illumination updated: "); Serial.println(illuminationValue);
    updateLedStates();
    return;
  }
  Serial.print("MQTT msg "); Serial.print(topic); Serial.print(" len="); Serial.println(length);
}

void mqttSetup() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttOnMessage);
  mqttClient.setBufferSize(512); // 适当增大以容纳状态 JSON
}

void mqttEnsureConnected() {
  // 始终应用最新的 MQTT 服务器配置
  mqttClient.setServer(mqttHostStr.c_str(), mqttPortVal);
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > 3000) {
      lastMqttReconnectAttempt = now;
      String clientId = deviceType + "-" + deviceName + "-" + deviceMacNoColon;
      String statusTopic = mqttRootTopic + "/status";
      // 使用 LWT 在断线时自动发布 offline
      bool connected = false;
      if (mqttUser.length() > 0 || mqttPass.length() > 0) {
        connected = mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str(), statusTopic.c_str(), 0, true, "offline");
      } else {
        connected = mqttClient.connect(clientId.c_str(), statusTopic.c_str(), 0, true, "offline");
      }
      if (connected) {
        mqttClient.publish(statusTopic.c_str(), "online", true);
        // 订阅命令主题
        String cmdTopic = mqttRootTopic + "/cmd/#";
        mqttClient.subscribe(cmdTopic.c_str());
        // 订阅 Light 命令主题（JSON schema）
        String ledOnCmd = mqttRootTopic + "/led_on/set";
        String ledOffCmd = mqttRootTopic + "/led_off/set";
        mqttClient.subscribe(ledOnCmd.c_str());
        mqttClient.subscribe(ledOffCmd.c_str());
        // 订阅 Daytime 阈值设置主题（MQTT number）
        String daytimeCmd = mqttRootTopic + "/daytime/set";
        mqttClient.subscribe(daytimeCmd.c_str());
        // 订阅照度传感器主题
        mqttClient.subscribe(SENSOR_ILLUM_TOPIC);
        Serial.print("MQTT subscribed: "); Serial.println(cmdTopic);
        Serial.print("MQTT subscribed sensor: "); Serial.println(SENSOR_ILLUM_TOPIC);
        Serial.println("MQTT connected");
        // 仅手动触发 HA 发现，不在连接时自动发送
        mqttPublishFullState();
        // MQTT 已连接，恢复 LED 按开关状态正常渲染
        updateLedStates();
        // 发布两个 Light 的当前状态，便于 HA 立即同步
        String ledOnSt = mqttRootTopic + "/led_on/state";
        String ledOnJson = String("{\"state\":\"ON\",\"brightness\":") + String((int)ledOnBrightness) + ",\"color_mode\":\"rgb\",\"color\":{\"r\":" + String((int)ledOnR) + ",\"g\":" + String((int)ledOnG) + ",\"b\":" + String((int)ledOnB) + "}}";
        mqttClient.publish(ledOnSt.c_str(), ledOnJson.c_str(), true);
        String ledOffSt = mqttRootTopic + "/led_off/state";
        String ledOffJson = String("{\"state\":\"ON\",\"brightness\":") + String((int)ledOffBrightness) + ",\"color_mode\":\"rgb\",\"color\":{\"r\":" + String((int)ledOffR) + ",\"g\":" + String((int)ledOffG) + ",\"b\":" + String((int)ledOffB) + "}}";
        mqttClient.publish(ledOffSt.c_str(), ledOffJson.c_str(), true);
        // 发布 Daytime 阈值当前状态（便于 HA 立即同步）
        String daytimeSt = mqttRootTopic + "/daytime/state";
        String daytimeStr = String((unsigned)daytimeThreshold);
        mqttClient.publish(daytimeSt.c_str(), daytimeStr.c_str(), true);
      } else {
        Serial.print("MQTT connect failed, rc=");
        Serial.println(mqttClient.state());
      }
    }
  }
}

void mqttPublishFullState() {
  if (!mqttClient.connected()) return;
  String topic = mqttRootTopic + "/state";
  // 构造紧凑状态：ch0/ch1、curtain、on/off 颜色与亮度
  char onHex[10]; snprintf(onHex, sizeof(onHex), "#%02X%02X%02X", ledOnR, ledOnG, ledOnB);
  char offHex[10]; snprintf(offHex, sizeof(offHex), "#%02X%02X%02X", ledOffR, ledOffG, ledOffB);
  String payload = "{";
  payload += "\"name\":\""; payload += deviceName; payload += "\",";
  payload += "\"type\":\""; payload += deviceType; payload += "\",";
  payload += "\"mac\":\""; payload += deviceMacNoColon; payload += "\",";
  payload += "\"channels\":" + String(NUM_CHANNELS) + ",";
  payload += "\"states\":[";
  for (int i = 0; i < NUM_CHANNELS; i++) {
    payload += relayStates[i] ? "\"on\"" : "\"off\"";
    if (i < NUM_CHANNELS - 1) payload += ",";
  }
  payload += "],\"curtain\":"; payload += curtainMode ? "true" : "false";
  payload += ",\"led\":{\"color\":\""; payload += String(onHex);
  payload += "\",\"brightness\":" + String(ledOnBrightness) + ",\"offColor\":\"";
  payload += String(offHex) + "\",\"offBrightness\":" + String(ledOffBrightness) + "}}";
  mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

// 发布 Home Assistant 发现配置（每个通道一个 switch）
void publishHaDiscovery() {
  if (!mqttClient.connected()) return;
  String nodeId = sanitizeId(deviceName + String("-") + deviceMacNoColon, 64);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    String objId = String("ch") + String(i);
    String cfgTopic = String("homeassistant/switch/") + nodeId + "/" + objId + "/config";
    String cfg = "{\"~\":\"" + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + deviceName + " CH " + String(i + 1) + "\",";
    cfg += "\"uniq_id\":\"" + deviceMacNoColon + "-ch" + String(i) + "\",";
    cfg += "\"stat_t\":\"~/state\",";
    cfg += "\"val_tpl\":\"{{ value_json.states[" + String(i) + "] }}\",";
    cfg += "\"state_on\":\"on\",\"state_off\":\"off\",";
    cfg += "\"cmd_t\":\"~/cmd/" + String(i) + "\",\"pl_on\":\"on\",\"pl_off\":\"off\",";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += "\"device\":{\"identifiers\":[\"" + deviceMacNoColon + "\"],\"name\":\"" + deviceName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"switch_example\"}}";
    mqttClient.publish(cfgTopic.c_str(), cfg.c_str(), true);
  }

  // 发布两个可控 Light（JSON schema），用于配置开/关态 LED 的颜色与亮度
  {
    String cfgTopic = String("homeassistant/light/") + nodeId + "/led_on/config";
    String cfg = "{\"~\":\"" + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + deviceName + " LED On\",";
    cfg += "\"uniq_id\":\"" + deviceMacNoColon + "-led-on\",";
    cfg += "\"schema\":\"json\",";
    cfg += "\"cmd_t\":\"~/led_on/set\",";
    cfg += "\"stat_t\":\"~/led_on/state\",";
    cfg += "\"brightness\":true,";
    cfg += "\"color_mode\":true,";
    cfg += "\"supported_color_modes\":[\"rgb\"],";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += "\"device\":{\"identifiers\":[\"" + deviceMacNoColon + "\"],\"name\":\"" + deviceName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"switch_example\"}}";
    mqttClient.publish(cfgTopic.c_str(), cfg.c_str(), true);
  }
  {
    String cfgTopic = String("homeassistant/light/") + nodeId + "/led_off/config";
    String cfg = "{\"~\":\"" + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + deviceName + " LED Off\",";
    cfg += "\"uniq_id\":\"" + deviceMacNoColon + "-led-off\",";
    cfg += "\"schema\":\"json\",";
    cfg += "\"cmd_t\":\"~/led_off/set\",";
    cfg += "\"stat_t\":\"~/led_off/state\",";
    cfg += "\"brightness\":true,";
    cfg += "\"color_mode\":true,";
    cfg += "\"supported_color_modes\":[\"rgb\"],";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += "\"device\":{\"identifiers\":[\"" + deviceMacNoColon + "\"],\"name\":\"" + deviceName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"switch_example\"}}";
    mqttClient.publish(cfgTopic.c_str(), cfg.c_str(), true);
  }
  // 发布 Daytime 阈值的 MQTT number 发现（可读写）
  {
    String cfgTopic = String("homeassistant/number/") + nodeId + "/daytime/config";
    String cfg = "{\"~\":\"" + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + deviceName + " Daytime 阈值\",";
    cfg += "\"uniq_id\":\"" + deviceMacNoColon + "-daytime\",";
    cfg += "\"cmd_t\":\"~/daytime/set\",";
    cfg += "\"stat_t\":\"~/daytime/state\",";
    cfg += "\"min\":0,\"max\":2000,\"step\":1,\"mode\":\"slider\",";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += "\"device\":{\"identifiers\":[\"" + deviceMacNoColon + "\"],\"name\":\"" + deviceName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"switch_example\"}}";
    mqttClient.publish(cfgTopic.c_str(), cfg.c_str(), true);
  }
  // 清理旧的传感器实体：发布空配置（retain）以删除
  {
    String delOn = String("homeassistant/sensor/") + nodeId + "/led_on/config";
    mqttClient.publish(delOn.c_str(), "", true);
    String delOff = String("homeassistant/sensor/") + nodeId + "/led_off/config";
    mqttClient.publish(delOff.c_str(), "", true);
  }
}

// ======================= WiFi 与 WebServer =======================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  // 先设置 DHCP 主机名为设备名（路由器显示名）
  String hn = sanitizeId(deviceName, 24);
  if (hn.length() == 0) {
    String mac = WiFi.macAddress(); mac.replace(":", "");
    hn = String("ESP-") + mac.substring(mac.length() - 6);
  }
  WiFi.hostname(hn.c_str());
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  Serial.printf("Connecting to %s", wifiSsid.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    // 避免阻塞期间触发硬件看门狗复位
    ESP.wdtFeed();
    yield();
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("DHCP hostname: "); Serial.println(hn);
    apMode = false;
  } else {
    Serial.println("WiFi connect failed, starting AP for config...");
    // 启动 AP 以便手机/电脑连接配置 WiFi
    String apSsid = sanitizeId(deviceName, 24);
    if (apSsid.length() == 0) apSsid = "esp-switch";
    apSsid += "-setup";
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsid.c_str());
    apMode = true;
    Serial.print("AP SSID: "); Serial.println(apSsid);
  }
}

String buildHtml() {
  String html = "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP8266 开关面板</title>";
  html += "<style>body{font-family:system-ui,Arial;margin:16px;}h1{font-size:18px;}";
  html += ".row{display:flex;gap:8px;align-items:center;margin:8px 0;}button{padding:6px 10px;}";
  html += "#status{margin:12px 0;padding:8px;background:#f4f4f4;border-radius:6px;}";
  html += ".ip{color:#666;font-size:12px;}</style></head><body>";
  html += "<h1>ESP8266 开关面板</h1><div id='status'>载入中...</div>";
  // 设备信息
  html += "<div class='row'><label>设备名称: <input type='text' id='devName' style='width:160px'></label>";
  html += "<label style='margin-left:12px'>设备类型: <input type='text' id='devType' style='width:120px'></label>";
  html += "<button onclick=\"setDevice()\">保存</button></div>";
  // 显示 MQTT 根主题
  html += "<div class='row'><span>MQTT主题: <code id='mqttRoot'></code></span></div>";
  // 显示状态与命令主题
  html += "<div class='row'><span>状态主题: 在线 <code id='mqttStatus'></code> ，全量 <code id='mqttState'></code></span></div>";
  html += "<div class='row'><span>命令主题: <code id='mqttCmd'></code></span></div>";
  // 显示 WiFi 连接信息
  html += "<div class='row'><span>WiFi: SSID <code id='wifiSsid'></code> ，信号 <code id='wifiRssi'></code> dBm</span></div>";
  // WiFi 配网（AP 模式或需要更改凭据时）
  html += "<div class='row'><label>WiFi SSID: <input type='text' id='cfgSsid' style='width:160px'></label>";
  html += "<label style='margin-left:12px'>密码: <input type='password' id='cfgPass' style='width:160px'></label>";
  html += "<button onclick=\"setWifi()\">保存WiFi</button><span class='ip'>保存后设备将尝试连接路由器</span></div>";
  // MQTT 配置（主机、端口、账号）
  html += "<div class='row'><label>MQTT主机: <input type='text' id='cfgMqttHost' style='width:160px' placeholder='例如 192.168.1.10'></label>";
  html += "<label style='margin-left:12px'>端口: <input type='number' id='cfgMqttPort' style='width:100px' min='1' max='65535' value='1883'></label>";
  html += "<label style='margin-left:12px'>用户: <input type='text' id='cfgMqttUser' style='width:140px'></label>";
  html += "<label style='margin-left:12px'>密码: <input type='password' id='cfgMqttPass' style='width:140px' placeholder='不显示'></label>";
  html += "<button onclick=\"setMqtt()\">保存MQTT</button><span class='ip'>保存后设备将重连 MQTT</span></div>";
  // 手动重发 HA 自动发现
  html += "<div class='row'><button onclick=\"sendHa()\">重发 HA 发现</button><span class='ip'>用于 Home Assistant 自动发现</span></div>";
  // LED 打开颜色与亮度设置
  html += "<div class='row'><label>LED打开颜色: <input type='color' id='ledColor' onchange=\"setLed()\"></label>";
  html += "<label style='margin-left:12px'>亮度: <input type='range' id='ledBright' min='0' max='255' step='1' oninput=\"setLed()\"> <code id='ledBrightVal' class='ip'>0</code></label></div>";
  // LED 关闭颜色设置
  html += "<div class='row'><label>LED关闭颜色: <input type='color' id='ledOffColor' onchange=\"setLedOff()\"></label>";
  html += "<label style='margin-left:12px'>亮度: <input type='range' id='ledOffBright' min='0' max='255' step='1' oninput=\"setLedOff()\"> <code id='ledOffBrightVal' class='ip'>0</code></label></div>";
  // Daytime 阈值与照度显示
  html += "<div class='row'><label>白天阈值(daytime): <input type='number' id='cfgDaytime' style='width:100px' min='0' max='2000' step='1'></label>";
  html += "<button style='margin-left:8px' onclick=\"setDaytime()\">保存Daytime</button>";
  html += "<span style='margin-left:12px' class='ip'>最近照度: <code id='illumValue'>未知</code></span></div>";
  // 互斥窗帘模式切换
  html += "<div class='row'><label><input type='checkbox' id='curtain'";
  if (curtainMode) html += " checked";
  html += " onchange=\"setCurtain(this.checked)\">窗帘互斥模式</label></div>";
  for (int i = 0; i < NUM_CHANNELS; i++) {
    html += "<div class='row'><b>通道 ";
    html += String(i);
    html += "</b>";
    html += "<button onclick=\"toggle(" + String(i) + ")\">切换</button>";
    html += "<button onclick=\"setOn(" + String(i) + ")\">开</button>";
    html += "<button onclick=\"setOff(" + String(i) + ")\">关</button></div>";
  }
  html += "<script>";
  html += "function refresh(){fetch('/api/state').then(r=>r.json()).then(j=>{var ae=(document.activeElement&&document.activeElement.id)||'';document.getElementById('status').innerText='状态: '+j.states.join(', ')+' | 互斥: '+(j.curtain?'开':'关');var el=document.getElementById('curtain');if(el){el.checked=j.curtain;}var dn=document.getElementById('devName');var dt=document.getElementById('devType');if(dn&&j.name&&ae!=='devName'){dn.value=j.name;}if(dt&&j.type&&ae!=='devType'){dt.value=j.type;}var mr=document.getElementById('mqttRoot');if(mr&&j.mqttRoot){mr.innerText=j.mqttRoot;}var ts=document.getElementById('mqttStatus');var st=document.getElementById('mqttState');var cm=document.getElementById('mqttCmd');if(j.topics){if(ts&&j.topics.status){ts.innerText=j.topics.status;}if(st&&j.topics.state){st.innerText=j.topics.state;}if(cm&&j.topics.cmd){cm.innerText=j.topics.cmd;}}else{if(ts&&j.mqttRoot){ts.innerText=j.mqttRoot+'/status';}if(st&&j.mqttRoot){st.innerText=j.mqttRoot+'/state';}if(cm&&j.mqttRoot){cm.innerText=j.mqttRoot+'/cmd';}}var ws=document.getElementById('wifiSsid');var wr=document.getElementById('wifiRssi');if(ws&&j.wifi&&j.wifi.ssid){ws.innerText=j.wifi.ssid;}if(wr&&j.wifi&&typeof j.wifi.rssi!=='undefined'){wr.innerText=j.wifi.rssi;}var mh=document.getElementById('cfgMqttHost');var mp=document.getElementById('cfgMqttPort');var mu=document.getElementById('cfgMqttUser');if(mh&&j.mqtt&&j.mqtt.host&&ae!=='cfgMqttHost'){mh.value=j.mqtt.host;}if(mp&&j.mqtt&&typeof j.mqtt.port!=='undefined'&&ae!=='cfgMqttPort'){mp.value=j.mqtt.port;}if(mu&&j.mqtt&&typeof j.mqtt.user!=='undefined'&&ae!=='cfgMqttUser'){mu.value=j.mqtt.user;}var lc=document.getElementById('ledColor');var lb=document.getElementById('ledBright');var lo=document.getElementById('ledOffColor');var lob=document.getElementById('ledOffBright');if(lc&&j.led&&j.led.color&&ae!=='ledColor'){lc.value=j.led.color;}if(lb&&j.led&&typeof j.led.brightness!=='undefined'&&ae!=='ledBright'){lb.value=j.led.brightness;}if(lo&&j.led&&j.led.offColor&&ae!=='ledOffColor'){lo.value=j.led.offColor;}if(lob&&j.led&&typeof j.led.offBrightness!=='undefined'&&ae!=='ledOffBright'){lob.value=j.led.offBrightness;}syncBrightness();});}";
  html += "function toggle(ch){fetch('/api/toggle?ch='+ch).then(()=>refresh());}";
  html += "function setOn(ch){fetch('/api/on?ch='+ch).then(()=>refresh());}";
  html += "function setOff(ch){fetch('/api/off?ch='+ch).then(()=>refresh());}";
  html += "function setLed(){var c=document.getElementById('ledColor').value;var b=document.getElementById('ledBright').value;syncBrightness();fetch('/api/led?color='+encodeURIComponent(c)+'&bright='+b).then(()=>refresh());}";
  html += "function setLedOff(){var c=document.getElementById('ledOffColor').value;var b=document.getElementById('ledOffBright').value;syncBrightness();fetch('/api/led_off?color='+encodeURIComponent(c)+'&bright='+b).then(()=>refresh());}";
  html += "function setDevice(){var n=document.getElementById('devName').value;var t=document.getElementById('devType').value;fetch('/api/device?name='+encodeURIComponent(n)+'&type='+encodeURIComponent(t)).then(()=>refresh());}";
  html += "function setCurtain(v){fetch('/api/mode?curtain='+(v?1:0)).then(()=>refresh());}";
  html += "function sendHa(){fetch('/api/ha_discovery').then(()=>refresh());}";
  html += "function setMqtt(){var h=document.getElementById('cfgMqttHost').value;var p=document.getElementById('cfgMqttPort').value;var u=document.getElementById('cfgMqttUser').value;var pw=document.getElementById('cfgMqttPass').value;fetch('/api/mqtt_config?host='+encodeURIComponent(h)+'&port='+encodeURIComponent(p)+'&user='+encodeURIComponent(u)+'&pass='+encodeURIComponent(pw)).then(()=>setTimeout(refresh,800));}";
  html += "function setWifi(){var s=document.getElementById('cfgSsid').value;var p=document.getElementById('cfgPass').value;fetch('/api/wifi_config?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)).then(()=>setTimeout(refresh,800));}";
  html += "function setDaytime(){var d=document.getElementById('cfgDaytime').value;fetch('/api/daytime?value='+encodeURIComponent(d)).then(()=>refresh());}";
  html += "function syncBrightness(){var b=document.getElementById('ledBright');var bv=document.getElementById('ledBrightVal');var ob=document.getElementById('ledOffBright');var obv=document.getElementById('ledOffBrightVal');if(b&&bv){bv.innerText=b.value;}if(ob&&obv){obv.innerText=ob.value;}}";
  html += "function refreshIllum(){fetch('/api/state').then(r=>r.json()).then(j=>{var ae=(document.activeElement&&document.activeElement.id)||'';var dd=document.getElementById('cfgDaytime');if(dd&&typeof j.daytime!=='undefined'&&ae!=='cfgDaytime'){dd.value=j.daytime;}var iv=document.getElementById('illumValue');if(iv){iv.innerText=(typeof j.illum!=='undefined'&&j.illum>=0)?j.illum:'未知';}var d=j.debug; if(d){var de=document.getElementById('dbgEepSize'); if(de) de.innerText=d.eepSize; var dc=document.getElementById('dbgCfgSize'); if(dc) dc.innerText=d.cfgSize; var dm=document.getElementById('dbgMagic'); if(dm) dm.innerText='0x'+(Number(d.magic).toString(16)).toUpperCase(); var dv=document.getElementById('dbgVer'); if(dv) dv.innerText=d.version; var ds=document.getElementById('dbgChk'); if(ds) ds.innerText=d.checksum; var dl=document.getElementById('dbgCalc'); if(dl) dl.innerText=d.calc; var dk=document.getElementById('dbgLoad'); if(dk) dk.innerText=d.loadOk?'OK':'FAIL';}});}";
  html += "setInterval(refreshIllum,2500);";
  html += "setInterval(refresh,2000);refresh();";
  html += "</script></body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", buildHtml());
}

void handleGetState() {
  String json = "{\"channels\":" + String(NUM_CHANNELS) + ",\"states\":[";
  for (int i = 0; i < NUM_CHANNELS; i++) {
    json += relayStates[i] ? "\"on\"" : "\"off\"";
    if (i < NUM_CHANNELS - 1) json += ",";
  }
  json += "],\"curtain\":";
  json += curtainMode ? "true" : "false";
  // 设备信息
  json += ",\"name\":\"";
  json += deviceName;
  json += "\",\"type\":\"";
  json += deviceType;
  json += "\",\"mac\":\"";
  json += deviceMacNoColon;
  json += "\",\"mqttRoot\":\"";
  json += mqttRootTopic;
  json += "\"";
  // 完整主题对象
  json += ",\"topics\":{\"root\":\""; json += mqttRootTopic; json += "\",\"status\":\""; json += mqttRootTopic; json += "/status\",\"state\":\""; json += mqttRootTopic; json += "/state\",\"cmd\":\""; json += mqttRootTopic; json += "/cmd\"}";
  // MQTT 配置
  json += ",\"mqtt\":{\"host\":\""; json += mqttHostStr; json += "\",\"port\":"; json += String(mqttPortVal); json += ",\"user\":\""; json += mqttUser; json += "\"}";
  // WiFi 信息
  json += ",\"wifi\":{\"ssid\":\""; json += WiFi.SSID(); json += "\",\"rssi\":"; json += String(WiFi.RSSI()); json += "}";
  // LED 配置
  char hexbuf[10];
  snprintf(hexbuf, sizeof(hexbuf), "#%02X%02X%02X", ledOnR, ledOnG, ledOnB);
  json += ",\"led\":{\"color\":\"";
  json += String(hexbuf);
  json += "\",\"brightness\":" + String(ledOnBrightness);
  char hexbuf2[10];
  snprintf(hexbuf2, sizeof(hexbuf2), "#%02X%02X%02X", ledOffR, ledOffG, ledOffB);
  json += ",\"offColor\":\"";
  json += String(hexbuf2);
  json += "\",\"offBrightness\":" + String(ledOffBrightness);
  json += "},\"daytime\":"; json += String(daytimeThreshold);
  json += ",\"illum\":"; json += String(illuminationValue);
  // 调试对象：EEPROM 与配置加载信息
  json += ",\"debug\":{\"eepSize\":"; json += String(EEPROM_SIZE);
  json += ",\"cfgSize\":"; json += String(sizeof(LedConfig));
  json += ",\"magic\":"; json += String(eepMagic);
  json += ",\"version\":"; json += String(eepVersion);
  json += ",\"checksum\":"; json += String(eepStoredChecksum);
  json += ",\"calc\":"; json += String(eepComputedChecksum);
  json += ",\"loadOk\":"; json += (eepLoadOk ? "true" : "false");
  json += "}";
  json += "}";
  server.send(200, "application/json", json);
}

bool parseChannelArg(int& chOut) {
  if (!server.hasArg("ch")) return false;
  int ch = server.arg("ch").toInt();
  if (ch < 0 || ch >= NUM_CHANNELS) return false;
  chOut = ch;
  return true;
}

void handleToggle() {
  int ch;
  if (!parseChannelArg(ch)) { server.send(400, "application/json", "{\"error\":\"invalid ch\"}"); return; }
  setRelayState(ch, !relayStates[ch]);
  server.send(200, "application/json", "{\"ok\":true}");
  mqttPublishFullState();
}

void handleOn() {
  int ch;
  if (!parseChannelArg(ch)) { server.send(400, "application/json", "{\"error\":\"invalid ch\"}"); return; }
  setRelayState(ch, true);
  server.send(200, "application/json", "{\"ok\":true}");
  mqttPublishFullState();
}

void handleOff() {
  int ch;
  if (!parseChannelArg(ch)) { server.send(400, "application/json", "{\"error\":\"invalid ch\"}"); return; }
  setRelayState(ch, false);
  server.send(200, "application/json", "{\"ok\":true}");
  mqttPublishFullState();
}

void handleSetMode() {
  // 支持 0/1/true/false/on/off
  if (!server.hasArg("curtain")) { server.send(400, "application/json", "{\"error\":\"missing curtain\"}"); return; }
  String v = server.arg("curtain");
  v.toLowerCase();
  if (v == "1" || v == "true" || v == "on") curtainMode = true;
  else if (v == "0" || v == "false" || v == "off") curtainMode = false;
  server.send(200, "application/json", String("{\"ok\":true,\"curtain\":") + (curtainMode?"true":"false") + "}");
  mqttPublishFullState();
}

// 解析 #RRGGBB 或 RRGGBB
bool parseColorHex(String s, uint8_t &r, uint8_t &g, uint8_t &b) {
  s.trim();
  if (s.length() == 7 && s.charAt(0) == '#') s = s.substring(1);
  if (s.length() != 6) return false;
  char buf[7];
  s.toCharArray(buf, 7);
  unsigned long v = strtoul(buf, nullptr, 16);
  r = (v >> 16) & 0xFF;
  g = (v >> 8) & 0xFF;
  b = v & 0xFF;
  return true;
}

void handleSetLed() {
  bool changed = false;
  if (server.hasArg("color")) {
    String c = server.arg("color");
    uint8_t r,g,b; if (parseColorHex(c, r, g, b)) { ledOnR=r; ledOnG=g; ledOnB=b; changed = true; }
  }
  if (server.hasArg("bright")) {
    int br = server.arg("bright").toInt();
    if (br < 0) br = 0; if (br > 255) br = 255;
    ledOnBrightness = (uint8_t)br; changed = true;
  }
  if (changed) updateLedStates();
  if (changed) saveLedConfig();
  if (changed) mqttPublishFullState();
  char hexbuf[10];
  snprintf(hexbuf, sizeof(hexbuf), "#%02X%02X%02X", ledOnR, ledOnG, ledOnB);
  String resp = String("{\"ok\":true,\"color\":\"") + String(hexbuf) + "\",\"brightness\":" + String(ledOnBrightness) + "}";
  server.send(200, "application/json", resp);
}

void handleSetLedOff() {
  if (!server.hasArg("color")) { server.send(400, "application/json", "{\"error\":\"missing color\"}"); return; }
  String c = server.arg("color");
  uint8_t r,g,b; if (!parseColorHex(c, r, g, b)) { server.send(400, "application/json", "{\"error\":\"invalid color\"}"); return; }
  ledOffR = r; ledOffG = g; ledOffB = b;
  if (server.hasArg("bright")) {
    int br = server.arg("bright").toInt();
    if (br < 0) br = 0; if (br > 255) br = 255;
    ledOffBrightness = (uint8_t)br;
  }
  updateLedStates();
  saveLedConfig();
  mqttPublishFullState();
  char hexbuf[10];
  snprintf(hexbuf, sizeof(hexbuf), "#%02X%02X%02X", ledOffR, ledOffG, ledOffB);
  String resp = String("{\"ok\":true,\"offColor\":\"") + String(hexbuf) + "\",\"offBrightness\":" + String(ledOffBrightness) + "}";
  server.send(200, "application/json", resp);
}

void handleSetDaytime() {
  if (!server.hasArg("value")) { server.send(400, "application/json", "{\"error\":\"missing value\"}"); return; }
  int v = server.arg("value").toInt();
  if (v < 0) v = 0; if (v > 2000) v = 2000;
  daytimeThreshold = (uint16_t)v;
  saveLedConfig();
  updateLedStates();
  mqttPublishFullState();
  // 同步到 MQTT number 的状态主题
  if (mqttClient.connected()) {
    String stTopic = mqttRootTopic + "/daytime/state";
    String s = String((unsigned)daytimeThreshold);
    mqttClient.publish(stTopic.c_str(), s.c_str(), true);
  }
  server.send(200, "application/json", String("{\"ok\":true,\"daytime\":") + String(daytimeThreshold) + "}");
}

void handleSetWifi() {
  if (!server.hasArg("ssid")) { server.send(400, "application/json", "{\"error\":\"missing ssid\"}"); return; }
  String s = server.arg("ssid"); s.trim();
  String p = server.hasArg("pass") ? server.arg("pass") : String("");
  if (s.length() == 0) { server.send(400, "application/json", "{\"error\":\"empty ssid\"}"); return; }

  wifiSsid = s; wifiPass = p;
  saveLedConfig();

  // 关闭 AP 并重连 WiFi
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(200);
  apMode = false;
  connectWiFi();

  // 应用新的网络后，尝试重连 MQTT 并更新状态
  computeMqttRootTopic();
  mqttEnsureConnected();
  mqttPublishFullState();

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSetMqtt() {
  if (!server.hasArg("host")) { server.send(400, "application/json", "{\"error\":\"missing host\"}"); return; }
  String h = server.arg("host"); h.trim();
  String portStr = server.hasArg("port") ? server.arg("port") : String("");
  uint16_t p = portStr.length() ? (uint16_t)portStr.toInt() : 1883;
  String u = server.hasArg("user") ? server.arg("user") : String("");
  String pw = server.hasArg("pass") ? server.arg("pass") : String("");
  if (h.length() == 0) { server.send(400, "application/json", "{\"error\":\"empty host\"}"); return; }

  mqttHostStr = h; mqttPortVal = p; mqttUser = u; mqttPass = pw;
  saveLedConfig();

  // 应用新的 MQTT 设置并尝试重连
  if (mqttClient.connected()) mqttClient.disconnect();
  mqttClient.setServer(mqttHostStr.c_str(), mqttPortVal);
  mqttEnsureConnected();
  mqttPublishFullState();

  server.send(200, "application/json", "{\"ok\":true}");
}

String sanitizeId(String s, size_t maxLen) {
  String out = "";
  for (size_t i = 0; i < s.length() && out.length() < maxLen; i++) {
    char c = s.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c=='-' || c=='_' ) {
      out += c;
    }
  }
  return out;
}

void handleSetDevice() {
  bool changed = false;
  if (server.hasArg("name")) {
    String n = server.arg("name"); n.trim();
    String nn = sanitizeId(n, sizeof(gLedCfg.name)-1);
    if (nn.length() > 0) { deviceName = nn; changed = true; }
  }
  if (server.hasArg("type")) {
    String t = server.arg("type"); t.trim();
    String tt = sanitizeId(t, sizeof(gLedCfg.type)-1);
    if (tt.length() > 0) { deviceType = tt; changed = true; }
  }
  if (changed) {
    saveLedConfig();
    computeMqttRootTopic();
    // 应用新的 DHCP 主机名并重启 WiFi，路由器将显示设备名
    String hn = sanitizeId(deviceName, 24);
    if (hn.length() == 0) hn = "esp-switch";
    WiFi.hostname(hn.c_str());
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect();
      delay(200);
      connectWiFi();
    }
    // 重启 mDNS 使用新的设备名
    String mdnsHost = hn; mdnsHost.toLowerCase();
    if (MDNS.begin(mdnsHost.c_str())) {
      MDNS.addService("http", "tcp", 80);
      Serial.print("mDNS: http://"); Serial.print(mdnsHost); Serial.println(".local");
    }
    if (mqttClient.connected()) mqttClient.disconnect();
    mqttEnsureConnected();
    mqttPublishFullState();
  }
  String resp = String("{\"ok\":true,\"name\":\"") + deviceName + "\",\"type\":\"" + deviceType + "\",\"root\":\"" + mqttRootTopic + "\"}";
  server.send(200, "application/json", resp);
}

// ======================= Arduino 生命周期 =======================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.printf("EEPROM_SIZE=%u, LedConfig=%u bytes\n", EEPROM_SIZE, (unsigned)sizeof(LedConfig));

  // 看门狗：ESP8266 硬件看门狗默认开启，确保在阻塞等待时喂狗

  // 初始化继电器输出（全部关闭）
  for (int i = 0; i < (int)(sizeof(RELAY_PINS)/sizeof(RELAY_PINS[0])); i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
  }

  // 初始化按键为上拉输入
  for (int i = 0; i < (int)(sizeof(BUTTON_PINS)/sizeof(BUTTON_PINS[0])); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  // 仅为参与通道的按钮绑定中断（避免不必要的引脚干扰）
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int idx = CHANNEL_PIN_MAP[i];
    attachButtonInterruptForIndex(idx);
  }

  // 初始化 NeoPixel
  pixels.begin();
  pixels.clear();
  pixels.show();

  // 初始化 EEPROM 并加载 LED 配置
  EEPROM.begin(EEPROM_SIZE);
  if (loadLedConfig()) {
    Serial.println("EEPROM: LED config loaded");
  } else {
    saveLedConfig();
    Serial.println("EEPROM: LED config initialized");
  }

  // 连接 WiFi 与启动 WebServer
  connectWiFi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleGetState);
  server.on("/api/toggle", HTTP_GET, handleToggle);
  server.on("/api/on", HTTP_GET, handleOn);
  server.on("/api/off", HTTP_GET, handleOff);
  server.on("/api/mode", HTTP_GET, handleSetMode);
  server.on("/api/led", HTTP_GET, handleSetLed);
  server.on("/api/led_off", HTTP_GET, handleSetLedOff);
  server.on("/api/ha_discovery", HTTP_GET, handlePublishHa);
  server.on("/api/device", HTTP_GET, handleSetDevice);
  server.on("/api/wifi_config", HTTP_GET, handleSetWifi);
  server.on("/api/mqtt_config", HTTP_GET, handleSetMqtt);
  server.on("/api/daytime", HTTP_GET, handleSetDaytime);
  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });
  server.begin();

  // 初始化 MQTT
  computeMqttRootTopic();
  mqttSetup();

  // 启用 mDNS，使用设备名作为主机名（小写，安全化）
  String mdnsHost = sanitizeId(deviceName, 24);
  mdnsHost.toLowerCase();
  if (mdnsHost.length() == 0) mdnsHost = "esp-switch";
  if (MDNS.begin(mdnsHost.c_str())) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mDNS: http://"); Serial.print(mdnsHost); Serial.println(".local");
  } else {
    Serial.println("mDNS start failed");
  }

  updateLedStates();

  // 尝试连接 MQTT 并发布一次初始状态
  mqttEnsureConnected();
  mqttPublishFullState();
}

void loop() {
  // 喂狗：主循环心跳（防止意外长任务触发复位）
  ESP.wdtFeed();

  // 处理 Web 请求
  server.handleClient();

  // 维护 WiFi 连接与 AP 回退
  maintainWiFiConnectivity();

  // 维护 MQTT 连接与循环
  mqttClient.loop();
  mqttEnsureConnected();

  // 在循环中周期性渲染连接状态指示灯（MQTT 未连时）
  renderConnectivityLed();

  // 若当前未在采样，且有按键中断触发，开始一次采样序列
  if (samplingPhysicalIndex == -1 && buttonPressedIndex != -1) {
    noInterrupts();
    samplingPhysicalIndex = buttonPressedIndex; // 冻结本次采样目标
    buttonPressedIndex = -1;                    // 清空中断标记，忽略后续抖动
    interrupts();

    buttonSampleLowCount = 0;
    buttonSampleCounter  = 0;
    lastSampleMs = millis();
  }

  // 进行非阻塞采样
  if (samplingPhysicalIndex != -1) {
    unsigned long now = millis();
    if (now - lastSampleMs >= BUTTON_SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;

      if (digitalRead(BUTTON_PINS[samplingPhysicalIndex]) == LOW) {
        buttonSampleLowCount++;
      }
      buttonSampleCounter++;

      // 达到采样次数后进行判定
      if (buttonSampleCounter >= BUTTON_SAMPLE_COUNT) {
        if (buttonSampleLowCount >= BUTTON_STABLE_THRESHOLD) {
          // 找到对应的逻辑通道，切换状态并驱动继电器
          for (int i = 0; i < NUM_CHANNELS; i++) {
            if (CHANNEL_PIN_MAP[i] == samplingPhysicalIndex) {
              setRelayState(i, !relayStates[i]);
              break;
            }
          }
        }

        // 结束本次采样
        samplingPhysicalIndex = -1;
        buttonSampleLowCount = 0;
        buttonSampleCounter  = 0;
      }
    }
  }
}

// 手动发布 Home Assistant 自动发现配置
void handlePublishHa() {
  publishHaDiscovery();
  mqttPublishFullState();
  server.send(200, "application/json", "{\"ok\":true}");
}

// 运行时维护 WiFi：断线后先重连，失败则开启 AP 供配网
void maintainWiFiConnectivity() {
  unsigned long now = millis();
  if (now - lastWiFiHealthCheckMs < 500) return; // 500ms 周期
  lastWiFiHealthCheckMs = now;

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected) {
    // 若已连接且之前处于 AP 模式，关闭 AP
    if (apMode) {
      WiFi.softAPdisconnect(true);
      apMode = false;
      Serial.println("WiFi reconnected, AP closed");
    }
    wifiLostSinceMs = 0;
    return;
  }

  // 断线：记录首次断线时间并尝试重连
  if (wifiLostSinceMs == 0) {
    wifiLostSinceMs = now;
    Serial.println("WiFi lost, attempting reconnect...");
    WiFi.reconnect();
    return;
  }

  // 超过重连窗口仍未连接，开启 AP 配网
  if (!apMode && (now - wifiLostSinceMs > WIFI_RETRY_WINDOW_MS)) {
    String apSsid = sanitizeId(deviceName, 24);
    if (apSsid.length() == 0) apSsid = "esp-switch";
    apSsid += "-setup";
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsid.c_str());
    apMode = true;
    Serial.print("AP started for setup, SSID: "); Serial.println(apSsid);
  }
}