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

ESP8266WebServer server(80);

// ======================= MQTT 配置 =======================
const char* MQTT_HOST = "192.168.1.250";
const uint16_t MQTT_PORT = 1883;
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
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
uint8_t ledOnR = 0;    // 默认绿色低亮度
uint8_t ledOnG = 16;
uint8_t ledOnB = 0;
uint8_t ledOnBrightness = 255; // 0-255，应用于打开像素的缩放

// ======================= LED 关闭颜色 =======================
uint8_t ledOffR = 0;
uint8_t ledOffG = 0;
uint8_t ledOffB = 0; // 默认黑色（熄灭）
uint8_t ledOffBrightness = 255; // 0-255，应用于关闭像素的缩放

// ======================= EEPROM 持久化 =======================
#define EEPROM_SIZE     128
#define EEPROM_MAGIC    0xA7
#define EEPROM_VERSION  2

struct LedConfig {
  uint8_t magic;
  uint8_t version;
  uint8_t onR, onG, onB, onBrightness;
  uint8_t offR, offG, offB, offBrightness;
  char name[24];
  char type[16];
  uint16_t checksum;
};

LedConfig gLedCfg;

uint16_t calcChecksum(const LedConfig &c) {
  uint32_t sum = 0;
  sum += c.magic; sum += c.version;
  sum += c.onR; sum += c.onG; sum += c.onB; sum += c.onBrightness;
  sum += c.offR; sum += c.offG; sum += c.offB; sum += c.offBrightness;
  for (size_t i = 0; i < sizeof(c.name); i++) sum += (uint8_t)c.name[i];
  for (size_t i = 0; i < sizeof(c.type); i++) sum += (uint8_t)c.type[i];
  return (uint16_t)(sum & 0xFFFF);
}

void applyLedConfigToRuntime(const LedConfig &c) {
  ledOnR = c.onR; ledOnG = c.onG; ledOnB = c.onB; ledOnBrightness = c.onBrightness;
  ledOffR = c.offR; ledOffG = c.offG; ledOffB = c.offB; ledOffBrightness = c.offBrightness;
  deviceName = String(c.name);
  deviceType = String(c.type);
  updateLedStates();
}

bool loadLedConfig() {
  EEPROM.get(0, gLedCfg);
  if (gLedCfg.magic != EEPROM_MAGIC || gLedCfg.version != EEPROM_VERSION) return false;
  if (calcChecksum(gLedCfg) != gLedCfg.checksum) return false;
  applyLedConfigToRuntime(gLedCfg);
  return true;
}

void saveLedConfig() {
  gLedCfg.magic = EEPROM_MAGIC;
  gLedCfg.version = EEPROM_VERSION;
  gLedCfg.onR = ledOnR; gLedCfg.onG = ledOnG; gLedCfg.onB = ledOnB; gLedCfg.onBrightness = ledOnBrightness;
  gLedCfg.offR = ledOffR; gLedCfg.offG = ledOffG; gLedCfg.offB = ledOffB; gLedCfg.offBrightness = ledOffBrightness;
  memset(gLedCfg.name, 0, sizeof(gLedCfg.name));
  memset(gLedCfg.type, 0, sizeof(gLedCfg.type));
  deviceName.toCharArray(gLedCfg.name, sizeof(gLedCfg.name));
  deviceType.toCharArray(gLedCfg.type, sizeof(gLedCfg.type));
  gLedCfg.checksum = calcChecksum(gLedCfg);
  EEPROM.put(0, gLedCfg);
  EEPROM.commit();
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
      // 继电器 OFF -> 使用可配置关闭颜色与亮度
      uint8_t r = (uint16_t)ledOffR * ledOffBrightness / 255;
      uint8_t g = (uint16_t)ledOffG * ledOffBrightness / 255;
      uint8_t b = (uint16_t)ledOffB * ledOffBrightness / 255;
      pixels.setPixelColor(physical_index, pixels.Color(r, g, b));
    }
  }
  pixels.show();
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
  // 打印其他消息以便调试
  Serial.print("MQTT msg "); Serial.print(topic); Serial.print(" len="); Serial.println(length);
}

void mqttSetup() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttOnMessage);
  mqttClient.setBufferSize(512); // 适当增大以容纳状态 JSON
}

void mqttEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > 3000) {
      lastMqttReconnectAttempt = now;
      String clientId = deviceType + "-" + deviceName + "-" + deviceMacNoColon;
      String statusTopic = mqttRootTopic + "/status";
      // 使用 LWT 在断线时自动发布 offline
      if (mqttClient.connect(clientId.c_str(), statusTopic.c_str(), 0, true, "offline")) {
        mqttClient.publish(statusTopic.c_str(), "online", true);
        // 订阅命令主题
        String cmdTopic = mqttRootTopic + "/cmd/#";
        mqttClient.subscribe(cmdTopic.c_str());
        Serial.print("MQTT subscribed: "); Serial.println(cmdTopic);
        Serial.println("MQTT connected");
        publishHaDiscovery();
        mqttPublishFullState();
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
}

// ======================= WiFi 与 WebServer =======================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed");
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
  // 手动重发 HA 自动发现
  html += "<div class='row'><button onclick=\"sendHa()\">重发 HA 发现</button><span class='ip'>用于 Home Assistant 自动发现</span></div>";
  // LED 打开颜色与亮度设置
  html += "<div class='row'><label>LED打开颜色: <input type='color' id='ledColor' onchange=\"setLed()\"></label>";
  html += "<label style='margin-left:12px'>亮度: <input type='range' id='ledBright' min='0' max='255' step='1' oninput=\"setLed()\"></label></div>";
  // LED 关闭颜色设置
  html += "<div class='row'><label>LED关闭颜色: <input type='color' id='ledOffColor' onchange=\"setLedOff()\"></label>";
  html += "<label style='margin-left:12px'>亮度: <input type='range' id='ledOffBright' min='0' max='255' step='1' oninput=\"setLedOff()\"></label></div>";
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
  html += "function refresh(){fetch('/api/state').then(r=>r.json()).then(j=>{var ae=(document.activeElement&&document.activeElement.id)||'';document.getElementById('status').innerText='状态: '+j.states.join(', ')+' | 互斥: '+(j.curtain?'开':'关');var el=document.getElementById('curtain');if(el){el.checked=j.curtain;}var dn=document.getElementById('devName');var dt=document.getElementById('devType');if(dn&&j.name&&ae!=='devName'){dn.value=j.name;}if(dt&&j.type&&ae!=='devType'){dt.value=j.type;}var mr=document.getElementById('mqttRoot');if(mr&&j.mqttRoot){mr.innerText=j.mqttRoot;}var ts=document.getElementById('mqttStatus');var st=document.getElementById('mqttState');var cm=document.getElementById('mqttCmd');if(j.topics){if(ts&&j.topics.status){ts.innerText=j.topics.status;}if(st&&j.topics.state){st.innerText=j.topics.state;}if(cm&&j.topics.cmd){cm.innerText=j.topics.cmd;}}else{if(ts&&j.mqttRoot){ts.innerText=j.mqttRoot+'/status';}if(st&&j.mqttRoot){st.innerText=j.mqttRoot+'/state';}if(cm&&j.mqttRoot){cm.innerText=j.mqttRoot+'/cmd';}}var lc=document.getElementById('ledColor');var lb=document.getElementById('ledBright');var lo=document.getElementById('ledOffColor');var lob=document.getElementById('ledOffBright');if(lc&&j.led&&j.led.color&&ae!=='ledColor'){lc.value=j.led.color;}if(lb&&j.led&&typeof j.led.brightness!=='undefined'&&ae!=='ledBright'){lb.value=j.led.brightness;}if(lo&&j.led&&j.led.offColor&&ae!=='ledOffColor'){lo.value=j.led.offColor;}if(lob&&j.led&&typeof j.led.offBrightness!=='undefined'&&ae!=='ledOffBright'){lob.value=j.led.offBrightness;}});}";
  html += "function toggle(ch){fetch('/api/toggle?ch='+ch).then(()=>refresh());}";
  html += "function setOn(ch){fetch('/api/on?ch='+ch).then(()=>refresh());}";
  html += "function setOff(ch){fetch('/api/off?ch='+ch).then(()=>refresh());}";
  html += "function setLed(){var c=document.getElementById('ledColor').value;var b=document.getElementById('ledBright').value;fetch('/api/led?color='+encodeURIComponent(c)+'&bright='+b).then(()=>refresh());}";
  html += "function setLedOff(){var c=document.getElementById('ledOffColor').value;var b=document.getElementById('ledOffBright').value;fetch('/api/led_off?color='+encodeURIComponent(c)+'&bright='+b).then(()=>refresh());}";
  html += "function setDevice(){var n=document.getElementById('devName').value;var t=document.getElementById('devType').value;fetch('/api/device?name='+encodeURIComponent(n)+'&type='+encodeURIComponent(t)).then(()=>refresh());}";
  html += "function setCurtain(v){fetch('/api/mode?curtain='+(v?1:0)).then(()=>refresh());}";
  html += "function sendHa(){fetch('/api/ha_discovery').then(()=>refresh());}";
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
  json += "\",\"offBrightness\":" + String(ledOffBrightness) + "}}";
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
    if (mqttClient.connected()) mqttClient.disconnect();
    mqttEnsureConnected();
    publishHaDiscovery();
    mqttPublishFullState();
  }
  String resp = String("{\"ok\":true,\"name\":\"") + deviceName + "\",\"type\":\"" + deviceType + "\",\"root\":\"" + mqttRootTopic + "\"}";
  server.send(200, "application/json", resp);
}

// ======================= Arduino 生命周期 =======================
void setup() {
  Serial.begin(115200);
  delay(50);

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
  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });
  server.begin();

  // 初始化 MQTT
  computeMqttRootTopic();
  mqttSetup();

  // 启用 mDNS，便于通过域名访问
  if (MDNS.begin("esp-switch")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://esp-switch.local");
  } else {
    Serial.println("mDNS start failed");
  }

  updateLedStates();

  // 尝试连接 MQTT 并发布一次初始状态
  mqttEnsureConnected();
  mqttPublishFullState();
}

void loop() {
  // 处理 Web 请求
  server.handleClient();

  // 维护 MQTT 连接与循环
  mqttClient.loop();
  mqttEnsureConnected();

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