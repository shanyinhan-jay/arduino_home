// ESP8266 舵机测试 + 简易网页UI控制（GPIO5 作为 PWM 输出）
// 板型：NodeMCU v2（esp8266:esp8266:nodemcuv2）
// 引脚映射：GPIO5 = D1

#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <PubSubClient.h>

static const int SERVO_PIN = 5; // GPIO5（D1）

Servo servo;
int sweepDir = 1;
int angle = 90;
unsigned long lastStepMs = 0;
bool sweepEnabled = false; // UI可开关；false=停在设定角度
int minPulseUs = 500; // 默认脉宽下限（可在UI调整）
int maxPulseUs = 2500; // 默认脉宽上限（可在UI调整）

ESP8266WebServer server(80);

// WiFi/MQTT 运行时参数（上移到顶端，便于后续函数使用）
String wifiSsid = "";
String wifiPass = "";
bool apMode = false;
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
String mqttHostStr = "";
uint16_t mqttPortVal = 1883;
String mqttUser = "";
String mqttPass = "";
String deviceId;
String mqttRootTopic; // 例如 esp-servo/<MAC>
unsigned long lastMqttReconnectAttempt = 0;
String deviceName = ""; // 设备显示名称（路由器与mDNS）
String deviceType = "servo"; // 设备类型

// 前置声明：避免在 loop/computeId 中调用未声明函数导致编译错误
void mqttPublishState();
void mqttEnsureConnected();
void computeMqttRootTopic();
void handleGetCombos();
void handleSaveCombo();
void handleDeleteCombo();
void publishHaComboDiscovery(const String &name);
void publishHaDiscovery();
void handlePublishHa();
void clearHaDiscovery();
void clearHaDiscoveryCombo(const String &name);
void handleHaClearCombo();
void handleOtaUpload();
void handleOtaDone();
void renderConnectivityLed();
void mqttSetup();
void handleSetMqtt();
void handleSetDevice();
void handleExecCombo();
void handleConfig();
void handleGetState();
void handleSetWifi();
void computeId();
void connectWiFi();
String sanitizeId(String s, size_t maxLen);
uint8_t findComboIndexByName(const String &name);
bool startSequenceFromScript(String script);

// 组合动作（序列执行）
enum StepType : uint8_t { STEP_ANGLE = 0, STEP_WAIT = 1 };
struct SequenceStep { uint8_t type; int value; };
SequenceStep seqSteps[16];
int seqCount = 0;
int seqIndex = 0;
bool seqRunning = false;
unsigned long seqWaitUntil = 0;

// 已保存的组合动作（运行时）
uint8_t comboCount = 0;

// EEPROM 配置结构与函数（上移至顶部，保证宏与类型在使用前定义）
#define SERVO_EEPROM_SIZE 512
#define SERVO_EEPROM_MAGIC 0xB5
#define SERVO_EEPROM_VERSION 2
// 组合动作存储容量
#define MAX_COMBOS 2
#define COMBO_NAME_LEN 24
#define COMBO_SCRIPT_LEN 64
String comboNames[MAX_COMBOS];
String comboScripts[MAX_COMBOS];
struct ServoConfig {
  uint8_t magic;
  uint8_t version;
  char wifiSsidArr[32];
  char wifiPassArr[64];
  char mqttHostArr[64];
  uint16_t mqttPort;
  char mqttUserArr[32];
  char mqttPassArr[64];
  uint16_t minPulse;
  uint16_t maxPulse;
  char devNameArr[32];
  char devTypeArr[16];
  uint8_t combosCount;
  char comboNames[MAX_COMBOS][COMBO_NAME_LEN];
  char comboScripts[MAX_COMBOS][COMBO_SCRIPT_LEN];
  uint16_t checksum;
};
ServoConfig gCfg;

// 连接指示灯（GPIO2 / D4），NodeMCU 内置 LED 为低电平点亮
static const int LED_PIN = 2;
static const bool LED_ACTIVE_LOW = true;
bool otaInProgress = false;
unsigned long ledLastToggleMs = 0;
bool ledState = false;

uint16_t cfgChecksum(const ServoConfig &c){
uint32_t s=0; s+=c.magic; s+=c.version; s+=c.mqttPort; s+=c.minPulse; s+=c.maxPulse;
for(size_t i=0;i<sizeof(c.wifiSsidArr);i++) s+=(uint8_t)c.wifiSsidArr[i];
for(size_t i=0;i<sizeof(c.wifiPassArr);i++) s+=(uint8_t)c.wifiPassArr[i];
for(size_t i=0;i<sizeof(c.mqttHostArr);i++) s+=(uint8_t)c.mqttHostArr[i];
for(size_t i=0;i<sizeof(c.mqttUserArr);i++) s+=(uint8_t)c.mqttUserArr[i];
for(size_t i=0;i<sizeof(c.mqttPassArr);i++) s+=(uint8_t)c.mqttPassArr[i];
return (uint16_t)(s & 0xFFFF);
}
void applyCfg(const ServoConfig &c){
  wifiSsid = String(c.wifiSsidArr);
  wifiPass = String(c.wifiPassArr);
  mqttHostStr = String(c.mqttHostArr);
  mqttPortVal = c.mqttPort ? c.mqttPort : 1883;
  mqttUser = String(c.mqttUserArr);
  mqttPass = String(c.mqttPassArr);
  deviceName = String(c.devNameArr);
  deviceType = String(c.devTypeArr);
  // 组合动作（版本>=2）
  comboCount = 0;
  if(c.version >= 2){
    comboCount = c.combosCount;
    if(comboCount > MAX_COMBOS) comboCount = MAX_COMBOS;
    for(uint8_t i=0;i<comboCount;i++){
      comboNames[i] = String(c.comboNames[i]);
      comboScripts[i] = String(c.comboScripts[i]);
    }
    for(uint8_t i=comboCount;i<MAX_COMBOS;i++){ comboNames[i] = String(""); comboScripts[i] = String(""); }
  } else {
    for(uint8_t i=0;i<MAX_COMBOS;i++){ comboNames[i] = String(""); comboScripts[i] = String(""); }
  }
}
bool loadCfg(){
  EEPROM.get(0, gCfg);
  if(gCfg.magic==SERVO_EEPROM_MAGIC && gCfg.version==SERVO_EEPROM_VERSION && cfgChecksum(gCfg)==gCfg.checksum){
    applyCfg(gCfg);
    Serial.println("EEPROM: config loaded");
    return true;
  }
  Serial.println("EEPROM: config init");
  memset(&gCfg,0,sizeof(gCfg));
  gCfg.magic = SERVO_EEPROM_MAGIC;
  gCfg.version = SERVO_EEPROM_VERSION;
  gCfg.mqttPort = 1883;
  // 默认 WiFi 凭据（首次初始化时写入 EEPROM）
  strncpy(gCfg.wifiSsidArr, "Linksys-2.4G-wifi5", sizeof(gCfg.wifiSsidArr)-1);
  strncpy(gCfg.wifiPassArr, "19811201", sizeof(gCfg.wifiPassArr)-1);
  gCfg.checksum = cfgChecksum(gCfg);
  EEPROM.put(0, gCfg); EEPROM.commit();
  // 应用到运行时，使本次启动直接使用默认 WiFi
  applyCfg(gCfg);
  return false;
}
void saveCfg(){
  memset(&gCfg,0,sizeof(gCfg));
  gCfg.magic = SERVO_EEPROM_MAGIC;
  gCfg.version = SERVO_EEPROM_VERSION;
  strncpy(gCfg.wifiSsidArr, wifiSsid.c_str(), sizeof(gCfg.wifiSsidArr)-1);
  strncpy(gCfg.wifiPassArr, wifiPass.c_str(), sizeof(gCfg.wifiPassArr)-1);
  strncpy(gCfg.mqttHostArr, mqttHostStr.c_str(), sizeof(gCfg.mqttHostArr)-1);
  gCfg.mqttPort = mqttPortVal;
  strncpy(gCfg.mqttUserArr, mqttUser.c_str(), sizeof(gCfg.mqttUserArr)-1);
  strncpy(gCfg.mqttPassArr, mqttPass.c_str(), sizeof(gCfg.mqttPassArr)-1);
  strncpy(gCfg.devNameArr, deviceName.c_str(), sizeof(gCfg.devNameArr)-1);
  strncpy(gCfg.devTypeArr, deviceType.c_str(), sizeof(gCfg.devTypeArr)-1);
  gCfg.combosCount = comboCount > MAX_COMBOS ? MAX_COMBOS : comboCount;
  for(uint8_t i=0;i<gCfg.combosCount;i++){
    strncpy(gCfg.comboNames[i], comboNames[i].c_str(), COMBO_NAME_LEN-1);
    strncpy(gCfg.comboScripts[i], comboScripts[i].c_str(), COMBO_SCRIPT_LEN-1);
  }
  gCfg.checksum = cfgChecksum(gCfg);
  EEPROM.put(0, gCfg); EEPROM.commit();
}

static const char INDEX_HTML[] PROGMEM = R"=====(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP8266 舵机控制与配置</title>
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto;max-width:520px;margin:24px auto;padding:0 12px}
    h1{font-size:20px;margin:0 0 12px}
    .card{border:1px solid #ddd;border-radius:8px;padding:12px;margin-top:12px}
    .row{display:flex;align-items:center;gap:12px;margin:8px 0}
    input[type=range]{width:100%}
    button{padding:6px 10px;border:1px solid #888;border-radius:6px;background:#fafafa;cursor:pointer}
    button:hover{background:#f0f0f0}
    .hint{color:#666;font-size:12px}
    input[type=text],input[type=number],input[type=password]{padding:6px;border:1px solid #ccc;border-radius:6px}
    label>input{margin-left:6px}
  </style>
</head>
<body>
  <h1>ESP8266 舵机控制与配置 <span class="hint" id="devLabel"></span></h1>
  <div class="card">
    <div class="row"><strong>设备信息</strong></div>
    <div class="row">
      <label>设备名<input id="devName" type="text" placeholder="例如：客厅窗帘"></label>
      <label>类型<input id="devType" type="text" placeholder="servo"></label>
      <button onclick="saveDevice()">保存设备信息</button>
    </div>
    <div class="hint">说明：设备名用于路由器和 mDNS 显示（http://设备名.local）。</div>
  </div>

  <div class="card">
    <div class="row"><strong>WiFi 配置</strong></div>
    <div class="row">
      <label>SSID<input id="wifiSsid" type="text" placeholder="WiFi 名称"></label>
      <label>密码<input id="wifiPass" type="password" placeholder="WiFi 密码"></label>
      <button onclick="saveWifi()">保存并连接</button>
    </div>
    <div class="row"><div class="hint">当前连接：<span id="wifiSsidNow">-</span>；强度：<span id="wifiStrength">-</span></div></div>
    <div class="hint">若路由器连接失败，设备会自动开启 AP “esp-servo-setup”用于配网。</div>
  </div>

  <div class="card">
    <div class="row"><strong>MQTT 配置</strong></div>
    <div class="row">
      <label>主机<input id="mqttHost" type="text" placeholder="mqtt.example.com"></label>
      <label>端口<input id="mqttPort" type="number" value="1883" style="width:100px"></label>
    </div>
    <div class="row">
      <label>用户名<input id="mqttUser" type="text"></label>
      <label>密码<input id="mqttPass" type="password"></label>
      <button onclick="saveMqtt()">保存并连接</button>
    </div>
    <div class="row"><strong>MQTT 主题</strong></div>
    <div class="row">
      <div class="hint">前缀：<code id="mqttPrefix"></code></div>
    </div>
    <div class="row">
      <div class="hint">发布角度：<code id="mqttAnglePub"></code></div>
    </div>
    <div class="row">
      <div class="hint">订阅设角：<code id="mqttSetSub"></code></div>
    </div>
    <div class="row">
      <button onclick="sendHa()">重发 HA 自动发现</button>
      <button onclick="sendHaClearAll()" style="margin-left:8px">清理自动发现（设备及全部实体）</button>
      <span class="hint">将删除该设备在 Home Assistant 的所有已发现实体。</span>
    </div>
  </div>

  <div class="card">
    <div class="row"><strong>OTA 升级</strong></div>
    <div class="row" style="flex-direction:column;align-items:flex-start">
      <input id="otaFile" type="file" accept=".bin,.bin.gz">
      <button onclick="doOta()" style="margin-top:8px">上传固件并升级</button>
      <div class="hint">选择由 Arduino 编译生成的固件 .bin 文件，上传后设备会自动重启。</div>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <label for="angle">角度: <span id="val">-</span>°</label>
    </div>
    <div class="row">
      <input id="angle" type="range" min="0" max="180" step="1">
    </div>
    <div class="row">
      <button onclick="setAngle(0)">最小(0°)</button>
      <button onclick="setAngle(90)">居中(90°)</button>
      <button onclick="setAngle(180)">最大(180°)</button>
    </div>
    <div class="row">
      <label><input id="sweep" type="checkbox"> 开启自动扫动</label>
    </div>
    <div class="hint">提示：拖动滑条或点按钮可立即设置角度；开启扫动将忽略滑条固定角度。</div>
  </div>
  
  <div class="card">
    <div class="row"><strong>脉宽范围校准 (微秒)</strong></div>
    <div class="row">
      <label>最小(us)：<input id="minUs" type="number" min="400" max="2600" step="10" style="width:100px"></label>
      <label>最大(us)：<input id="maxUs" type="number" min="400" max="2600" step="10" style="width:100px"></label>
      <button onclick="applyPulse()">应用</button>
    </div>
    <div class="hint">说明：不同舵机出厂行程不同，适当扩大范围（如 500..2500）可获得更大角度；请避免机械卡滞。</div>
  </div>

  <div class="card">
    <div class="row"><strong>组合动作</strong> <span class="hint" id="seqStatus">未运行</span></div>
    <div class="row">
      <label>名称<input id="comboName" type="text" placeholder="例如：往返动作"></label>
    </div>
    <div class="row"><strong>简单模式</strong></div>
    <div class="row">
      <label>A角度<input id="comboA" type="number" min="0" max="180" step="1" style="width:100px" placeholder="90"></label>
      <label>延时(ms)<input id="comboD" type="number" min="0" max="5000" step="50" style="width:120px" placeholder="300"></label>
      <label>B角度<input id="comboB" type="number" min="0" max="180" step="1" style="width:100px" placeholder="30"></label>
    </div>
    <div class="row"><strong>脚本模式</strong></div>
    <div class="row">
      <textarea id="comboScript" rows="3" style="width:100%" placeholder="示例：A:90;W:300;A:30"></textarea>
    </div>
    <div class="row">
      <button id="execBtn" onclick="execCombo()">执行组合动作</button>
      <button onclick="saveCombo()">保存组合动作</button>
    </div>
    <div class="row"><strong>已保存动作</strong></div>
    <div id="comboList" class="row" style="flex-direction:column;align-items:flex-start"></div>
    <div class="hint">脚本支持：A:角度 / A角度；W:毫秒 / wait=毫秒 / delay=毫秒；使用 ; 或 , 分隔步骤。</div>
  </div>

  <script>
  const angleEl = document.getElementById('angle');
  const valEl = document.getElementById('val');
  const sweepEl = document.getElementById('sweep');
  const minUsEl = document.getElementById('minUs');
  const maxUsEl = document.getElementById('maxUs');
  const devLabel = document.getElementById('devLabel');
  const mqttPrefixEl = document.getElementById('mqttPrefix');
  const mqttAnglePubEl = document.getElementById('mqttAnglePub');
  const mqttSetSubEl = document.getElementById('mqttSetSub');
  const devNameEl = document.getElementById('devName');
  const devTypeEl = document.getElementById('devType');
  const wifiSsidEl = document.getElementById('wifiSsid');
  const wifiPassEl = document.getElementById('wifiPass');
  const wifiSsidNowEl = document.getElementById('wifiSsidNow');
  const wifiStrengthEl = document.getElementById('wifiStrength');
  const mqttHostEl = document.getElementById('mqttHost');
  const mqttPortEl = document.getElementById('mqttPort');
  const mqttUserEl = document.getElementById('mqttUser');
  const mqttPassEl = document.getElementById('mqttPass');
  const comboNameEl = document.getElementById('comboName');
  const comboScriptEl = document.getElementById('comboScript');
  const comboAEl = document.getElementById('comboA');
  const comboDEl = document.getElementById('comboD');
  const comboBEl = document.getElementById('comboB');
  const seqStatusEl = document.getElementById('seqStatus');
  const execBtn = document.getElementById('execBtn');
  const comboListEl = document.getElementById('comboList');

  function clamp(v){v=+v; if(isNaN(v)) v=0; if(v<0)v=0; if(v>180)v=180; return v;}
  // 当用户正在编辑时，避免自动刷新覆盖输入内容
  window.refreshPaused = false;
  function safeSetValue(el, v){ if(document.activeElement === el) return; el.value = v; }
  // 为常见可编辑控件添加焦点/失焦事件，暂停刷新
  (function(){
    const editableEls = [angleEl, minUsEl, maxUsEl, devNameEl, devTypeEl, wifiSsidEl, wifiPassEl, mqttHostEl, mqttPortEl, mqttUserEl, mqttPassEl, comboNameEl, comboScriptEl, comboAEl, comboDEl, comboBEl].filter(Boolean);
    editableEls.forEach(el=>{
      el.addEventListener('focus', ()=>{ window.refreshPaused = true; });
      el.addEventListener('blur', ()=>{ setTimeout(()=>{ window.refreshPaused = false; }, 500); });
    });
  })();
  function refresh(){
    if(window.refreshPaused) return;
    fetch('/api/angle').then(r=>r.json()).then(j=>{
      const ang = clamp(j.angle);
      if(document.activeElement !== angleEl){ safeSetValue(angleEl, ang); valEl.textContent = ang; }
      else { valEl.textContent = angleEl.value; }
      if(document.activeElement !== sweepEl) sweepEl.checked = !!j.sweep;
    }).catch(console.error);
    fetch('/api/pulse').then(r=>r.json()).then(j=>{
      safeSetValue(minUsEl, j.min);
      safeSetValue(maxUsEl, j.max);
    }).catch(console.error);
    fetch('/api/state').then(r=>r.json()).then(st=>{
      const tp = st.topics || { prefix: st.topic, angle_pub: (st.topic? st.topic+"/angle":""), set_sub: (st.topic? st.topic+"/set":"") };
      mqttPrefixEl.textContent = tp.prefix || '';
      mqttAnglePubEl.textContent = tp.angle_pub || '';
      mqttSetSubEl.textContent = tp.set_sub || '';
      devLabel.textContent = (st.name||'') ? ('设备：'+st.name+'（类型：'+(st.type||'')+'）') : '';
      safeSetValue(devNameEl, st.name || '');
      safeSetValue(devTypeEl, st.type || 'servo');
      safeSetValue(wifiSsidEl, (st.wifi && st.wifi.ssid) ? st.wifi.ssid : '');
      const w = st.wifi || {};
      if(wifiSsidNowEl) wifiSsidNowEl.textContent = w.ssid || '-';
      if(wifiStrengthEl) wifiStrengthEl.textContent = (typeof w.rssi === 'number') ? (w.rssi + ' dBm') : '-';
      safeSetValue(mqttHostEl, (st.mqtt && st.mqtt.host) ? st.mqtt.host : '');
      safeSetValue(mqttPortEl, (st.mqtt && st.mqtt.port) ? st.mqtt.port : 1883);
      const seq = st.sequence || { running:0, index:0, count:0 };
      seqStatusEl.textContent = seq.running ? ('运行中（'+seq.index+'/'+seq.count+'）') : '未运行';
      execBtn.disabled = !!seq.running;
    }).catch(console.error);
    refreshCombos();
  }
  function setAngle(v){
    v = clamp(v);
    fetch('/api/set?angle='+v).then(()=>refresh()).catch(console.error);
  }
  angleEl.addEventListener('input', e=>{
    const v = clamp(e.target.value);
    valEl.textContent = v;
    fetch('/api/set?angle='+v).catch(console.error);
  });
  sweepEl.addEventListener('change', e=>{
    const en = e.target.checked?1:0;
    fetch('/api/sweep?enable='+en).then(()=>refresh()).catch(console.error);
  });
  function applyPulse(){
    const min = +minUsEl.value || 500;
    const max = +maxUsEl.value || 2500;
    fetch('/api/pulse?min='+min+'&max='+max).then(()=>refresh()).catch(console.error);
  }
  function saveDevice(){
    const n = encodeURIComponent(devNameEl.value||'');
    const t = encodeURIComponent(devTypeEl.value||'');
    fetch('/api/device?name='+n+'&type='+t).then(()=>refresh()).catch(console.error);
  }
  function saveWifi(){
    const s = encodeURIComponent(wifiSsidEl.value||'');
    const p = encodeURIComponent(wifiPassEl.value||'');
    fetch('/api/wifi_config?ssid='+s+'&pass='+p).then(()=>refresh()).catch(console.error);
  }
  function saveMqtt(){
    const h = encodeURIComponent(mqttHostEl.value||'');
    const po = encodeURIComponent(mqttPortEl.value||'');
    const u = encodeURIComponent(mqttUserEl.value||'');
    const pw = encodeURIComponent(mqttPassEl.value||'');
    fetch('/api/mqtt_config?host='+h+'&port='+po+'&user='+u+'&pass='+pw).then(()=>refresh()).catch(console.error);
  }
  function execCombo(){
    execBtn.disabled = true;
    const name = encodeURIComponent(comboNameEl.value||'');
    const scriptRaw = (comboScriptEl.value||'').trim();
    let url;
    if(scriptRaw){
      url = '/api/exec_combo?name='+name+'&script='+encodeURIComponent(scriptRaw);
    } else {
      const a = encodeURIComponent(comboAEl.value||'');
      const d = encodeURIComponent(comboDEl.value||'');
      const b = encodeURIComponent(comboBEl.value||'');
      url = '/api/exec_combo?name='+name+'&a='+a+'&d='+d+'&b='+b;
    }
    fetch(url).then(()=>{ refresh(); execBtn.disabled=false; }).catch((e)=>{ console.error(e); execBtn.disabled=false; });
  }
  function saveCombo(){
    const name = encodeURIComponent((comboNameEl.value||'').trim());
    const scriptRaw = (comboScriptEl.value||'').trim();
    let url;
    if(scriptRaw){
      url = '/api/combo_save?name='+name+'&script='+encodeURIComponent(scriptRaw);
    } else {
      const a = encodeURIComponent(comboAEl.value||'');
      const d = encodeURIComponent(comboDEl.value||'');
      const b = encodeURIComponent(comboBEl.value||'');
      url = '/api/combo_save?name='+name+'&a='+a+'&d='+d+'&b='+b;
    }
    fetch(url).then(()=>{ refreshCombos(); }).catch(console.error);
  }
  function sendHa(){
    fetch('/api/ha_discovery')
      .then(()=>{ alert('已发送 HA 自动发现'); refresh(); })
      .catch((e)=>{ console.error(e); alert('发送 HA 自动发现失败'); });
  }
  function sendHaReset(){
    fetch('/api/ha_discovery?reset=1').then(()=>refresh()).catch(console.error);
  }
  function sendHaClearAll(){
    fetch('/api/ha_discovery?reset=1')
      .then(()=>{ alert('已清理设备及全部实体的自动发现'); refresh(); })
      .catch((e)=>{ console.error(e); alert('清理自动发现失败'); });
  }
  function sendHaClearCombo(){
    const el = document.getElementById('haClearName');
    const name = (el && el.value) ? el.value.trim() : '';
    if(!name){ alert('请输入要清理的动作名称'); return; }
    fetch('/api/ha_clear_combo?name='+encodeURIComponent(name))
      .then(()=>{ alert('已清理动作发现：'+name); refresh(); })
      .catch((e)=>{ console.error(e); alert('清理动作发现失败：'+name); });
  }
  function doOta(){
    const fEl = document.getElementById('otaFile');
    const f = fEl && fEl.files && fEl.files[0];
    if(!f){ alert('请选择固件文件(.bin)'); return; }
    const fd = new FormData();
    fd.append('firmware', f, f.name);
    fetch('/api/ota', { method:'POST', body:fd })
      .then(r=>r.text())
      .then(t=>{
        try{
          const j = JSON.parse(t);
          alert(j.ok ? '升级完成，设备将重启' : ('升级失败: '+(j.error||t)));
        }catch(e){ alert(t); }
        setTimeout(()=>refresh(), 3000);
      })
      .catch(e=>{ alert('上传失败: '+e); });
  }
  function refreshCombos(){
    fetch('/api/combo_list').then(r=>r.json()).then(j=>{
      const list = j.combos || [];
      let html = '';
      if(!list.length) html = '<div class="hint">暂无保存的动作</div>';
      else {
        for(const it of list){
          const escName = it.name.replace(/[&<>]/g, s=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[s]));
          const escScript = it.script.replace(/[&<>]/g, s=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[s]));
          html += '<div style="display:flex;gap:8px;align-items:center;">'
            + '<strong>'+escName+'</strong>'
            + '<code style="background:#f6f6f6;padding:2px 4px;border-radius:4px">'+escScript+'</code>'
            + '<button onclick="execSaved(\''+encodeURIComponent(it.name)+'\')">执行</button>'
            + '<button onclick="delSaved(\''+encodeURIComponent(it.name)+'\')">删除</button>'
            + '</div>';
        }
      }
      comboListEl.innerHTML = html;
    }).catch(console.error);
  }
  function execSaved(nameEnc){
    const name = decodeURIComponent(nameEnc);
    // 获取脚本后执行
    fetch('/api/combo_list').then(r=>r.json()).then(j=>{
      const it = (j.combos||[]).find(x=>x.name===name);
      if(!it) return;
      fetch('/api/exec_combo?name='+encodeURIComponent(name)+'&script='+encodeURIComponent(it.script))
        .then(()=>refresh()).catch(console.error);
    }).catch(console.error);
  }
  function delSaved(nameEnc){
    const name = decodeURIComponent(nameEnc);
    fetch('/api/combo_delete?name='+encodeURIComponent(name))
      .then(()=>{
        return fetch('/api/ha_clear_combo?name='+encodeURIComponent(name));
      })
      .then(()=>{ alert('已删除并清理该动作的自动发现：'+name); refreshCombos(); })
      .catch((e)=>{ console.error(e); alert('删除或清理失败：'+name); });
  }
  refresh();
  setInterval(refresh, 1000);
  </script>
</body>
</html>
)=====";

static inline int clampAngle(int v){ if(v<0) return 0; if(v>180) return 180; return v; }
static inline int clampPulse(int us){ if(us<400) return 400; if(us>2600) return 2600; return us; }

// 组合动作解析与启动
void clearSequence(){ seqCount=0; seqIndex=0; seqRunning=false; seqWaitUntil=0; }
bool appendStep(uint8_t type, int value){ if(seqCount>=16) return false; seqSteps[seqCount].type=type; seqSteps[seqCount].value=value; seqCount++; return true; }
bool parseScript(String script){
  clearSequence();
  script.trim(); if(!script.length()) return false;
  // 支持分隔符 ; 或 ,
  int start=0;
  while(start < (int)script.length()){
    int sep = script.indexOf(';', start);
    int sep2 = script.indexOf(',', start);
    int end;
    if(sep==-1 && sep2==-1) end = script.length();
    else if(sep==-1) end = sep2; else if(sep2==-1) end = sep; else end = min(sep, sep2);
    String tok = script.substring(start, end); tok.trim();
    if(tok.length()){
      String t = tok; t.toLowerCase();
      // 允许格式：a:90 / angle=90 / angle:90 / a90
      if(t.startsWith("a:")) { int v = t.substring(2).toInt(); if(!appendStep(STEP_ANGLE, clampAngle(v))) return false; }
      else if(t.startsWith("angle:")) { int v = t.substring(6).toInt(); if(!appendStep(STEP_ANGLE, clampAngle(v))) return false; }
      else if(t.startsWith("angle=")) { int v = t.substring(6).toInt(); if(!appendStep(STEP_ANGLE, clampAngle(v))) return false; }
      else if(t.startsWith("a")) { int v = t.substring(1).toInt(); if(!appendStep(STEP_ANGLE, clampAngle(v))) return false; }
      // 允许格式：w:300 / wait=300 / delay:300 / d300
      else if(t.startsWith("w:")) { int v = t.substring(2).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else if(t.startsWith("wait:")) { int v = t.substring(5).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else if(t.startsWith("delay:")) { int v = t.substring(6).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else if(t.startsWith("wait=")) { int v = t.substring(5).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else if(t.startsWith("delay=")) { int v = t.substring(6).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else if(t.startsWith("d")) { int v = t.substring(1).toInt(); if(!appendStep(STEP_WAIT, max(0, v))) return false; }
      else { return false; }
    }
    start = end + 1;
  }
  return seqCount>0;
}
bool startSequenceFromScript(String script){ if(!parseScript(script)) return false; seqRunning=true; seqIndex=0; seqWaitUntil=0; return true; }

void handleRoot(){
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleGetAngle(){
  String j = String("{")+
    "\"angle\":" + String(angle) + "," +
    "\"sweep\":" + String(sweepEnabled ? 1 : 0) +
  "}";
  server.send(200, "application/json", j);
}

void handleSetAngle(){
  if (server.hasArg("angle")) {
    String s = server.arg("angle");
    Serial.print("[HTTP] set angle: "); Serial.println(s);
    int v = clampAngle(s.toInt());
    angle = v;
    sweepEnabled = false; // 一旦手动设定角度，关闭扫动
    servo.write(angle);
    mqttPublishState();
  } else {
    Serial.println("[HTTP] set angle: missing 'angle'");
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSweep(){
  if (server.hasArg("enable")) {
    int en = server.arg("enable").toInt();
    Serial.print("[HTTP] sweep: "); Serial.println(en);
    sweepEnabled = (en != 0);
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGetPulse(){
  String j = String("{")+
    "\"min\":" + String(minPulseUs) + "," +
    "\"max\":" + String(maxPulseUs) +
  "}";
  server.send(200, "application/json", j);
}

void handleSetPulse(){
  int minUs = minPulseUs;
  int maxUs = maxPulseUs;
  if (server.hasArg("min")) minUs = clampPulse(server.arg("min").toInt());
  if (server.hasArg("max")) maxUs = clampPulse(server.arg("max").toInt());
  if (minUs >= maxUs) {
    int mid = (minUs + maxUs) / 2;
    minUs = mid - 500; maxUs = mid + 500;
  }
  minUs = clampPulse(minUs);
  maxUs = clampPulse(maxUs);
  minPulseUs = minUs;
  maxPulseUs = maxUs;
  Serial.print("[HTTP] pulse range: "); Serial.print(minPulseUs); Serial.print(".."); Serial.println(maxPulseUs);
  servo.detach();
  delay(10);
  servo.attach(SERVO_PIN, minPulseUs, maxPulseUs);
  servo.write(angle);
  server.send(200, "application/json", "{\"ok\":true}");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("ESP8266 Servo + Web UI on GPIO5 (D1)");
  Serial.println("- 舵机信号线接 D1(GPIO5)");
  Serial.println("- 若未连接路由器，将开启 AP 'esp-servo-setup' 配网，访问 http://192.168.4.1/");
  // 初始化连接指示灯：默认熄灭
  pinMode(LED_PIN, OUTPUT);
  if(LED_ACTIVE_LOW) digitalWrite(LED_PIN, HIGH); else digitalWrite(LED_PIN, LOW);

  // 舵机初始化
  servo.attach(SERVO_PIN, minPulseUs, maxPulseUs);
  servo.write(angle); // 先居中

  // EEPROM 与设备ID
  EEPROM.begin(SERVO_EEPROM_SIZE);
  loadCfg();
  computeId();

  // 优先连接 WiFi；失败则保留 AP 配网
  connectWiFi();
  if(WiFi.getMode()==WIFI_AP || apMode){ IPAddress ip = WiFi.softAPIP(); Serial.print("AP IP: "); Serial.println(ip); }

  // mDNS（若已连路由器）
  if(WiFi.status()==WL_CONNECTED){
    String defaultName = String("esp-servo-") + (deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId);
    String mdnsHost = sanitizeId(deviceName.length()?deviceName:defaultName, 24);
    mdnsHost.toLowerCase();
    if(MDNS.begin(mdnsHost.c_str())){
      MDNS.addService("http","tcp",80);
      Serial.printf("mDNS: http://%s.local\n", mdnsHost.c_str());
    }
  }

  // WebServer 路由
  server.on("/", handleRoot);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/api/angle", HTTP_GET, handleGetAngle);
  server.on("/api/state", HTTP_GET, handleGetState);
  server.on("/api/set", HTTP_GET, handleSetAngle);
  server.on("/api/set", HTTP_POST, handleSetAngle);
  server.on("/api/sweep", HTTP_GET, handleSweep);
  server.on("/api/sweep", HTTP_POST, handleSweep);
  server.on("/api/pulse", HTTP_GET, handleGetPulse);
  server.on("/api/pulse", HTTP_POST, handleSetPulse);
  server.on("/api/wifi_config", HTTP_GET, handleSetWifi);
  server.on("/api/mqtt_config", HTTP_GET, handleSetMqtt);
  server.on("/api/device", HTTP_GET, handleSetDevice);
  server.on("/api/exec_combo", HTTP_GET, handleExecCombo);
  server.on("/api/combo_list", HTTP_GET, handleGetCombos);
  server.on("/api/combo_save", HTTP_GET, handleSaveCombo);
  server.on("/api/combo_delete", HTTP_GET, handleDeleteCombo);
  server.on("/api/ota", HTTP_POST, handleOtaDone, handleOtaUpload);
  server.onNotFound([](){
    Serial.print("[HTTP] 404: "); Serial.println(server.uri());
    server.send(404, "text/plain", "Not Found");
  });
  server.on("/api/ha_discovery", HTTP_GET, handlePublishHa);
  server.on("/api/ha_clear_combo", HTTP_GET, handleHaClearCombo);
  server.begin();
  Serial.println("WebServer started.");

  // 舵机与 MQTT 初始化
  mqttSetup();
}

void loop() {
  // 处理HTTP请求
  server.handleClient();
  mqttClient.loop();
  mqttEnsureConnected();
  renderConnectivityLed();

  // 读取串口输入（整数角度 0..180）
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        int v = buf.toInt();
        v = clampAngle(v);
        angle = v;
        sweepEnabled = false;
        servo.write(angle);
        Serial.print("Set angle: "); Serial.println(v);
        buf = "";
      }
    } else if (c >= 32 && c <= 126) {
      buf += c;
      if (buf.length() > 6) buf = buf.substring(0, 6);
    }
  }

  // 扫动（0..180..0），步进周期约 20ms；当 sweepEnabled=true 时生效
  if (sweepEnabled) {
    unsigned long now = millis();
    if (now - lastStepMs >= 20) {
      lastStepMs = now;
      angle += sweepDir;
      if (angle >= 180) { angle = 180; sweepDir = -1; }
      if (angle <= 0)   { angle = 0;   sweepDir = 1; }
      servo.write(angle);
      mqttPublishState();
    }
  }

  // 组合动作运行（非阻塞状态机）
  if(seqRunning){
    if(seqIndex >= seqCount){ seqRunning=false; seqWaitUntil=0; }
    else {
      SequenceStep &st = seqSteps[seqIndex];
      if(st.type == STEP_ANGLE){
        int v = clampAngle(st.value);
        angle = v; sweepEnabled = false; servo.write(angle); mqttPublishState();
        seqIndex++;
      } else if(st.type == STEP_WAIT){
        unsigned long now = millis();
        if(seqWaitUntil==0) seqWaitUntil = now + (unsigned long)max(0, st.value);
        if(now >= seqWaitUntil){ seqWaitUntil=0; seqIndex++; }
      }
    }
  }
  else {
    // 保底：未扫动时在主循环中维持当前角度
    static int lastAngle = -1;
    if (lastAngle != angle) {
      servo.write(angle);
      lastAngle = angle;
      mqttPublishState();
    }
  }
}

// 连接指示灯渲染逻辑（GPIO2）：
// - WiFi 断开：快闪
// - WiFi 连接但 MQTT 未连接：慢闪
// - MQTT 连接：熄灭
static inline void ledSet(bool on){ if(LED_ACTIVE_LOW) digitalWrite(LED_PIN, on?LOW:HIGH); else digitalWrite(LED_PIN, on?HIGH:LOW); }
void renderConnectivityLed(){
  // OTA 进行中：优先展示较快闪烁，以便用户感知升级状态
  unsigned long now = millis();
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool mqttOk = mqttClient.connected();
  if(mqttOk){
    ledSet(false);
    return;
  }
  unsigned long interval = 0;
  if(otaInProgress){
    interval = 150; // OTA 加载时更快闪烁
  } else if(wifiOk){
    interval = 700; // 慢闪
  } else {
    interval = 200; // 快闪
  }
  if(now - ledLastToggleMs >= interval){
    ledLastToggleMs = now;
    ledState = !ledState;
    ledSet(ledState);
  }
}

// ======================= OTA 上传处理 =======================
void handleOtaUpload(){
  HTTPUpload &upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    Serial.printf("OTA Start: %s\n", upload.filename.c_str());
    otaInProgress = true;
    renderConnectivityLed();
    size_t sketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if(!Update.begin(sketchSpace)){
      Update.printError(Serial);
    }
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
      Update.printError(Serial);
    }
    renderConnectivityLed();
  } else if(upload.status == UPLOAD_FILE_END){
    if(Update.end(true)){
      Serial.printf("OTA Success: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
    renderConnectivityLed();
  }
  yield();
}
void handleOtaDone(){
  bool ok = !Update.hasError();
  otaInProgress = false;
  String resp = ok ? "{\"ok\":true,\"msg\":\"update success, rebooting\"}" : "{\"ok\":false,\"error\":\"update failed\"}";
  server.send(200, "application/json", resp);
  if(ok){
    delay(300);
    ESP.restart();
  }
}


// WiFi 连接与 AP 回退
String sanitizeId(String s, size_t maxLen){ String o=""; for(size_t i=0;i<s.length() && o.length()<maxLen;i++){ char c=s.charAt(i); if((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_') o+=c; } return o; }
void computeId(){ String mac=WiFi.macAddress(); mac.replace(":",""), deviceId=mac; computeMqttRootTopic(); }
void computeMqttRootTopic(){
  String typeSan = sanitizeId(deviceType.length()?deviceType:String("servo"), 16);
  String nameSan = sanitizeId(deviceName.length()?deviceName:String("servo"), 24);
  String macPart = deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId;
  mqttRootTopic = String("smarthome/") + typeSan + "/" + nameSan + "-" + macPart;
}
void connectWiFi(){
  WiFi.mode(WIFI_STA);
  String defaultName = String("esp-servo-") + (deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId);
  String hn = sanitizeId(deviceName.length()?deviceName:defaultName, 24);
  WiFi.hostname(hn.c_str());
  if(wifiSsid.length()){
    Serial.printf("Connecting WiFi SSID=%s\n", wifiSsid.c_str());
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    unsigned long st=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-st<15000){ delay(300); ESP.wdtFeed(); yield(); }
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP()); apMode=false;
  } else {
    Serial.println("WiFi failed, start AP for config...");
    WiFi.mode(WIFI_AP_STA);
    String apSsid = String("esp-servo-setup");
    WiFi.softAP(apSsid.c_str()); apMode=true; Serial.print("AP SSID: "); Serial.println(apSsid);
  }
}

// MQTT 集成：发布角度与接收设角
void mqttCallback(char* topic, byte* payload, unsigned int len){
  String t=String(topic);
  String pl=""; for(unsigned int i=0;i<len;i++) pl+=(char)payload[i];
  if(t==mqttRootTopic+"/set"){ int v = pl.toInt(); if(v<0) v=0; if(v>180) v=180; angle=v; sweepEnabled=false; servo.write(angle); Serial.printf("[MQTT] set angle=%d\n", v); }
  // 组合动作按钮：~/combo/<name>/press
  String comboPrefix = mqttRootTopic + "/combo/";
  if(t.startsWith(comboPrefix) && t.endsWith("/press")){
    int start = comboPrefix.length();
    int end = t.length() - String("/press").length();
    String name = t.substring(start, end);
    name.trim();
    uint8_t idx = findComboIndexByName(name);
    if(idx != 255){
      String script = comboScripts[idx];
      if(!startSequenceFromScript(script)){
        Serial.printf("[MQTT] combo '%s' invalid script\n", name.c_str());
      } else {
        Serial.printf("[MQTT] exec combo '%s'\n", name.c_str());
      }
    } else {
      Serial.printf("[MQTT] combo '%s' not found\n", name.c_str());
    }
  }
}
void mqttSetup(){ mqttClient.setServer(mqttHostStr.c_str(), mqttPortVal); mqttClient.setCallback(mqttCallback); }
void mqttPublishState(){
  if(!mqttClient.connected()) return;
  // 舵机角度
  String st = String(angle);
  mqttClient.publish((mqttRootTopic+"/angle").c_str(), st.c_str(), true);
  // WiFi 状态（SSID 与 RSSI），用于 HA 诊断传感器
  String ssid = WiFi.SSID();
  int rssi = WiFi.RSSI();
  if(WiFi.status()==WL_CONNECTED){
    mqttClient.publish((mqttRootTopic+"/wifi/ssid").c_str(), ssid.c_str(), true);
    String rssiStr = String(rssi);
    mqttClient.publish((mqttRootTopic+"/wifi/rssi").c_str(), rssiStr.c_str(), true);
  } else {
    mqttClient.publish((mqttRootTopic+"/wifi/ssid").c_str(), "offline", true);
    mqttClient.publish((mqttRootTopic+"/wifi/rssi").c_str(), "-100", true);
  }
}
void mqttEnsureConnected(){
  if(WiFi.status()!=WL_CONNECTED) return;
  if(mqttClient.connected()) return;
  unsigned long now=millis();
  if(now - lastMqttReconnectAttempt < 2000) return;
  lastMqttReconnectAttempt=now;
  String cid = String("servo-")+deviceId;
  Serial.println("MQTT connecting...");
  // 设置 LWT: topic ~/status = offline (retain)
  String willTopic = mqttRootTopic + String("/status");
  if(mqttClient.connect(cid.c_str(), mqttUser.length()?mqttUser.c_str():NULL, mqttPass.length()?mqttPass.c_str():NULL, willTopic.c_str(), 0, true, "offline")){
    // 上线状态
    mqttClient.publish(willTopic.c_str(), "online", true);
    mqttClient.subscribe((mqttRootTopic+"/set").c_str());
    mqttClient.subscribe((mqttRootTopic+"/combo/+/press").c_str());
    mqttPublishState();
    Serial.println("MQTT connected");
  }
}

// 配网 REST 接口
void handleGetState(){
  String j = String("{");
  j += "\"name\":\"" + deviceName + "\",";
  j += "\"type\":\"" + deviceType + "\",";
  j += "\"wifi\":{\"ssid\":\"" + WiFi.SSID() + "\",\"rssi\":" + String(WiFi.RSSI()) + "},";
  j += "\"mqtt\":{\"host\":\"" + mqttHostStr + "\",\"port\":" + String(mqttPortVal) + ",\"connected\":" + String(mqttClient.connected()?1:0) + "},";
  j += "\"topic\":\"" + mqttRootTopic + "\","; // 保留旧字段
  j += "\"topics\":{\"prefix\":\"" + mqttRootTopic + "\",\"angle_pub\":\"" + (mqttRootTopic+"/angle") + "\",\"set_sub\":\"" + (mqttRootTopic+"/set") + "\"},";
  j += "\"sequence\":{\"running\":" + String(seqRunning?1:0) + ",\"index\":" + String(seqIndex) + ",\"count\":" + String(seqCount) + "}";
  j += "}";
  server.send(200, "application/json", j);
}
void handleSetWifi(){
  if(!server.hasArg("ssid")){ Serial.println("[HTTP] /api/wifi_config missing ssid param"); server.send(400, "application/json", "{\"error\":\"missing ssid\"}"); return; }
  String s=server.arg("ssid"); s.trim(); String p=server.hasArg("pass")?server.arg("pass"):String(""); if(s.length()==0){ server.send(400, "application/json", "{\"error\":\"empty ssid\"}"); return; }
  Serial.printf("[HTTP] /api/wifi_config ssid=%s\n", s.c_str());
  wifiSsid=s; wifiPass=p; saveCfg();
  WiFi.softAPdisconnect(true); WiFi.disconnect(); delay(200); apMode=false; connectWiFi();
  mqttEnsureConnected();
  server.send(200, "application/json", "{\"ok\":true}");
}
void handleSetMqtt(){
  if(!server.hasArg("host")){ Serial.println("[HTTP] /api/mqtt_config missing host param"); server.send(400, "application/json", "{\"error\":\"missing host\"}"); return; }
  String h=server.arg("host"); h.trim(); String portStr=server.hasArg("port")?server.arg("port"):String(""); uint16_t p=portStr.length()? (uint16_t)portStr.toInt() : 1883; String u=server.hasArg("user")?server.arg("user"):String(""); String pw=server.hasArg("pass")?server.arg("pass"):String(""); if(h.length()==0){ server.send(400, "application/json", "{\"error\":\"empty host\"}"); return; }
  Serial.printf("[HTTP] /api/mqtt_config host=%s port=%u user=%s\n", h.c_str(), p, u.c_str());
  mqttHostStr=h; mqttPortVal=p; mqttUser=u; mqttPass=pw; saveCfg();
  if(mqttClient.connected()) mqttClient.disconnect(); mqttClient.setServer(mqttHostStr.c_str(), mqttPortVal); mqttEnsureConnected(); mqttPublishState();
  server.send(200, "application/json", "{\"ok\":true}");
}
void handleExecCombo(){
  String name = server.hasArg("name") ? server.arg("name") : String("");
  String script;
  if(server.hasArg("script")){
    script = server.arg("script");
  } else if(server.hasArg("a") || server.hasArg("b") || server.hasArg("d")){
    int a = server.hasArg("a") ? server.arg("a").toInt() : angle;
    int d = server.hasArg("d") ? server.arg("d").toInt() : 300;
    int b = server.hasArg("b") ? server.arg("b").toInt() : angle;
    script = String("A:") + String(a) + ";W:" + String(max(0,d)) + ";A:" + String(b);
  } else {
    server.send(400, "application/json", "{\"error\":\"missing script or a/d/b\"}"); return;
  }
  if(!startSequenceFromScript(script)){
    server.send(400, "application/json", "{\"error\":\"invalid script\"}"); return;
  }
  Serial.print("[COMBO] "); Serial.print(name); Serial.print(" script: "); Serial.println(script);
  server.send(200, "application/json", "{\"ok\":true}");
}
uint8_t findComboIndexByName(const String &name){
  for(uint8_t i=0;i<comboCount;i++){ if(comboNames[i] == name) return i; }
  return 255;
}
void handleGetCombos(){
  String j = String("{");
  j += "\"count\":" + String(comboCount) + ",\"combos\":[";
  for(uint8_t i=0;i<comboCount;i++){
    if(i) j += ",";
    j += "{\"name\":\"" + comboNames[i] + "\",\"script\":\"" + comboScripts[i] + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}
void handleSaveCombo(){
  if(!server.hasArg("name")) { server.send(400, "application/json", "{\"error\":\"missing name\"}"); return; }
  String name = server.arg("name"); name.trim();
  if(!name.length()){ server.send(400, "application/json", "{\"error\":\"empty name\"}"); return; }
  String script;
  if(server.hasArg("script")){
    script = server.arg("script");
  } else if(server.hasArg("a") || server.hasArg("b") || server.hasArg("d")){
    int a = server.hasArg("a") ? server.arg("a").toInt() : angle;
    int d = server.hasArg("d") ? server.arg("d").toInt() : 300;
    int b = server.hasArg("b") ? server.arg("b").toInt() : angle;
    script = String("A:") + String(a) + ";W:" + String(max(0,d)) + ";A:" + String(b);
  } else {
    server.send(400, "application/json", "{\"error\":\"missing script or a/d/b\"}"); return;
  }
  script.trim();
  // 验证脚本是否可解析
  if(!parseScript(script)){ server.send(400, "application/json", "{\"error\":\"invalid script\"}"); return; }
  uint8_t idx = findComboIndexByName(name);
  if(idx != 255){
    comboScripts[idx] = script;
  } else {
    if(comboCount >= MAX_COMBOS){ server.send(400, "application/json", "{\"error\":\"full\"}"); return; }
    comboNames[comboCount] = name;
    comboScripts[comboCount] = script;
    comboCount++;
  }
  saveCfg();
  // 保存后立即将该动作的 HA 实体发布到自动发现
  publishHaComboDiscovery(name);
  server.send(200, "application/json", "{\"ok\":true}");
}
void handleDeleteCombo(){
  if(!server.hasArg("name")) { server.send(400, "application/json", "{\"error\":\"missing name\"}"); return; }
  String name = server.arg("name");
  uint8_t idx = findComboIndexByName(name);
  if(idx == 255){ server.send(404, "application/json", "{\"error\":\"not found\"}"); return; }
  for(uint8_t i=idx;i+1<comboCount;i++){ comboNames[i]=comboNames[i+1]; comboScripts[i]=comboScripts[i+1]; }
  if(comboCount>0) comboCount--;
  saveCfg();
  // 同步清理该动作的 HA 自动发现
  clearHaDiscoveryCombo(name);
  server.send(200, "application/json", "{\"ok\":true}");
}
void handleSetDevice(){
  String name = server.hasArg("name")?server.arg("name"):String("");
  String type = server.hasArg("type")?server.arg("type"):String("");
  name.trim(); type.trim();
  if(!name.length() && !type.length()) Serial.println("[HTTP] /api/device no fields provided");
  Serial.printf("[HTTP] /api/device name=%s type=%s\n", name.c_str(), type.c_str());
  if(name.length()) deviceName = name;
  if(type.length()) deviceType = type;
  saveCfg();
  // 立即更新主机名（仅 STA 模式下）
  if(WiFi.getMode()!=WIFI_AP){
    String defaultName = String("esp-servo-") + (deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId);
    String hn = sanitizeId(deviceName.length()?deviceName:defaultName, 24);
    WiFi.hostname(hn.c_str());
  }
  // 重算 MQTT 主题并更新订阅
  computeMqttRootTopic();
  if(mqttClient.connected()){
    mqttClient.subscribe((mqttRootTopic+"/set").c_str());
    mqttPublishState();
  }
  server.send(200, "application/json", "{\"ok\":true}");
}
void handleConfig(){
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}
// ---------------- HA 自动发现 ----------------
void publishHaDiscovery(){
  if(!mqttClient.connected()) { mqttEnsureConnected(); }
  if(!mqttClient.connected()) { Serial.println("[HA] MQTT not connected, skip discovery"); return; }
  String macPart = deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId;
  String nodeId = sanitizeId(deviceType + String("-") + (deviceName.length()?deviceName:String("servo")) + String("-") + macPart, 48);
  // 统一的 device 信息（用于在 HA 中创建设备并归属实体）
  String dispName = deviceName.length() ? deviceName : (String("esp-servo-") + macPart);
  String deviceJson = String("\"device\":{\"identifiers\":[\"") + deviceId + "\"],\"name\":\"" + dispName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"servo_test\"}";

  // 滑杆（number）：角度控制
  {
    String objAngle = String("angle");
    String cfgTopicAngle = String("homeassistant/number/") + nodeId + "/" + objAngle + "/config";
    String cfg = String("{\"~\":\"") + mqttRootTopic + "\","; // base topic
    String angleName = (deviceName.length()?deviceName:String("servo")) + String("-angle");
    cfg += "\"name\":\"" + angleName + "\",";
    cfg += "\"uniq_id\":\"" + nodeId + String("_") + objAngle + "\",";
    cfg += "\"cmd_t\":\"~/set\",\"stat_t\":\"~/angle\",";
    cfg += "\"min\":0,\"max\":180,\"step\":1,";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += deviceJson;
    cfg += "}";
    mqttClient.publish(cfgTopicAngle.c_str(), cfg.c_str(), true);
  }

  // 已保存动作：为每个生成按钮
  for(uint8_t i=0;i<comboCount;i++){
    String name = comboNames[i];
    String objId = sanitizeId(name, 48);
    String cfgTopicBtn = String("homeassistant/button/") + nodeId + "/" + objId + "/config";
    String cfg = String("{\"~\":\"") + mqttRootTopic + "\","; // base topic
    String btnName = (deviceName.length()?deviceName:String("servo")) + String("-") + name;
    cfg += "\"name\":\"" + btnName + "\",";
    cfg += "\"uniq_id\":\"" + nodeId + String("_") + objId + "\",";
    cfg += "\"cmd_t\":\"~/combo/" + objId + "/press\",";
    cfg += "\"pl_prs\":\"PRESS\",";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += deviceJson;
    cfg += "}";
    mqttClient.publish(cfgTopicBtn.c_str(), cfg.c_str(), true);
  }
  // WiFi 诊断传感器：RSSI（dBm）与 SSID
  {
    String cfgTopicRssi = String("homeassistant/sensor/") + nodeId + "/wifi_rssi/config";
    String cfg = String("{\"~\":\"") + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + dispName + " WiFi 强度\",";
    cfg += "\"uniq_id\":\"" + nodeId + "_wifi_rssi\",";
    cfg += "\"stat_t\":\"~/wifi/rssi\",";
    cfg += "\"device_class\":\"signal_strength\",\"unit_of_measurement\":\"dBm\",";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += deviceJson;
    cfg += "}";
    mqttClient.publish(cfgTopicRssi.c_str(), cfg.c_str(), true);
  }
  {
    String cfgTopicSsid = String("homeassistant/sensor/") + nodeId + "/wifi_ssid/config";
    String cfg = String("{\"~\":\"") + mqttRootTopic + "\",";
    cfg += "\"name\":\"" + dispName + " WiFi SSID\",";
    cfg += "\"uniq_id\":\"" + nodeId + "_wifi_ssid\",";
    cfg += "\"stat_t\":\"~/wifi/ssid\",";
    cfg += "\"entity_category\":\"diagnostic\",";
    cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
    cfg += deviceJson;
    cfg += "}";
    mqttClient.publish(cfgTopicSsid.c_str(), cfg.c_str(), true);
  }
  mqttPublishState();
  Serial.println("[HA] Discovery published");
}

// 仅发布单个组合动作的 HA 自动发现按钮实体
void publishHaComboDiscovery(const String &name){
  if(!mqttClient.connected()) { mqttEnsureConnected(); }
  if(!mqttClient.connected()) { Serial.println("[HA] MQTT not connected, skip single combo discovery"); return; }
  String macPart = deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId;
  String nodeId = sanitizeId(deviceType + String("-") + (deviceName.length()?deviceName:String("servo")) + String("-") + macPart, 48);
  String dispName = deviceName.length() ? deviceName : (String("esp-servo-") + macPart);
  String deviceJson = String("\"device\":{\"identifiers\":[\"") + deviceId + "\"],\"name\":\"" + dispName + "\",\"model\":\"" + deviceType + "\",\"manufacturer\":\"ESP8266\",\"sw_version\":\"servo_test\"}";

  String objId = sanitizeId(name, 48);
  String cfgTopicBtn = String("homeassistant/button/") + nodeId + "/" + objId + "/config";
  String cfg = String("{\"~\":\"") + mqttRootTopic + "\","; // base topic
  String btnName = (deviceName.length()?deviceName:String("servo")) + String("-") + name;
  cfg += "\"name\":\"" + btnName + "\",";
  cfg += "\"uniq_id\":\"" + nodeId + String("_") + objId + "\",";
  cfg += "\"cmd_t\":\"~/combo/" + objId + "/press\",";
  cfg += "\"pl_prs\":\"PRESS\",";
  cfg += "\"avty_t\":\"~/status\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",";
  cfg += deviceJson;
  cfg += "}";
  mqttClient.publish(cfgTopicBtn.c_str(), cfg.c_str(), true);
  Serial.printf("[HA] Published combo discovery: %s\n", name.c_str());
}
// 清理 HA 自动发现：向 config 主题发布空保留消息以删除旧实体
void clearHaDiscovery(){
  if(!mqttClient.connected()) { mqttEnsureConnected(); }
  if(!mqttClient.connected()) { Serial.println("[HA] MQTT not connected, skip clear"); return; }
  String macPart = deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId;
  String nodeId = sanitizeId(deviceType + String("-") + (deviceName.length()?deviceName:String("servo")) + String("-") + macPart, 48);
  // 删除角度 number
  {
    String objAngle = String("angle");
    String cfgTopicAngle = String("homeassistant/number/") + nodeId + "/" + objAngle + "/config";
    mqttClient.publish(cfgTopicAngle.c_str(), "", true);
  }
  // 删除所有按钮
  for(uint8_t i=0;i<comboCount;i++){
    String name = comboNames[i];
    String objId = sanitizeId(name, 48);
    String cfgTopicBtn = String("homeassistant/button/") + nodeId + "/" + objId + "/config";
    mqttClient.publish(cfgTopicBtn.c_str(), "", true);
  }
  // 删除 WiFi 诊断传感器
  {
    String cfgTopicRssi = String("homeassistant/sensor/") + nodeId + "/wifi_rssi/config";
    mqttClient.publish(cfgTopicRssi.c_str(), "", true);
  }
  {
    String cfgTopicSsid = String("homeassistant/sensor/") + nodeId + "/wifi_ssid/config";
    mqttClient.publish(cfgTopicSsid.c_str(), "", true);
  }
  Serial.println("[HA] Discovery cleared");
}

// 清理单个组合动作的 HA 发现
void clearHaDiscoveryCombo(const String &name){
  if(!mqttClient.connected()) { mqttEnsureConnected(); }
  if(!mqttClient.connected()) { Serial.println("[HA] MQTT not connected, skip clear combo"); return; }
  String macPart = deviceId.length()>=6 ? deviceId.substring(deviceId.length()-6) : deviceId;
  String nodeId = sanitizeId(deviceType + String("-") + (deviceName.length()?deviceName:String("servo")) + String("-") + macPart, 48);
  String objId = sanitizeId(name, 48);
  String cfgTopicBtn = String("homeassistant/button/") + nodeId + "/" + objId + "/config";
  mqttClient.publish(cfgTopicBtn.c_str(), "", true);
  Serial.printf("[HA] Cleared combo discovery: %s\n", name.c_str());
}

// HTTP: /api/ha_clear_combo?name=<动作名称>
void handleHaClearCombo(){
  if(!server.hasArg("name")){
    server.send(400, "application/json", "{\"error\":\"missing name\"}");
    return;
  }
  String name = server.arg("name"); name.trim();
  if(!name.length()){
    server.send(400, "application/json", "{\"error\":\"empty name\"}");
    return;
  }
  clearHaDiscoveryCombo(name);
  String resp = String("{\"ok\":true,\"name\":\"") + name + String("\"}");
  server.send(200, "application/json", resp);
}
void handlePublishHa(){
  bool doReset = server.hasArg("reset") && server.arg("reset") != String("0");
  if(doReset) clearHaDiscovery();
  publishHaDiscovery();
  String resp = String("{\"ok\":true,\"reset\":") + String(doReset?1:0) + String("}");
  server.send(200, "application/json", resp);
}