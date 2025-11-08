# ESP8266 双路继电器开关（switch_example）使用说明

本说明适用于本仓库中的 `switch_example` 固件，基于 ESP8266（NodeMCU v2 等）与 Arduino 框架。固件提供 2 路继电器控制、可配置的 LED 开/关颜色与亮度、白天阈值(daytime)联动逻辑，以及与 Home Assistant 的 MQTT 集成（自动发现）。

## 设备特性
- 2 路继电器通道（`CH 1`、`CH 2`），支持开/关/切换
- LED 显示：
  - “打开态”与“关闭态”各自独立的 RGB 颜色与亮度（0–255）
  - Home Assistant 中以两个 `light` 实体暴露：`LED On` 与 `LED Off`
- Daytime 白天阈值：
  - 从外部照度传感器 MQTT 主题读取照度
  - 当照度值大于阈值且通道关闭时，关闭态 LED 可选择“熄灭”以减少刺眼（见逻辑说明）
  - 阈值范围 `0..2000`，可在 HA 中通过 `number` 实体直接调节
- 窗帘互斥模式：打开一侧通道时自动关闭另一侧（`CH 0` 与 `CH 1`）
- MQTT 与 WiFi 参数可通过网页配置，并保存在 EEPROM（掉电不丢失）

## 上电与网络配置
- 正常启动：设备尝试以已保存的 WiFi 凭据连接路由器；连接成功后再连接 MQTT。
- 断线与 AP 配网：长时间无法连接 WiFi 时，设备会开启 AP 配网模式，SSID 基于设备名并带有 `-setup` 后缀（如 `esp-switch-setup`）。可连接该 AP 后访问默认网关页面（通常为 `http://192.168.4.1/`）进行配置。

## 网页管理页面（`GET /`）
- 展示状态：各通道开关状态、互斥模式、设备信息、WiFi 和 MQTT 信息、Daytime 阈值与最近照度
- 操作控件：
  - 通道：切换/开/关
  - LED 打开/关闭颜色选择器与亮度滑条
  - Daytime 阈值输入（`0..2000`）与保存按钮
  - 互斥模式勾选、设备名/类型、WiFi/MQTT 参数设置
  - “重发 HA 发现”按钮（手动发布自动发现配置）

## REST API 列表
- `GET /api/state`：返回整体状态 JSON（通道、LED 配置、daytime、illum 等）
- `GET /api/toggle?ch=<0|1>`：切换指定通道状态
- `GET /api/on?ch=<0|1>`：打开指定通道
- `GET /api/off?ch=<0|1>`：关闭指定通道
- `GET /api/mode?curtain=<0|1|true|false|on|off>`：设置互斥窗帘模式
- `GET /api/led?color=<#RRGGBB>&bright=<0..255>`：设置“打开态” LED 的颜色与亮度
- `GET /api/led_off?color=<#RRGGBB>&bright=<0..255>`：设置“关闭态” LED 的颜色与亮度
- `GET /api/daytime?value=<0..2000>`：设置 Daytime 阈值（同时发布到 MQTT 状态主题）
- `GET /api/device?name=<name>&type=<type>`：设置设备名与类型并更新 MQTT 根主题
- `GET /api/wifi_config?ssid=<ssid>&pass=<pass>`：更新 WiFi 凭据并尝试重连
- `GET /api/mqtt_config?host=<ip|host>&port=<1..65535>&user=<u>&pass=<p>`：更新 MQTT 参数并尝试重连
- `GET /api/ha_discovery`：手动发布 Home Assistant 自动发现配置（推荐在 HA 中重新加载 MQTT 后调用）

## MQTT 主题结构
- 根主题：`smarthome/<deviceType>/<deviceName>-<MAC_NO_COLON>`
  - 示例：`smarthome/switch/esp-switch-5CCF7F02E3D8`
- 主题说明：
  - `~/status`：LWT 在线状态（`online`/`offline`，retain）
  - `~/state`：整体状态 JSON（retain）
  - `~/cmd/#`：通道控制命令
    - `~/cmd/<channel>`：payload 为 `on`/`off`/`toggle`
  - `~/led_on/set` / `~/led_on/state`：`LED On` Light 的命令与状态（JSON，retain）
  - `~/led_off/set` / `~/led_off/state`：`LED Off` Light 的命令与状态（JSON，retain）
  - `~/daytime/set` / `~/daytime/state`：Daytime 阈值（纯数字，retain）

### Light 命令与状态（JSON schema）
- 命令载荷示例（`~/led_on/set` 或 `~/led_off/set`）：
  ```json
  {"brightness": 128, "color": {"r": 0, "g": 255, "b": 0}}
  ```
- 状态载荷示例（设备发布到 `~/.../state`）：
  ```json
  {"state":"ON","brightness":128,"color_mode":"rgb","color":{"r":0,"g":255,"b":0}}
  ```

### Daytime 阈值命令与状态
- 设定：向 `~/daytime/set` 发送纯数字字符串，设备会自动限幅到 `0..2000`
- 状态：`~/daytime/state` 为纯数字（retain），用于 HA number 实体同步

## Home Assistant 自动发现
- 发现仅在手动触发时发布：访问 `GET /api/ha_discovery`
- 生成的实体：
  - `switch`：`CH 1` 与 `CH 2`
  - `light`（JSON schema）：`LED On` 与 `LED Off`（支持亮度与 RGB 颜色）
  - `number`：`Daytime 阈值`（范围 `0..2000`，滑条）
- 如果曾经出现旧的“传感器”实体，本固件已在发现流程中发送空配置（retain）进行清理；重新加载 MQTT 后即会消失。

## Daytime 与照度逻辑
- 设备订阅照度传感器主题：`smarthome/sensor/outloor_illumination`
- 当收到照度值 `illuminationValue` 后：
  - 判断 `illuminationValue > daytimeThreshold` 视为白天
  - 在白天且通道关闭时，LED 根据关闭态策略进行渲染（可选择熄灭或显示关闭颜色，取决于逻辑与亮度设置）
- 如需更改照度传感器主题，请修改固件中的常量 `SENSOR_ILLUM_TOPIC` 并重新编译刷写。

## EEPROM 与持久化
- EEPROM 大小 `512` 字节；配置版本 `4`
- 持久化字段：LED 打开/关闭颜色与亮度、设备名与类型、WiFi 与 MQTT 配置、Daytime 阈值等
- 载入时会对 `daytimeThreshold` 做上限保护（大于 `2000` 则压到 `2000`）

## 故障排查
- 看不到 HA 实体：
  - 在 HA 中“设置 → 设备与服务 → 重新加载 MQTT”，然后访问设备 `GET /api/ha_discovery`
  - 确认 HA 与设备的 MQTT Broker 可达，账号密码正确
- Light 不显示颜色或亮度控件：
  - 确认实体来自 JSON schema，状态中包含 `"color_mode":"rgb"`
- Daytime 不生效或范围异常：
  - 检查 `~/daytime/state` 是否为期望值
  - 检查照度主题 `smarthome/sensor/outloor_illumination` 是否有有效数值（纯数字）
- 无法连接 WiFi：
  - 设备会进入 AP 配网模式（SSID 以设备名为前缀并带 `-setup`），连接后在网页里配置正确的 SSID/密码
- 修改设备名或类型后：
  - 根主题会改变；请在 HA 中重新加载 MQTT，并重新触发自动发现

## OTA 升级与 LED 指示
- 网页端支持 OTA：在主页底部选择 `.bin` 固件并点击“开始升级”，设备接收后写入并自动重启。
- 升级进行中：设备所有 NeoPixel LED 以“绛紫色”闪烁，闪烁周期约 `300ms`，用于清晰指示正在升级。
- 升级成功：页面返回 `{"ok":true}`，约 0.3 秒后设备重启；升级失败则返回错误信息并停止闪烁。
- REST 接口：也可直接 `POST /api/ota` 上传固件文件（`multipart/form-data`），无须额外字段名要求（取首个文件）。

## 版本与许可
- 固件标识：`switch_example`
- 平台：ESP8266 + Arduino
- 如需扩展（如合并双灯为一个实体、支持色温/渐变等），欢迎在此文档基础上提出需求。