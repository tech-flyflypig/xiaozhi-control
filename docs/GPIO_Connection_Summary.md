# IoT设备GPIO连接总结

本文档总结了小智控制系统中所有IoT设备的GPIO引脚连接配置。

## 目录
- [传感器设备](#传感器设备)
- [控制设备](#控制设备)
- [显示和指示设备](#显示和指示设备)
- [通信接口](#通信接口)
- [GPIO冲突检查](#gpio冲突检查)

---

## 传感器设备

### 1. DHT11温湿度传感器
- **设备名称**: `DHT11Sensor`
- **GPIO引脚**: `GPIO_NUM_12`
- **接口类型**: 数字IO（单总线协议）
- **功能**: 温度和湿度检测
- **读取间隔**: 5秒
- **连接说明**: 
  - VCC → 3.3V
  - GND → GND
  - DATA → GPIO12

### 2. 光照强度传感器
- **设备名称**: `LightSensor`
- **GPIO引脚**: `GPIO_NUM_1` (ADC1_CH0)
- **接口类型**: ADC模拟输入
- **功能**: 光照强度检测
- **读取间隔**: 5秒
- **连接说明**:
  - 光敏电阻一端 → 3.3V
  - 光敏电阻另一端 → GPIO1 + 10kΩ下拉电阻到GND

### 3. 雨量传感器
- **设备名称**: `RainSensor`
- **GPIO引脚**: `GPIO_NUM_8` (ADC1_CH7)
- **接口类型**: ADC模拟输入
- **功能**: 雨量强度检测
- **读取间隔**: 5秒
- **连接说明**:
  - VCC → 3.3V
  - GND → GND
  - AO → GPIO8

### 4. 甲醛传感器
- **设备名称**: `HCHOSensor`
- **GPIO引脚**: 
  - TX: `GPIO_NUM_17`
  - RX: `GPIO_NUM_18`
- **接口类型**: UART1 (9600波特率)
- **功能**: 甲醛浓度检测
- **读取间隔**: 5秒
- **连接说明**:
  - VCC → 5V
  - GND → GND
  - TX → GPIO18 (ESP32 RX)
  - RX → GPIO17 (ESP32 TX)

---

## 控制设备

### 5. 智能风扇
- **设备名称**: `Fan`
- **GPIO引脚**: 
  - ESP32: `GPIO_NUM_12`
  - ESP32-S3: `GPIO_NUM_38`
- **接口类型**: 数字输出
- **功能**: 风扇开关控制
- **控制逻辑**: 高电平开启，低电平关闭
- **连接说明**:
  - 控制端 → GPIO引脚
  - 电源通过继电器或MOSFET控制

### 6. 有源蜂鸣器
- **设备名称**: `Buzzer`
- **GPIO引脚**: `GPIO_NUM_14`
- **接口类型**: 数字输出
- **功能**: 报警声音输出
- **控制逻辑**: 高电平触发（高电平开启，低电平关闭）
- **连接说明**:
  - 正极 → GPIO14
  - 负极 → GND

### 7. 窗户控制器（步进电机）
- **设备名称**: `window_controller`
- **GPIO引脚**:
  - IN1: `GPIO_NUM_19`
  - IN2: `GPIO_NUM_20`
  - IN3: `GPIO_NUM_10`
  - IN4: `GPIO_NUM_11`
- **接口类型**: 数字输出（4线步进电机）
- **功能**: 窗户开关控制
- **电机型号**: 28BYJ-48步进电机 + ULN2003驱动板
- **步数配置**: 2048步/圈
- **连接说明**:
  - ULN2003驱动板IN1-IN4分别连接对应GPIO
  - 电机电源独立供电（5V推荐）

---

## 显示和指示设备

### 8. 三色LED灯
- **设备名称**: `3clight`
- **GPIO引脚**:
  - 红色: `GPIO_NUM_3`
  - 绿色: `GPIO_NUM_46`
  - 蓝色: `GPIO_NUM_9`
- **接口类型**: 数字输出
- **功能**: 状态指示和氛围灯
- **支持状态**: 红、绿、蓝、白光、关闭（5种状态）
- **连接说明**:
  - 每个LED通过限流电阻连接到对应GPIO
  - 共阴极接GND

---

## 通信接口

### MQTT数据格式
所有IoT设备通过MQTT进行数据交换：

**发布数据格式**:
```json
{
  "HCHO": 20,
  "TEMP": 2402,
  "HUMI": 5900,
  "LX": 95,
  "RAIN": 15,
  "FAN": true,
  "WINDOW": false,
  "LIGHT": "red"
}
```

**控制命令格式**:
```json
{
  "fan": "on",
  "window": "off",
  "alarm": "on",
  "light": "blue"
}
```

---

## GPIO冲突检查

### 已使用的GPIO引脚
| GPIO | 设备 | 功能 | 接口类型 |
|------|------|------|----------|
| GPIO1 | 光照传感器 | ADC输入 | ADC1_CH0 |
| GPIO3 | 三色灯(红) | 数字输出 | GPIO |
| GPIO8 | 雨量传感器 | ADC输入 | ADC1_CH7 |
| GPIO9 | 三色灯(蓝) | 数字输出 | GPIO |
| GPIO10 | 窗户控制器(IN3) | 数字输出 | GPIO |
| GPIO11 | 窗户控制器(IN4) | 数字输出 | GPIO |
| GPIO12 | DHT11传感器 | 数字IO | GPIO |
| GPIO14 | 蜂鸣器 | 数字输出 | GPIO |
| GPIO17 | 甲醛传感器(TX) | UART发送 | UART1_TX |
| GPIO18 | 甲醛传感器(RX) | UART接收 | UART1_RX |
| GPIO19 | 窗户控制器(IN1) | 数字输出 | GPIO |
| GPIO20 | 窗户控制器(IN2) | 数字输出 | GPIO |
| GPIO38 | 风扇(S3) | 数字输出 | GPIO |
| GPIO46 | 三色灯(绿) | 数字输出 | GPIO |

### 注意事项
1. **ADC冲突**: GPIO1和GPIO8都使用ADC1，已通过共享ADC句柄解决
2. **UART冲突**: GPIO17/18被甲醛传感器占用，窗户控制器已避开这些引脚
3. **ESP32/ESP32-S3差异**: 风扇控制器根据芯片类型使用不同GPIO
4. **蜂鸣器类型**: 使用有源蜂鸣器，简单GPIO控制，高电平触发

### 可用GPIO引脚
以下GPIO引脚当前未被IoT设备使用，可用于扩展：
- GPIO2, GPIO4, GPIO5, GPIO6, GPIO7
- GPIO13, GPIO15, GPIO16, GPIO21
- GPIO35, GPIO36, GPIO37, GPIO39 (仅输入)

---

## 板子兼容性

当前配置主要针对 `bread-compact-wifi-lcd` 板子进行优化，其他板子可能需要根据具体硬件调整GPIO配置。

### 修改GPIO配置
如需修改GPIO配置，请编辑对应设备的源文件：
- DHT11: `main/iot/things/dht11_sensor.cc`
- 光照传感器: `main/iot/things/light_sensor.cc`
- 雨量传感器: `main/iot/things/rain_sensor.cc`
- 甲醛传感器: `main/iot/things/hcho_sensor.cc`
- 风扇: `main/iot/things/fan.cc`
- 蜂鸣器: `main/iot/things/buzzer.cc`
- 窗户控制器: `main/iot/things/window_controller.cc`
- 三色灯: `main/iot/things/3clight.cc`

---

*最后更新: 2024年12月*
